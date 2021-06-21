#include "object_segmentation.hpp"

ObjectSegmentation::ObjectSegmentation()
{
    cam1_scene_cloud_ptr_ = PointCloudT::Ptr(new PointCloudT);
    cam1_scene_cloud_ptr_filtered_ = PointCloudT::Ptr(new PointCloudT);
    box_filtered_cloud_ptr = PointCloudT::Ptr(new PointCloudT);
    outlier_removal_output_cloud_ptr = PointCloudT::Ptr(new PointCloudT);

    cam1_scene_cloud_ptr_clusters_colored = pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    normals = pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
    tree = pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
}

void ObjectSegmentation::loadInputPCD() {
    // Read input pcd file into cloud data
    pcl::PCDReader reader;
    reader.read("../data/delta1_normals.pcd", *cam1_scene_cloud_ptr_);
}

void ObjectSegmentation::applyBoxFilter()
{    
    float minX = -0.15, minY = -0.5, minZ = -0.7;
    float maxX = +0.16, maxY = +0.5, maxZ = +1.1;

    // Using a CropBox filter to extract the region of interest from the camera scene.
    pcl::CropBox<pcl::PointXYZRGBNormal> box_filter;
    std::cout<<"Input Point Cloud has: "<<cam1_scene_cloud_ptr_->points.size()<<std::endl;
    box_filter.setInputCloud(cam1_scene_cloud_ptr_);
    box_filter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
    box_filter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
    box_filter.filter(*box_filtered_cloud_ptr);
    std::cout<<"The box-filtered point cloud has: "<<box_filtered_cloud_ptr->points.size()<<std::endl;
}

void ObjectSegmentation::applyStatisticalOutlierRemovalFilter(){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
    sor.setInputCloud (box_filtered_cloud_ptr);
    sor.setMeanK (50);
    sor.setStddevMulThresh (5);
    sor.filter (*outlier_removal_output_cloud_ptr);
    
}

void ObjectSegmentation::applyPlaneSegmentation()
{ 
    // Normal Estimation
    pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod(tree);

    normal_estimator.setInputCloud(outlier_removal_output_cloud_ptr);
    normal_estimator.setKSearch(50);
    normal_estimator.compute(*normals);

    // Create the segmentation object for the planar model and set all theparameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    // seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.07);
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setInputCloud(outlier_removal_output_cloud_ptr);
    seg.setInputNormals(normals);
    seg.setNormalDistanceWeight(0.00000999);
    seg.segment(*inliers, *coefficients);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setInputCloud(outlier_removal_output_cloud_ptr);
    extract.setIndices(inliers);

    // Get the points associated with the planar surface
    extract.filter(*cloud_plane);
    std::cout << "PointCloud representing the planar component: "
                << cloud_plane->size() << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cam1_scene_cloud_ptr_filtered_);
    std::cout << "PointCloud after removing planar component: "
                << cam1_scene_cloud_ptr_filtered_->size() << " data points." << std::endl;

}


void ObjectSegmentation::applyRegionGrowingSegmentation(){  // Region Growing RGB object detection
  
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cam1_scene_cloud_ptr_filtered_, *indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGBNormal, pcl::Normal> reg;
    reg.setInputCloud(cam1_scene_cloud_ptr_filtered_);
    reg.setIndices(indices);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(20);
    reg.setPointColorThreshold(6);
    reg.setRegionColorThreshold(5);
    reg.setMinClusterSize(1000);
    reg.setInputNormals(normals);
    reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
    // reg.setCurvatureThreshold(0);

    std::vector<pcl::PointIndices> cam1_scene_cloud_ptr_clusters_;
    reg.extract(cam1_scene_cloud_ptr_clusters_);

    cam1_scene_cloud_ptr_clusters_colored = reg.getColoredCloud();
}

void ObjectSegmentation::visualiseProcessed3DImage(){

     // Initialize visualiser and render results from each step
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(255, 255, 255, 0);
    
    // Render original point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler_original(cam1_scene_cloud_ptr_, 0, 255, 0);
    viewer.addPointCloud(cam1_scene_cloud_ptr_, colorHandler_original, "original");

    // The transformed one's points will be red in color.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler_box(box_filtered_cloud_ptr, 255, 0, 0);
    viewer.addPointCloud(box_filtered_cloud_ptr, colorHandler_box, "Crop Box");

    // Removed Outliers will be black in color.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler_sor(outlier_removal_output_cloud_ptr, 255, 255, 51);
    viewer.addPointCloud(outlier_removal_output_cloud_ptr, colorHandler_sor, "sor");

    // The transformed one's points will be white in color.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler_plane(cam1_scene_cloud_ptr_filtered_, 255, 0, 127);
    viewer.addPointCloud(cam1_scene_cloud_ptr_filtered_, colorHandler_plane, "Plane segmentation");

    // Region Growing clustering colored clouds
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> colorHandler_clouds(cam1_scene_cloud_ptr_clusters_colored, 0, 0, 255);
    viewer.addPointCloud(cam1_scene_cloud_ptr_clusters_colored, colorHandler_clouds, "Color Cloud");

    // Add 3D colored axes to help see the transformation.
    // viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}


int main(int argc, char** argv)
{
    ObjectSegmentation objSeg;

    // Start the timer
    double start = pcl::getTime();

    // Load Point CLoud Data
    objSeg.loadInputPCD();

    // Read points from the specified Bounding box.
    objSeg.applyBoxFilter();

    // Statistical Outlier Removal
    objSeg.applyStatisticalOutlierRemovalFilter();

    // objSeg.box_filtered_cloud_ptr = objSeg.cam1_scene_cloud_ptr_;
    objSeg.applyPlaneSegmentation();

    // Region Growing Segentation
    objSeg.applyRegionGrowingSegmentation();

    // Calculate duration
    double end = pcl::getTime();
    std::cout << "Duration: " << double(end - start) << " ms" << std::endl;

    // Visulaize opint lcoud components
    objSeg.visualiseProcessed3DImage();
    
}