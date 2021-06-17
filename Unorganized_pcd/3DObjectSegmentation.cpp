#include <chrono>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/time.h>
#include <pcl/console/parse.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h> // for pcl::removeNaNFromPointCloud
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <thread>

using namespace std::chrono_literals;

typedef pcl::PointXYZRGB PointT;

int main(int argc, char **argv)
{
  // Read input pcd file into cloud data
  pcl::PCDReader reader;
  // Write input pcd file into cloud data
  pcl::PCDWriter writer;

  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  reader.read("../Data/delta1_normals.pcd", *cloud);
  *cloud_filtered = *cloud;
  double start = pcl::getTime();


  // Create the filtering object: downsample the dataset using a leaf size of
  // 1cm
  pcl::VoxelGrid<pcl::PointXYZRGBNormal> vg; vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);

  /*
  // Statistical Outlier Removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormal> sor;
  sor.setInputCloud(cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(0);
  sor.filter(*cloud_filtered);

  // Inliers
  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  writer.write<pcl::PointXYZRGBNormal>("filter1.pcd", *cloud_filtered, false);

  //Outliers
  sor.setNegative(true);
  sor.filter(*cloud_filtered);
  writer.write<pcl::PointXYZRGBNormal>("filter2.pcd", *cloud_filtered, false);
  */
  // Normal Estimation
  
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZRGBNormal, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);

  normal_estimator.setInputCloud(cloud_filtered);
  normal_estimator.setKSearch(50);
  normal_estimator.compute(*normals);

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::Normal> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(10000);
  seg.setDistanceThreshold(0.11);
  seg.setAxis(Eigen::Vector3f::UnitZ());
  seg.setInputCloud(cloud_filtered);
  seg.setInputNormals(normals);
  seg.setNormalDistanceWeight(0.09);
  seg.segment(*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);

  // Get the points associated with the planar surface
  extract.filter(*cloud_plane);
  std::cout << "PointCloud representing the planar component: "
            << cloud_plane->size() << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);
  *cloud_filtered = *cloud_f;
  std::cout << "PointCloud after removing planar component: "
            << cloud_filtered->size() << " data points." << std::endl;

  std::stringstream ss2;
  ss2 << "object_WO_PLANAR.pcd";
  writer.write<pcl::PointXYZRGBNormal>(ss2.str(), *cloud_filtered, false);

  // Region Growing RGB object detection
  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*cloud_filtered, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  reg.setInputCloud(cloud_filtered);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(0);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  // reg.setMinClusterSize(100);
  // reg.setInputNormals(normals);
  // reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI);
  // reg.setCurvatureThreshold(1.0);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();

  double end = pcl::getTime();
  std::cout << "Duration: " << double(end - start) << " ms" << std::endl;

  // Visualize colored clouds
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud);

  // Add 3D colored axes to help see the transformation.
  // viewer.addCoordinateSystem();
  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
  }

  return (0);
}
