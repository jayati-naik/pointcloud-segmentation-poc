
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/time.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/filter_indices.h>

int main(int argc, char **argv)
{
    // Read input pcd file into cloud data
    pcl::PCDReader reader;
    // Write input pcd file into cloud data
    pcl::PCDWriter writer;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>),
        cloud_f(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    reader.read("../Data/setup_unorg.pcd", *cloud);
    *cloud_filtered = *cloud;
    double start = pcl::getTime();

    // Create the segmentation object for the planar model and set all the
    // parameters
    pcl::SACSegmentation<pcl::PointXYZRGBNormal> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane(
        new pcl::PointCloud<pcl::PointXYZRGBNormal>());

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.005);
    seg.setInputCloud(cloud_filtered);
    seg.segment(*inliers, *coefficients);

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers);

    // Remove the planar inliers, extract the rest
    extract.setNegative(true);
    extract.filter(*cloud_filtered);

    std::cerr << "Floor Plane Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    Eigen::Matrix<float, 1, 3> floor_plane_normal_vector, xy_plane_normal_vector, rotation_vector;

    floor_plane_normal_vector[0] = coefficients->values[0];
    floor_plane_normal_vector[1] = coefficients->values[1];
    floor_plane_normal_vector[2] = coefficients->values[2];

    std::cout << floor_plane_normal_vector << std::endl;

    xy_plane_normal_vector[0] = 0.0;
    xy_plane_normal_vector[1] = 0.0;
    xy_plane_normal_vector[2] = 1.0;

    std::cout << xy_plane_normal_vector << std::endl;

    rotation_vector = xy_plane_normal_vector.cross(floor_plane_normal_vector);
    std::cout << "Rotation Vector: " << rotation_vector << std::endl;

    float theta = -atan2(rotation_vector.norm(), xy_plane_normal_vector.dot(floor_plane_normal_vector));

    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    // transform_2.translation() << 0, 0, 2.5;
    transform_2.rotate(Eigen::AngleAxisf(theta, rotation_vector));
    std::cout << "Transformation matrix: " << std::endl
              << transform_2.matrix() << std::endl;

    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_2);

    double end = pcl::getTime();
    std::cout << "Duration: " << double(end - start) << " ms" << std::endl;

    std::stringstream ss;
    ss << "../Data/setup_rotated.pcd";
    writer.write<pcl::PointXYZRGBNormal>(ss.str(), *transformed_cloud, false);

    // Visualize both the original and the result.
    pcl::visualization::PCLVisualizer viewer(argv[1]);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler_original(cloud, 0, 255, 0);
    viewer.addPointCloud(cloud, colorHandler_original, "original");

    // The transformed one's points will be red in color.
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorHandler(transformed_cloud, 255, 0, 0);
    viewer.addPointCloud(transformed_cloud, colorHandler, "transformed");

    // Add 3D colored axes to help see the transformation.
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}