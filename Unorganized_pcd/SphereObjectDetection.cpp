#include <chrono>
#include <pcl/ModelCoefficients.h>
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
#include <thread>

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  // Read input pcd file into cloud data
  pcl::PCDReader reader;
  // Write input pcd file into cloud data
  pcl::PCDWriter writer;

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>), cloud_f(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  reader.read("setup_unorg.pcd", *cloud);

  /*
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  // Output has the PointNormal type in order to store the normals calculated by
  MLS pcl::PointCloud<pcl::PointNormal> mls_points;

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZRGBNormal, pcl::PointNormal> mls;

  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);

  // Reconstruct
  mls.process (mls_points);

  // Save output
  pcl::io::savePCDFile ("bun0-mls.pcd", mls_points);
  */

  /*
  // Create the filtering object: downsample the dataset using a leaf size of
  1cm pcl::VoxelGrid<pcl::PointXYZRGB> vg; vg.setInputCloud (cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  */

  // Create the segmentation object for the planar model and set all the
  // parameters
  pcl::SACSegmentationFromNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_plane(
      new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_NORMAL_SPHERE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(700);
  seg.setDistanceThreshold(0.01);
  seg.setInputCloud(cloud);
  seg.setInputNormals(cloud);
  seg.setOptimizeCoefficients(true);
  seg.setRadiusLimits(0.1, 0.15);
  seg.setEpsAngle(15 / (180/3.141592654));
  seg.setMaxIterations(1000000);

  seg.segment(*inliers, *coefficients);

  // Extract the planar inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);

  // Get the points associated with the planar surface
  extract.filter(*cloud_plane);
  std::cout << "PointCloud representing the planar component: "
            << cloud_plane->size() << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative(true);
  extract.filter(*cloud_f);
  *cloud = *cloud_f;
  std::cout << "PointCloud after removing planar component: " << cloud->size()
            << " data points." << std::endl;

  std::stringstream ss2;
  ss2 << "object_WO_PLANAR.pcd";
  writer.write<pcl::PointXYZRGBNormal>(ss2.str(), *cloud, false);

  // Region Growing RGB object detection
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::IndicesPtr indices(new std::vector<int>);
  pcl::removeNaNFromPointCloud(*cloud, *indices);

  pcl::RegionGrowingRGB<pcl::PointXYZRGBNormal> reg;
  reg.setInputCloud(cloud);
  reg.setIndices(indices);
  reg.setSearchMethod(tree);
  reg.setDistanceThreshold(1);
  reg.setPointColorThreshold(6);
  reg.setRegionColorThreshold(5);
  reg.setMinClusterSize(600);

  std::vector<pcl::PointIndices> clusters;
  reg.extract(clusters);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud();
  // Visualize colored clouds
  pcl::visualization::PCLVisualizer viewer("PCL Viewer");
  viewer.addPointCloud<pcl::PointXYZRGB>(colored_cloud);
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }

  return (0);
}
