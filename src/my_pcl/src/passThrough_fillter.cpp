#include <ros/ros.h>

#include <iostream>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// #include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  ///PassThrough filter declare
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_PassThrough_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // pcl::PointCloud<pcl::PointXYZ>::Ptr indices_rem(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr indices_x(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr indices_xz(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr indices_xyz(new pcl::PointCloud<pcl::PointXYZ>);

  // pcl::PointCloud<pcl::PointXYZ> indices_rem;

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Perform the actual filtering
  // pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  // sor.setInputCloud (cloudPtr);
  // sor.setLeafSize (0.01, 0.01, 0.01);
  // sor.filter (cloud_filtered);

  //Convert to pcl::PointCloud<pcl::pointxyz>::Ptr
  pcl::fromPCLPointCloud2(*cloud,*cloud_point);

  //////////////////////////////////////////PassThrough filter
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud_point);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.1, 1000.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_PassThrough_filtered);

  // pcl::PassThrough<pcl::PointXYZ> ptfilter (true); // Initializing with true will allow us to extract the removed indices
  // ptfilter.setInputCloud (cloud_point);
  // ptfilter.setFilterFieldName ("x");
  // ptfilter.setFilterLimits (0.0, 1000.0);
  // ptfilter.filter (*indices_x);
  // // The indices_x array indexes all points of cloud_in that have x between 0.0 and 1000.0
  // indices_rem = ptfilter.getRemovedIndices ();
  // // The indices_rem array indexes all points of cloud_in that have x smaller than 0.0 or larger than 1000.0
  // // and also indexes all non-finite points of cloud_in
  // // ptfilter.setIndices (indices_x);
  // ptfilter.setFilterFieldName ("z");
  // ptfilter.setFilterLimits (0.01, 1000.0);
  // ptfilter.setNegative (true);
  // ptfilter.filter (*indices_xz);
  // // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 0.01 or smaller than 1000.0
  // // ptfilter.setIndices (indices_xz);
  // ptfilter.setFilterFieldName ("y");
  // ptfilter.setFilterLimits (-1000.0, 1000.0);
  // ptfilter.setNegative (true);
  // ptfilter.filter (*indices_xyz);
  // // The indices_xyz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 0.01 or smaller than 1000.0 
  // // and have y between -1000.0 and 1000.0
  // ptfilter.setIndices (indices_xyz);
  // ptfilter.setFilterFieldName ("intensity");
  // ptfilter.setFilterLimits (FLT_MIN, 0.5);
  // ptfilter.setNegative (false);
  // ptfilter.filter (*cloud_out);
  // // The resulting cloud_out contains all points of cloud_in that are finite and have:
  // // x between 0.0 and 1000.0, z larger than 10.0 or smaller than -10.0 and have y between -1000.0 and 1000.0 and intensity smaller than 0.5.
  ////////////////////////////////

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
//   pcl_conversions::moveFromPCL(cloud_filtered, output);
  pcl::PCLPointCloud2 cloud_output;
  pcl::toPCLPointCloud2(*cloud_PassThrough_filtered, cloud_output);
  pcl_conversions::moveFromPCL(cloud_output, output);

  // Publish the data
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
