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
// ros::Publisher pub2;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // Container for original & filtered data
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;

  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point (new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_PassThrough_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  // pcl::PCLPointCloud2::Ptr cloud_before (new pcl::PCLPointCloud2 ());
  // pcl::PCLPointCloud2::Ptr cloud_after (new pcl::PCLPointCloud2 ());

  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // before fillter
  // pcl::fromPCLPointCloud2(cloudPtr, *cloud_before);
  // std::cerr << "PointCloud before filtering: " << (cloud->width) * (cloud->height) 
  //      << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.005, 0.005, 0.005);
  sor.filter (cloud_filtered);

  //after filter
  // pcl::fromPCLPointCloud2(cloud_filtered, *cloud_after);
  std::cerr << "PointCloud after filtering: " << cloud_filtered.width * cloud_filtered.height 
       << " data points (" << pcl::getFieldsList (cloud_filtered) << ")." << std::endl;

  //Convert to pcl::PointCloud<pcl::pointxyz>::Ptr
  // pcl::fromPCLPointCloud2(cloud_filtered,*cloud_point);

  //////////////////////////////////////////PassThrough filter
  // Create the filtering object
  // pcl::PassThrough<pcl::PointXYZ> pass;
  // pass.setInputCloud (cloud_point);
  // pass.setFilterFieldName ("z");
  // pass.setFilterLimits (0.5, 5.0);
  // pass.setFilterLimitsNegative (true);
  // pass.filter (*cloud_PassThrough_filtered);
  ////////////////////////////////


  ///////////////////////////Segmentation Processing
//   pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//   // Fill in the cloud data
//   temp_cloud->width  = 15;
//   temp_cloud->height = 1;
//   temp_cloud->points.resize (temp_cloud->width * temp_cloud->height);

//   // Generate the data
//   for (auto& point: *temp_cloud)
//   {
//     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
//     point.z = 1.0;
//   }

//   // Set a few outliers
//   (*temp_cloud)[0].z = 2.0;
//   (*temp_cloud)[3].z = -2.0;
//   (*temp_cloud)[6].z = 4.0;

//   std::cerr << "Point cloud data: " << temp_cloud->size () << " points" << std::endl;
//   for (const auto& point: *temp_cloud)
//     std::cerr << "    " << point.x << " "
//                         << point.y << " "
//                         << point.z << std::endl;

//   pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
//   pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
//   // Create the segmentation object
//   pcl::SACSegmentation<pcl::PointXYZ> seg;
//   // Optional
//   seg.setOptimizeCoefficients (true);
//   // Mandatory
//   seg.setModelType (pcl::SACMODEL_PLANE);
//   seg.setMethodType (pcl::SAC_RANSAC);
//   seg.setDistanceThreshold (0.01);

//   seg.setInputCloud (temp_cloud);
//   seg.segment (*inliers, *coefficients);

//   if (inliers->indices.size () == 0)
//   {
//     PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
//     // return (-1);
//   }

//   std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
//                                       << coefficients->values[1] << " "
//                                       << coefficients->values[2] << " " 
//                                       << coefficients->values[3] << std::endl;

//   std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
//   for (const auto& idx: inliers->indices)
//     std::cerr << idx << "    " << temp_cloud->points[idx].x << " "
//                                << temp_cloud->points[idx].y << " "
//                                << temp_cloud->points[idx].z << std::endl;

//   return (0);
  //////////////////////////////////////////////////////////////////////////////////////////

  ///

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::moveFromPCL(cloud_filtered, output);
  // sensor_msgs::PointCloud2 output2;
  // pcl_conversions::moveFromPCL(*cloud, output2);

  // pcl::PCLPointCloud2 cloud_output;
  // pcl::toPCLPointCloud2(*cloud_PassThrough_filtered, cloud_output);
  // pcl_conversions::moveFromPCL(cloud_output, output);

  // Publish the data
  pub.publish (output);
  // pub2.publish (output2);
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
  // pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);

  // Spin
  // while (ros::ok()) { ros::spinOnce(); }
  ros::spin ();
}
