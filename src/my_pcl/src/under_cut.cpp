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

void under_cut (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Container for pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_PassThrough_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::pointxyz>
    pcl::fromPCLPointCloud2(cloud_filtered,*cloud_point);

    // // Create the filtering object
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (cloud_point);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.5, 5.0);
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_PassThrough_filtered);

    // Convert to ROS data type
    pcl::PCLPointCloud2 cloud_output;
    sensor_msgs::PointCloud2 output_cut;

    pcl::toPCLPointCloud2(*cloud_PassThrough_filtered, cloud_output);
    pcl_conversions::moveFromPCL(cloud_output, output_cut);

    // Publish the data
    pub.publish (output_cut);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pc_cut");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("output", 1, under_cut);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output_cut", 1);
  // pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);

  // Spin
  // while (ros::ok()) { ros::spinOnce(); }
  ros::spin ();
}
