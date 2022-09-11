#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>


//for use pcl::PCLPointCloud2 http://wiki.ros.org/pcl/Tutorials/hydro?action=AttachFile&do=view&target=example_voxelgrid_pcl_types.cpp

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);

  // Print point
//   pcl::PointCloud<pcl::PointXYZ> input_;
//   pcl::fromROSMsg(*input, input_);
//   printf ("Cloud: width = %d, height = %d\n", input->width, input->height);
//   BOOST_FOREACH (const pcl::PointXYZ& pt, input->data)
//     printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth/color/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
