#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>

ros::Publisher pub;
ros::Subscriber sub;

// void cloud_prc (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// {
    // Container for original & filtered & indices data
    // pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // Convert to PCL data type
    // pcl_conversions::toPCL(*cloud_msg, *cloud);

    //Convert to pcl::PointCloud<pcl::pointxyz>::Ptr
    // pcl::fromPCLPointCloud2(*cloud,*cloud_point);

    // Fill in the cloud data
    // pcl::PCDReader reader;
    // // Replace the path below with the path where you saved your file
    // reader.read<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);

    // std::cerr << "Cloud before filtering: " << std::endl;
    // std::cerr << *cloud << std::endl;

    // Create the filtering object
    // pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    // pmf.setInputCloud (cloud);
    // pmf.setMaxWindowSize (20);
    // pmf.setSlope (-1.1367f); // tg = h/(d*12.5) => d=0.4, tg=0.1698 // slope = tg^-1(diff/d) ==> diff=(-(0.849-0.02)-0)/(0.4-0)=-2.0725, slope=(1/0.1698)/(-2.0725/0.4)=+-1.1367 
    // pmf.setInitialDistance (0.849f);
    // pmf.setMaxDistance (1.0f);
    // pmf.extract (ground->indices);

    // // Create the filtering object
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    // extract.setInputCloud (cloud);
    // extract.setIndices (ground);
    // extract.filter (*cloud_filtered);
    
    // std::cerr << " abc:" << cloud_point << std::endl;
    // std::cerr << " abc:" << cloud_filtered << std::endl;

    // // Extract non-ground returns
    // extract.setNegative (true);
    // extract.filter (*cloud_filtered);

    // std::cerr << "non-ground: " << cloud_filtered.x << " " 
    //                             << cloud_filtered.y << " "
    //                             << cloud_filtered.z << " " std::endl;
                                      
    // Convert to ROS data type
    // sensor_msgs::PointCloud2 output;
    // pcl::PCLPointCloud2 cloud_output;

    // pcl::toPCLPointCloud2(*cloud_filtered, cloud_output);
    // pcl_conversions::moveFromPCL(cloud_output, output);

    // Publish the data
    // pub.publish (output);
// }

int main (int argc, char** argv)
{
    // // Initialize ROS
    ros::init (argc, argv, "identify_ground");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointIndicesPtr ground (new pcl::PointIndices);

    // Fill in the cloud data
    pcl::PCDReader reader;
    // Replace the path below with the path where you saved your file
    reader.read<pcl::PointXYZ> ("/home/big/catkin_ws/src/my_pcl/src/samp11-utm.pcd", *cloud);

    std::cerr << "Cloud file: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
    pmf.setInputCloud (cloud);
    pmf.setMaxWindowSize (20);
    pmf.setSlope (1.0f);
    pmf.setInitialDistance (0.5f);
    pmf.setMaxDistance (3.0f);
    pmf.extract (ground->indices);

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (ground);
    extract.filter (*cloud_filtered);

    // std::cerr << "Ground cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);

    // Extract non-ground returns
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    // std::cerr << "Object cloud after filtering: " << std::endl;
    // std::cerr << *cloud_filtered << std::endl;

    writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

    return (0);
    // // Create a ROS subscriber for the input point cloud
    // sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/color/points", 1, cloud_prc);

    // // Create a ROS publisher for the output point cloud
    // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // // Spin
    // ros::spin ();
    
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::PointIndicesPtr ground (new pcl::PointIndices);

//   // Fill in the cloud data
//   pcl::PCDReader reader;
//   // Replace the path below with the path where you saved your file
//   reader.read<pcl::PointXYZ> ("samp11-utm.pcd", *cloud);

//   std::cerr << "Cloud before filtering: " << std::endl;
//   std::cerr << *cloud << std::endl;

//   // Create the filtering object
//   pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;
//   pmf.setInputCloud (cloud);
//   pmf.setMaxWindowSize (20);
//   pmf.setSlope (1.0f);
//   pmf.setInitialDistance (0.5f);
//   pmf.setMaxDistance (3.0f);
//   pmf.extract (ground->indices);

//   // Create the filtering object
//   pcl::ExtractIndices<pcl::PointXYZ> extract;
//   extract.setInputCloud (cloud);
//   extract.setIndices (ground);
//   extract.filter (*cloud_filtered);

//   std::cerr << "Ground cloud after filtering: " << std::endl;
//   std::cerr << *cloud_filtered << std::endl;

//   pcl::PCDWriter writer;
//   writer.write<pcl::PointXYZ> ("samp11-utm_ground.pcd", *cloud_filtered, false);

//   // Extract non-ground returns
//   extract.setNegative (true);
//   extract.filter (*cloud_filtered);

//   std::cerr << "Object cloud after filtering: " << std::endl;
//   std::cerr << *cloud_filtered << std::endl;

//   writer.write<pcl::PointXYZ> ("samp11-utm_object.pcd", *cloud_filtered, false);

//   return (0);
}