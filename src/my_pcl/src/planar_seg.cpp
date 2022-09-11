#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // Container for original & filtered data
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    //Convert to pcl::PointCloud<pcl::pointxyz>::Ptr
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_point (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud,*cloud_point);

    // std::cerr << "Point cloud data: " << cloud_point->size () << " points" << std::endl;
    // for (const auto& point: *cloud_point)
    // std::cerr << "    " << point.x << " "
    //                     << point.y << " "
    //                     << point.z << std::endl;

    //Planer segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud_point);
    seg.segment (*inliers, *coefficients);

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        // return (-1);
    }

    // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
    //                                     << coefficients->values[1] << " "
    //                                     << coefficients->values[2] << " " 
    //                                     << coefficients->values[3] << std::endl;

    std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
    for (std::size_t i = 0; i < inliers->indices.size (); ++i)
    for (const auto& idx: inliers->indices)
        std::cerr << idx << "    " << cloud_point->points[idx].x << " "
                                << cloud_point->points[idx].y << " "
                                << cloud_point->points[idx].z << std::endl;



    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl::PCLPointCloud2 cloud_output;

    pcl::fromPCL(*cloud_point,  cloud_output);
    pcl_conversions::moveFromPCL(cloud_output, output);

    // Publish the data
    pub.publish(output);
}

int main (int argc, char** argv){
    // Initialize ROS
    ros::init (argc, argv, "planar_seg");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    // Spin
    ros::spin ();
    }