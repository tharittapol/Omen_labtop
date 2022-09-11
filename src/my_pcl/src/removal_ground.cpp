#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/extract_indices.h>

#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    // pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    // pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2::Ptr pcl_pc2 (new pcl::PCLPointCloud2 ());
    pcl::PCLPointCloud2::Ptr cloud_downsampled (new pcl::PCLPointCloud2 ());

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *pcl_pc2);

    std::cerr << "PointCloud original: " << pcl_pc2->width * pcl_pc2->height 
              << " data points (" << pcl::getFieldsList (*pcl_pc2) << ")." << std::endl;

    /* Voxel Grid Downsampling*/
    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (pcl_pc2);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud_downsampled);

    std::cerr << "PointCloud after Downsampling: " << cloud_downsampled->width * cloud_downsampled->height 
              << " data points (" << pcl::getFieldsList (*cloud_downsampled) << ")." << std::endl;


    /* Region crop with y and x axis (! them use camera_depth_optical_frame) (quite unnecessary) */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_xyz (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_PassThrough (new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr indices_rem (new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointIndicesPtr buf (new pcl::PointIndices);

    // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZRGB>
    pcl::fromPCLPointCloud2(*cloud_downsampled, *point_xyz);

    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB> pass (true);
    pass.setInputCloud (point_xyz);
    pass.setFilterFieldName ("y");
    pass.setFilterLimits (-1.0, 0.0);
    // pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_PassThrough);

    // indices_rem = pass.getRemovedIndices ();
    
    // pass.setIndices (*buf);
    pass.setFilterFieldName ("x");
    pass.setFilterLimits (-0.45, 0.45);
    // pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_PassThrough);


    std::cerr << "PointCloud after PassThrough: " << cloud_PassThrough->width * cloud_PassThrough->height 
              << " data points (" << pcl::getFieldsList (*cloud_PassThrough) << ")." << std::endl;
    // for (const auto& point: *cloud_PassThrough)
    //     std::cerr << "    " << point.x << " "
    //                         << point.y << " "
    //                         << point.z << std::endl;

    /* Plane segmentation */
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    Eigen::Vector3f axis;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // select only ground perpendicular
    float set_angle = 45.0;
    axis = Eigen::Vector3f(pcl::fields::x,pcl::fields::y,pcl::fields::z);
    seg.setAxis(axis);
    seg.setEpsAngle((set_angle*(3.1415/180.0f)));
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE); //
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    // create Extract fillter(Non ground)
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;

    seg.setInputCloud (cloud_PassThrough);
    seg.segment (*inliers, *coefficients);
  
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar." << std::endl;
    }

    // Extract the outliers(Non ground)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ransac (new pcl::PointCloud<pcl::PointXYZRGB>);
    extract.setInputCloud (cloud_PassThrough);
    extract.setIndices (inliers);
    extract.setNegative (true); // select non ground index
    extract.filter (*cloud_ransac);

    std::cerr << "PointCloud after Removal_ground: " << cloud_ransac->width * cloud_ransac->height 
                                                    << " data points (" << pcl::getFieldsList (*cloud_ransac) << ")." << std::endl;

    // cloud_PassThrough.swap(cloud_ransac); // Swap a point cloud with another cloud
    // *cloud_PassThrough = *cloud_ransac; // not sure how different it

    /* statistical outlier removal */
    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_outlier_rmv (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> rem;
    rem.setInputCloud (cloud_ransac);
    rem.setMeanK (10);
    rem.setStddevMulThresh (0.001);
    rem.filter (*cloud_outlier_rmv);

    std::cerr << "PointCloud after Removal_outlier: " << cloud_outlier_rmv->width * cloud_outlier_rmv->height 
                                                      << " data points (" << pcl::getFieldsList (*cloud_outlier_rmv) << ")." << std::endl;

    /* Euclidean clustering (distinguish the objects from one another) */ //(From testing, it's not necessary and doesn't work well for needs.)
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_outlier_rmv);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (2500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_outlier_rmv);
    ec.extract (cluster_indices);

    int j = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (const auto& idx : it->indices)
        cloud_cluster->push_back ((*cloud_outlier_rmv)[idx]); //*
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
    
        std::cerr << "PointCloud after Cluster: " << cloud_cluster->size () << " data points (" << pcl::getFieldsList (*cloud_outlier_rmv) << ")." << std::endl;
        j++;
    }
    (*cloud_cluster).header = (*cloud_outlier_rmv).header;                                             

    /* Convert to ROS data type */
    sensor_msgs::PointCloud2 output;
    // pcl_conversions::fromPCL(*cloud_downsampled, output);

    // convert pcl::PointCloud<pcl::PointXYZRGB> to  pcl::PCLPointCloud2
    pcl::PCLPointCloud2::Ptr pcl_pc2_out (new pcl::PCLPointCloud2 ());
    pcl::toPCLPointCloud2(*cloud_cluster, *pcl_pc2_out);
    pcl_conversions::fromPCL(*pcl_pc2_out, output);

    // Publish the data
    pub.publish (output);
}

int main (int argc, char **argv)
{
    // Initialize ROS
    ros::init (argc, argv, "removal_ground");
    ros::NodeHandle nh;

    // create a subscriber
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/camera/depth/color/points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("non_ground", 1);

    // Spin
    ros::spin ();
}