 1#include <iostream>
 2#include <pcl/ModelCoefficients.h>
 3#include <pcl/io/pcd_io.h>
 4#include <pcl/point_types.h>
 5#include <pcl/sample_consensus/method_types.h>
 6#include <pcl/sample_consensus/model_types.h>
 7#include <pcl/segmentation/sac_segmentation.h>
 8#include <pcl/filters/voxel_grid.h>
 9#include <pcl/filters/extract_indices.h>
10
11int
12main ()
13{
14  pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
15  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>), cloud_p (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
16
17  // Fill in the cloud data
18  pcl::PCDReader reader;
19  reader.read ("table_scene_lms400.pcd", *cloud_blob);
20
21  std::cerr << "PointCloud before filtering: " << cloud_blob->width * cloud_blob->height << " data points." << std::endl;
22
23  // Create the filtering object: downsample the dataset using a leaf size of 1cm
24  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
25  sor.setInputCloud (cloud_blob);
26  sor.setLeafSize (0.01f, 0.01f, 0.01f);
27  sor.filter (*cloud_filtered_blob);
28
29  // Convert to the templated PointCloud
30  pcl::fromPCLPointCloud2 (*cloud_filtered_blob, *cloud_filtered);
31
32  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;
33
34  // Write the downsampled version to disk
35  pcl::PCDWriter writer;
36  writer.write<pcl::PointXYZ> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);
37
38  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
39  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
40  // Create the segmentation object
41  pcl::SACSegmentation<pcl::PointXYZ> seg;
42  // Optional
43  seg.setOptimizeCoefficients (true);
44  // Mandatory
45  seg.setModelType (pcl::SACMODEL_PLANE);
46  seg.setMethodType (pcl::SAC_RANSAC);
47  seg.setMaxIterations (1000);
48  seg.setDistanceThreshold (0.01);
49
50  // Create the filtering object
51  pcl::ExtractIndices<pcl::PointXYZ> extract;
52
53  int i = 0, nr_points = (int) cloud_filtered->size ();
54  // While 30% of the original cloud is still there
55  while (cloud_filtered->size () > 0.3 * nr_points)
56  {
57    // Segment the largest planar component from the remaining cloud
58    seg.setInputCloud (cloud_filtered);
59    seg.segment (*inliers, *coefficients);
60    if (inliers->indices.size () == 0)
61    {
62      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
63      break;
64    }
65
66    // Extract the inliers
67    extract.setInputCloud (cloud_filtered);
68    extract.setIndices (inliers);
69    extract.setNegative (false);
70    extract.filter (*cloud_p);
71    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;
72
73    std::stringstream ss;
74    ss << "table_scene_lms400_plane_" << i << ".pcd";
75    writer.write<pcl::PointXYZ> (ss.str (), *cloud_p, false);
76
77    // Create the filtering object
78    extract.setNegative (true);
79    extract.filter (*cloud_f);
80    cloud_filtered.swap (cloud_f);
81    i++;
82  }
83
84  return (0);
85}
