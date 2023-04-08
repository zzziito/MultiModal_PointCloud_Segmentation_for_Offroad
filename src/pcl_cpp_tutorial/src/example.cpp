#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int
 main (int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);


  pcl::io::loadPLYFile<pcl::PointXYZRGB> ("/home/rtlink/jiwon/socap_ws/Download/Rellis_3D_lidar_example/os1_cloud_node_color_ply/frame000000-1581624652_770.ply", *cloud);

  std::cout << "Loaded :" << cloud->width * cloud->height  << std::endl;

  // Create the filtering object
//   pcl::PassThrough<pcl::PointXYZRGB> pass;
//   pass.setInputCloud (cloud);
//   pass.setFilterFieldName ("label");
//   pass.setFilterLimits ("grass");
//   //pass.setFilterLimitsNegative (true);
//   pass.filter (*cloud_filtered);

//   std::cout << "Filtered :" << cloud_filtered->width * cloud_filtered->height  << std::endl;
  

//   pcl::io::savePCDFile<pcl::PointXYZRGB>("filtered_pointcloud1.pcd", *cloud); //Default binary mode save

  return (0);
}