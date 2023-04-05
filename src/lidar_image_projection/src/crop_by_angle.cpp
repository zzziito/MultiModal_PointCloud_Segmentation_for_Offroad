#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/crop_box.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
    // Read in the point cloud file
    std::string file_path = "/home/rtlink/jiwon/socap_ws/cloud.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud);

    // Define the camera intrinsics and image size
    double fx = 2813.643275;  // Focal length in x direction
    double fy = 2808.326079;  // Focal length in y direction
    double cx = 969.285772;  // Principal point x coordinate
    double cy = 624.049972;  // Principal point y coordinate
    int image_width = 1920;
    int image_height = 1200;

    // Calculate the camera's field of view
    double fov_x = 2 * atan(image_width / (2 * fx));
    double fov_y = 2 * atan(image_height / (2 * fy));

    // Calculate the limits of the crop box
    double xmin = -tan(fov_x / 2);
    double xmax = tan(fov_x / 2);
    double ymin = -tan(fov_y / 2);
    double ymax = tan(fov_y / 2);

    // Define the crop box filter
    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(cloud);
    
    ROS_INFO_STREAM(xmin);
    ROS_INFO_STREAM(xmax);
    ROS_INFO_STREAM(ymin);
    ROS_INFO_STREAM(ymax);

    crop_filter.setMin(Eigen::Vector4f(xmin, ymin, 0, 1));
    crop_filter.setMax(Eigen::Vector4f(xmax, ymax, 100, 1));  // Set the z limit to 100 for this example

    // Apply the crop filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*cropped_cloud);

    std::cout << "Filtered cloud has " << cropped_cloud->size() << " points." << std::endl;

    // Save the cropped point cloud to a new file
    // pcl::io::savePCDFileASCII("cropped_cloud.pcd", *cropped_cloud);
    // std::cout << "Saved " << cropped_cloud->points.size() << " data points to cropped_cloud.pcd." << std::endl;

    return 0;
}
