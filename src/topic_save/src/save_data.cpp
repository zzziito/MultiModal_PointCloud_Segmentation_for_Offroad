#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/Image.h>

using namespace std;

// Variables to store point cloud and imu data
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::Imu imu_data;

// File name variables
string pc_file_path = "/media/rtlink/JetsonSSD-256/socap_dataset/pc/";
string img_file_path = "/media/rtlink/JetsonSSD-256/socap_dataset/image/";
string imu_file_path = "/media/rtlink/JetsonSSD-256/socap_dataset/imu/";

string topic_cloud = "/pylon_camera_node/image_raw/cropped_point";
string topic_imu = "/os1_cloud_node/imu";
string image_prefix = "image_";
string cloud_prefix = "cloud_";
string imu_prefix = "imu_";
string file_extension_image = ".png";
string file_extension_cloud = ".pcd";
string file_extension_imu = ".csv";

// Counter to track the number of saved files
int count_file = 0;

// Helper function to convert imu data to string format
string imuToString(const sensor_msgs::Imu& imu_data)
{
    stringstream ss;
    ss << imu_data.linear_acceleration.x << "," << imu_data.linear_acceleration.y << ","
       << imu_data.linear_acceleration.z << "," << imu_data.angular_velocity.x << ","
       << imu_data.angular_velocity.y << "," << imu_data.angular_velocity.z << ","
       << imu_data.orientation.x << "," << imu_data.orientation.y << ","
       << imu_data.orientation.z << "," << imu_data.orientation.w;
    return ss.str();
}

// Callback function to extract the latest point cloud data
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::fromROSMsg(*msg, *cloud);
}

// Callback function to extract the latest imu data
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    imu_data = *msg;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_data");
    ros::NodeHandle nh;

    // Read the rosbag file
    string bag_path = "/media/rtlink/JetsonSSD-256/Download/rellis/00000_20210224/20200213_trail_2.bag";
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Read);

    // Get the start time of the rosbag file
    rosbag::View view_start(bag);
    ros::Time start_time = view_start.getBeginTime();

    // Create subscribers to point cloud and imu data
    ros::Subscriber sub_cloud = nh.subscribe<sensor_msgs::PointCloud2>(topic_cloud, 1, cloudCallback);
    ros::Subscriber sub_imu = nh.subscribe<sensor_msgs::Imu>(topic_imu, 1, imuCallback);

    // Main loop to save data periodically
    
    ros::Rate rate(2); // 0.5 Hz
    while(ros::ok())
    {
        // Get the latest point cloud data
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_curr (new pcl::PointCloud<pcl::PointXYZRGB>);
        while (cloud->empty())
        {
            ros::spinOnce();
            ros::Duration(0.01).sleep();
        }
        *cloud_curr = *cloud;

        // Get the time of the latest point cloud data
       
        ros::Time time_curr = pcl_conversions::fromPCL(cloud_curr->header.stamp);

        // Calculate the time offset to get the corresponding imu data
        float dist = 0.9; // distance between imu and point cloud in meters
        float speed = imu_data.linear_acceleration.x; // speed of the robot in m/s
        float time_offset = dist / speed; // time offset in seconds

        // Get the imu data at the corresponding time
        ros::Time time_imu = time_curr + ros::Duration(time_offset);
        rosbag::View view_imu(bag, rosbag::TopicQuery(topic_imu), time_imu, time_imu + ros::Duration(4.0));
        for (rosbag::MessageInstance const m : view_imu)
        {
            sensor_msgs::Imu::ConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
            if (imu_msg != nullptr)
            {
                imu_data = *imu_msg;
            }
        }

        // Save the point cloud data
        string file_name_cloud = pc_file_path + cloud_prefix + to_string(count_file) + "_" + to_string(time_curr.sec) + file_extension_cloud;
        pcl::io::savePCDFileBinary(file_name_cloud, *cloud_curr);

        // Downsample the point cloud and save as an image
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::VoxelGrid<pcl::PointXYZRGB> sor;
        sor.setInputCloud(cloud_curr);
        sor.setLeafSize(0.1, 0.1, 0.1);
        sor.filter(*cloud_downsampled);
        string file_name_image = img_file_path + image_prefix + to_string(count_file) + "_" + to_string(time_curr.sec) + file_extension_image;
        cv::Mat image(cloud_downsampled->height, cloud_downsampled->width, CV_8UC3);
        for (size_t i = 0; i < cloud_downsampled->size(); ++i)
        {
            pcl::PointXYZRGB point = cloud_downsampled->points[i];
            image.at<cv::Vec3b>(point.y, point.x)[0] = point.b;
            image.at<cv::Vec3b>(point.y, point.x)[1] = point.g;
            image.at<cv::Vec3b>(point.y, point.x)[2] = point.r;
        }
        cv::imwrite(file_name_image, image);

        // Save the imu data as a csv file
        string file_name_imu = imu_file_path + imu_prefix + to_string(count_file) + "_" + to_string(time_curr.sec) + file_extension_imu;
        ofstream file_imu(file_name_imu);
        file_imu << imuToString(imu_data);
        file_imu.close();

        // Increase the file count
        count_file++;

        // Sleep for the remaining time to achieve the desired frequency
        rate.sleep();
    }

    bag.close();

    return 0;
}
