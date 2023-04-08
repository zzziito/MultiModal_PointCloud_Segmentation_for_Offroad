#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <fstream>
#include <iomanip>
#include <chrono>
#include <string>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>

using namespace std;
using namespace std::chrono;
using namespace message_filters;

string image_folder = "/media/rtlink/JetsonSSD-256/socap_dataset/image/";
string pcd_folder = "/media/rtlink/JetsonSSD-256/socap_dataset/pc/";
string imu_folder = "/media/rtlink/JetsonSSD-256/socap_dataset/imu/";

int file_counter = 0;

void saveData(const sensor_msgs::PointCloud2::ConstPtr& pc_msg, const sensor_msgs::Image::ConstPtr& image_msg, const sensor_msgs::Imu::ConstPtr& imu_msg)
{
  // Convert point cloud to pcl format
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*pc_msg, cloud);

  // Save point cloud as pcd file
  string pcd_filename = pcd_folder + to_string(file_counter) + "_" + to_string(image_msg->header.stamp.toNSec()) + ".pcd";
  pcl::io::savePCDFileASCII(pcd_filename, cloud);

  // Save image as png file
  string image_filename = image_folder + to_string(file_counter) + "_" + to_string(image_msg->header.stamp.toNSec()) + ".png";
  ofstream outfile(image_filename, ios::out | ios::binary);
  outfile.write((char*)&image_msg->data[0], image_msg->data.size());

  // Compute time a
  float time_a = 0.9 / imu_msg->linear_acceleration.x;

  // Save imu data as csv file
  string imu_filename = imu_folder + to_string(file_counter) + "_" + to_string(image_msg->header.stamp.toNSec()) + ".csv";
  ofstream imu_outfile(imu_filename, ios::out);
  imu_outfile << "time,ax,ay,az,wx,wy,wz" << endl;
  ros::Time start_time = imu_msg->header.stamp - ros::Duration(time_a - 2);
  ros::Time end_time = imu_msg->header.stamp + ros::Duration(time_a + 2);
  imu_outfile << fixed << setprecision(9);
  for (const auto& imu_data : *(imu_sub->getQueue()))
    {
    const sensor_msgs::Imu::ConstPtr imu_msg = boost::dynamic_pointer_cast<const sensor_msgs::Imu>(imu_data);
    ros::Time imu_time = imu_msg->header.stamp;

    if (imu_time >= start_time && imu_time <= end_time)
    {
        imu_outfile << imu_time.toSec() << ","
                    << imu_msg->linear_acceleration.x << ","
                    << imu_msg->linear_acceleration.y << ","
                    << imu_msg->linear_acceleration.z << ","
                    << imu_msg->angular_velocity.x << ","
                    << imu_msg->angular_velocity.y << ","
                    << imu_msg->angular_velocity.z << endl;
    }
    }

  // Increment file counter
  file_counter++;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_data_node");
  ros::NodeHandle nh;
  // Set up subscribers
  Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, "/pylon_camera_node/image_raw/cropped_point", 1);
  Subscriber<sensor_msgs::Image> image_sub(nh, "/pylon_camera_node/image_raw", 1);
  Subscriber<sensor_msgs::Imu> imu_sub(nh, "/os1_cloud_node/imu", 1);

  // Synchronize the subscribers using approximate time policy
  typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image, sensor_msgs::Imu> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, image_sub, imu_sub);
  sync.registerCallback(boost::bind(&saveData, _1, _2, _3));

  // Spin
  ros::spin();

  return 0;
}