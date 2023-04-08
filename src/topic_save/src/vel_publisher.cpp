#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> 
#include <sensor_msgs/Image.h>
#include <iostream>
#include <geometry_msgs/Twist.h>

class ImuLinearVelPublisher {
public:
  ImuLinearVelPublisher(ros::NodeHandle& nh) {
    imu_subscriber_ = nh.subscribe("/os1_cloud_node/imu", 1, &ImuLinearVelPublisher::imuCallback, this);
    linear_vel_publisher_ = nh.advertise<geometry_msgs::Twist>("/linear_velocity", 1);
  }

private:
  ros::Subscriber imu_subscriber_;
  ros::Publisher linear_vel_publisher_;
  double last_imu_time_ = 0.0;
  Eigen::Quaterniond orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d velocity_ = Eigen::Vector3d::Zero();

  void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
    double dt = (msg->header.stamp - last_imu_time_).toSec();
    last_imu_time_ = msg->header.stamp;

    Eigen::Quaterniond delta_q(msg->angular_velocity.x * dt, msg->angular_velocity.y * dt, msg->angular_velocity.z * dt, 0.0);
    Eigen::Quaterniond new_orientation = (orientation_ * delta_q).normalized();

    velocity_ += new_orientation.toRotationMatrix() * Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z) * dt;
    orientation_ = new_orientation;

    geometry_msgs::Twist linear_vel_msg;
    linear_vel_msg.linear.x = velocity_.x();
    linear_vel_msg.linear.y = velocity_.y();
    linear_vel_msg.linear.z = velocity_.z();

    linear_vel_publisher_.publish(linear_vel_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "vel_publisher");
  ros::NodeHandle nh;

  ImuLinearVelPublisher imu_linear_vel_publisher(nh);

  ros::spin();
}
