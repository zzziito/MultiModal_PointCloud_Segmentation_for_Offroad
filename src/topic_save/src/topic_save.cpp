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

using namespace std;

class PointCloudIMUSaver {
public:
    PointCloudIMUSaver(ros::NodeHandle& nh) : orientation_(Eigen::Quaterniond::Identity()) {        
        point_cloud_subscriber_ = nh.subscribe("/os1_cloud_node/points", 5, &PointCloudIMUSaver::pointCloudCallback, this);
        imu_subscriber_ = nh.subscribe("/imu/data", 5, &PointCloudIMUSaver::imuCallback, this);
        img_subscriber_ = nh.subscribe("/pylon_camera_node/image_raw", 5, &PointCloudIMUSaver::imageCallback, this);
    }

private:
    ros::Subscriber point_cloud_subscriber_;  
    ros::Subscriber imu_subscriber_;
    ros::Subscriber img_subscriber_;
    std::deque<sensor_msgs::Imu> imu_buffer_;
    ros::Time last_imu_time_;
    double last_save_time_ = 0.0;  
    int save_index_ = 0;  
    double time_offset = 0.0;
    Eigen::Quaterniond orientation_;
    Eigen::Vector3d velocity_;

    sensor_msgs::PointCloud2ConstPtr last_point_cloud_msg_; // Added variable to store last point cloud message
    sensor_msgs::ImageConstPtr last_image_msg_; // Added variable to store last image message

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        last_point_cloud_msg_ = msg; // Store the current point cloud message

        // Check if an image message has been received and save the pair if so
        if (last_image_msg_ != nullptr) {
            double current_time = msg->header.stamp.toSec();

            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*msg, cloud);
            pcl::io::savePCDFileASCII("/media/rtlink/JetsonSSD-256/socap_dataset/pc/"+std::to_string(save_index_) + "_" + std::to_string(current_time) + ".pcd", cloud);


            cv_bridge::CvImagePtr cv_ptr;
            try {
                cv_ptr = cv_bridge::toCvCopy(last_image_msg_, sensor_msgs::image_encodings::BGR8);
            }
            catch (cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return;
            }

            cv::imwrite("/media/rtlink/JetsonSSD-256/socap_dataset/image/"+std::to_string(save_index_) + "_" + std::to_string(current_time) + ".png", cv_ptr->image);

            double time_offset = 0.0;
            if (!imu_buffer_.empty()) {
                time_offset = (current_time - imu_buffer_.back().header.stamp.toSec()) - 0.5;
            }

            findAndSaveCorrespondingIMU(current_time, time_offset);
            save_index_++;

            last_image_msg_ = nullptr; // Reset the image message
        }
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        last_image_msg_ = msg; // Store the current image message
    }

    void imuCallback(const sensor_msgs::ImuConstPtr& msg) {
        double dt = (msg->header.stamp - last_imu_time_).toSec();
        last_imu_time_ = msg->header.stamp;

        Eigen::Quaterniond delta_q(1.0, 0.5 * msg->angular_velocity.x * dt, 0.5 * msg->angular_velocity.y * dt, 0.5 * msg->angular_velocity.z * dt);
        orientation_ = (orientation_ * delta_q).normalized();

        velocity_ = Eigen::Vector3d(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);

        imu_buffer_.push_back(*msg);
        while (imu_buffer_.size() > 0 && (msg->header.stamp - imu_buffer_.front().header.stamp).toSec() > 5.0) {
        imu_buffer_.pop_front();
        }
    }

    void findAndSaveCorrespondingIMU(double point_cloud_time, double time_offset) {
        sensor_msgs::Imu corresponding_imu;
        bool found = false;
        double min_time_diff = std::numeric_limits<double>::max();

        for (const auto& imu : imu_buffer_) {
            double time_diff = std::abs(imu.header.stamp.toSec() - (point_cloud_time + time_offset));
            if (time_diff < min_time_diff) {
                min_time_diff = time_diff;
                corresponding_imu = imu;
                found = true;
            }
        }

        if (found) {

            Eigen::Quaterniond orientation(this->orientation_);
            Eigen::Vector3d angular_vel(corresponding_imu.angular_velocity.x, corresponding_imu.angular_velocity.y, corresponding_imu.angular_velocity.z);
            Eigen::Vector3d linear_vel = orientation.toRotationMatrix() * angular_vel;
            double imu_time_diff = (-0.9) / (linear_vel[0]*100);
            if (linear_vel[0]<0){
                ROS_INFO_STREAM("time_difference : "<< imu_time_diff);
                ROS_INFO_STREAM("linear velocity: "<< linear_vel[0]);
            }

            // saveIMUMeasurementsToFile(corresponding_imu, point_cloud_time + imu_time_diff, linear_vel);
            // } else {
            // ROS_WARN("Could not find corresponding IMU measurement for the point cloud.");
        }
    }

    void saveIMUMeasurementsToFile(const sensor_msgs::Imu& imu, double point_cloud_time, const Eigen::Vector3d& linear_vel) {        
        std::string filename = "/media/rtlink/JetsonSSD-256/socap_dataset/imu/" + std::to_string(save_index_) + "_" + std::to_string(imu.header.stamp.toSec()) + ".csv";
        std::ofstream outfile(filename);
        outfile << "header,timestamp,orientation_x,orientation_y,orientation_z,orientation_w,angular_velocity_x,angular_velocity_y,angular_velocity_z,linear_acceleration_x,linear_acceleration_y,linear_acceleration_z" << std::endl;
        outfile << imu.header.stamp << "," << imu.orientation.x << "," << imu.orientation.y << "," << imu.orientation.z << "," << imu.orientation.w << "," << imu.angular_velocity.x << "," << imu.angular_velocity.y << "," << imu.angular_velocity.z << "," << imu.linear_acceleration.x << "," << imu.linear_acceleration.y << "," << imu.linear_acceleration.z << std::endl;
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_imu_saver");
  ros::NodeHandle nh;

  PointCloudIMUSaver point_cloud_imu_saver(nh);


  ros::spin();
};