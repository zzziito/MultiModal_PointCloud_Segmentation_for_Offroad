#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <fstream>

std::ofstream outfile;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    outfile << msg->header.seq << ","
            << msg->angular_velocity.x << ","
            << msg->angular_velocity.y << ","
            << msg->angular_velocity.z << ","
            << msg->linear_acceleration.x << ","
            << msg->linear_acceleration.y << ","
            << msg->linear_acceleration.z << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "imu_data_logger");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu_data", 1000, imuCallback);

    outfile.open("imu_data.csv", std::ios::out | std::ios::trunc);
    outfile << "seq,ang_vel_x,ang_vel_y,ang_vel_z,lin_acc_x,lin_acc_y,lin_acc_z" << std::endl;

    ros::spin();

    outfile.close();
    return 0;
}
