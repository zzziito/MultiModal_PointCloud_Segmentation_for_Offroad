#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <vector>
#include <numeric>

using namespace std;

vector<double> velocities_x;
vector<double> velocities_y;
vector<double> velocities_z;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg)
{
  velocities_x.push_back(msg->linear_acceleration.x);
  velocities_y.push_back(msg->linear_acceleration.y);
  velocities_z.push_back(msg->linear_acceleration.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imu_velocity_node");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/os1_cloud_node/imu", 1000, imu_callback);
  ros::Publisher velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("/linear_velocity", 1000);

  ros::Rate loop_rate(10); // Publish rate: 10Hz

  while (ros::ok())
  {
    if (!velocities_x.empty() && !velocities_y.empty() && !velocities_z.empty())
    {
      // Calculate the average linear velocity in x, y, and z directions
      double average_velocity_x = accumulate(velocities_x.begin(), velocities_x.end(), 0.0) / velocities_x.size();
      double average_velocity_y = accumulate(velocities_y.begin(), velocities_y.end(), 0.0) / velocities_y.size();
      double average_velocity_z = accumulate(velocities_z.begin(), velocities_z.end(), 0.0) / velocities_z.size();

      // Clear the velocity vectors
      velocities_x.clear();
      velocities_y.clear();
      velocities_z.clear();

      // Create the twist message
      geometry_msgs::TwistStamped velocity_msg;
      velocity_msg.header.stamp = ros::Time::now(); // Set the current time as the timestamp
      velocity_msg.twist.linear.x = average_velocity_x;
      velocity_msg.twist.linear.y = average_velocity_y;
      velocity_msg.twist.linear.z = average_velocity_z;

      // Publish the message
      velocity_pub.publish(velocity_msg);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
