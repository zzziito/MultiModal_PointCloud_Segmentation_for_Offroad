#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

void twist_callback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  // Create the transform
  geometry_msgs::TransformStamped transformStamped;
  transformStamped.header.stamp = msg->header.stamp;
  transformStamped.header.frame_id = "odom";
  transformStamped.child_frame_id = "base_link";
  transformStamped.transform.translation.x = msg->twist.linear.x * 0.1; // Assuming time step is 0.1s
  transformStamped.transform.translation.y = msg->twist.linear.y * 0.1;
  transformStamped.transform.translation.z = msg->twist.linear.z * 0.1;
  tf2::Quaternion q;
  q.setRPY(0, 0, 0); // Assume zero rotation
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  // Publish the transform
  static tf2_ros::TransformBroadcaster br;
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tf_publisher");
  ros::NodeHandle nh;

  ros::Subscriber twist_sub = nh.subscribe("/linear_velocity", 1000, twist_callback);

  ros::spin();

  return 0;
}
