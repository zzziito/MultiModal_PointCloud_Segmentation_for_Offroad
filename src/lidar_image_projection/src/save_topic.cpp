#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h> // Include cv_bridge library
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  // 이미지 데이터 수신
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // 이미지 저장
  cv::imwrite("image.png", cv_ptr->image);
  std::cout << "Image saved to image.png" << std::endl;
}

void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // 포인트 클라우드 데이터 수신
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*msg, *cloud);

  // 포인트 클라우드 저장
  pcl::io::savePCDFileASCII("cloud.pcd", *cloud);
  std::cout << "Point cloud saved to cloud.pcd" << std::endl;
}

int main(int argc, char** argv)
{
  // ROS bag 파일 열기
  rosbag::Bag bag;
  bag.open("/home/rtlink/jiwon/socap_ws/Download/rellis/00000_20210224/20200213_trail_2.bag", rosbag::bagmode::Read);

  // ROS bag 파일에서 사용할 토픽 목록
  std::vector<std::string> topics;
  topics.push_back("/pylon_camera_node/image_raw");
  topics.push_back("/os1_cloud_node/points");

  // ROS bag 파일에서 데이터 읽기
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  // ROS 노드 초기화
  ros::init(argc, argv, "bag_reader");
  ros::NodeHandle nh;

  // 이미지 토픽 구독자 생성
  ros::Subscriber image_sub = nh.subscribe("/pylon_camera_node/image_raw", 1, imageCallback);

  // 포인트 클라우드 토픽 구독자 생성
  ros::Subscriber cloud_sub = nh.subscribe("/os1_cloud_node/points", 1, cloudCallback);

  // 데이터 처리 루프 시작
  for (rosbag::MessageInstance const m : view)
  {
    // 데이터 처리 중단
    if (ros::isShuttingDown())
      break;

    // ROS 메시지 처리
    if (m.getTopic() == "/pylon_camera_node/image_raw")
    {
      sensor_msgs::Image::ConstPtr img_msg = m.instantiate<sensor_msgs::Image>();
      if (img_msg != nullptr)
        imageCallback(img_msg);
    }
    else if (m.getTopic() == "/os1_cloud_node/points")
    {
      sensor_msgs::PointCloud2::ConstPtr cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (cloud_msg != nullptr)
        cloudCallback(cloud_msg);
    }
  }
}