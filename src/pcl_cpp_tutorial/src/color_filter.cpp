#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  // PCD 파일 경로와 색상 값 변수 초기화
  std::string file_path = "/home/rtlink/jiwon/socap_ws/tabletop_passthrough2.pcd";
  int remove_color = 255; // 제거할 색상 값

  // PCD 파일 읽기
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud);

  // 색상 값에 따라 필터링하여 제거
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < cloud->size(); i++) {
    pcl::PointXYZRGB point = cloud->points[i];
    // RGB 색상 값 추출
    uint8_t r = point.r;
    uint8_t g = point.g;
    uint8_t b = point.b;
    // 제거할 색상 값과 일치하지 않으면 필터링된 포인트 클라우드에 추가
    if (r != remove_color && g != remove_color && b != remove_color) {
      filtered_cloud->push_back(point);
    }
  }

  // 필터링된 포인트 클라우드를 PCD 파일로 저장
  pcl::io::savePCDFileASCII("/home/rtlink/jiwon/socap_ws/color_filtered_pointcloud1.pcd", *filtered_cloud);

  std::cout << "Filtered cloud saved to output.pcd" << std::endl;

  return 0;
}
