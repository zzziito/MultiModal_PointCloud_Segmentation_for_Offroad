// #include <iostream>
// #include <string>
// #include <fstream>
// #include <sstream>
// #include <vector>
// #include <pcl/io/ply_io.h>
// #include <pcl/point_types.h>

// int main(int argc, char** argv)
// {
//   // PLY 파일 경로 초기화
//   std::string file_path = "/home/rtlink/jiwon/socap_ws/Download/Rellis_3D_lidar_example/os1_cloud_node_color_ply/frame000000-1581624652_770.ply";

//   // PLY 파일 읽기
//   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
//   pcl::io::loadPLYFile<pcl::PointXYZRGB>(file_path, *cloud);

//   // 텍스트 파일로 저장
//   std::ofstream outfile("/home/rtlink/jiwon/socap_ws/output.txt");
//   if (outfile.is_open()) {
//     for (int i = 0; i < cloud->size(); i++) {
//       pcl::PointXYZRGB point = cloud->points[i];
//       // XYZ 좌표와 RGB 색상 값을 텍스트 파일에 저장
//     //   outfile << point.x << " " << point.y << " " << point.z << " " << (int)point.r << " " << (int)point.g << " " << (int)point.b << << std::endl;
//       outfile << point.x << " " << point.y << " " << point.z << " " << (int)point.rgb << std::endl;

//     }
//     outfile.close();
//     std::cout << "Converted to output.txt" << std::endl;
//   }
//   else {
//     std::cerr << "Unable to open file" << std::endl;
//     return -1;
//   }

//   return 0;
// }

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

int main(int argc, char** argv)
{
  // PCD 파일 경로 초기화
  std::string file_path = "/home/rtlink/jiwon/socap_ws/tabletop_passthrough2.pcd";

  // PCD 파일 읽기
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::io::loadPCDFile<pcl::PointXYZRGB>(file_path, *cloud);

  // 텍스트 파일로 저장
  std::ofstream outfile("/home/rtlink/jiwon/socap_ws/output.txt");
  if (outfile.is_open()) {
    for (int i = 0; i < cloud->size(); i++) {
      pcl::PointXYZRGB point = cloud->points[i];
      // XYZ 좌표와 RGB 색상 값을 텍스트 파일에 저장
      outfile << point.x << " " << point.y << " " << point.z << " " << (int)point.r << " " << (int)point.g << " " << (int)point.b << std::endl;
    }
    outfile.close();
    std::cout << "Converted to output.txt" << std::endl;
  }
  else {
    std::cerr << "Unable to open file" << std::endl;
    return -1;
  }

  return 0;
}
