#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <iostream>

int main(int argc, char** argv)
{
  // .pcd 파일 경로 설정
  std::string file_path = "/media/rtlink/JetsonSSD-256/socap_dataset/pc/1026_1581624733.968697.pcd";

  // .pcd 파일에서 포인트 클라우드 읽기
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1) // 파일 로딩 실패
  {
    PCL_ERROR("Failed to read .pcd file\n");
    return (-1);
  }

  // Normal 추정
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  ne.setSearchMethod(tree);

  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  ne.setRadiusSearch(0.03); // Normal 추정 반경 설정
  ne.compute(*normals);

  // Moment of Inertia 추정
  pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
  feature_extractor.setInputCloud(cloud);
  feature_extractor.setInputNormals(normals);

  pcl::PointXYZ min_point, max_point;
  pcl::PointXYZ center_point;
  Eigen::Matrix3f moment_of_inertia;
  std::vector<float> radii;

  feature_extractor.compute();
  feature_extractor.getAABB(min_point, max_point);
  feature_extractor.getEigenVectors(moment_of_inertia);
  feature_extractor.getEigenValues(radii);
  feature_extractor.getMassCenter(center_point);

  // Surface Roughness 계산
  float roughness = 0;
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    Eigen::Vector3f vec(cloud->points[i].x - center_point.x, cloud->points[i].y - center_point.y, cloud->points[i].z - center_point.z);
    vec = moment_of_inertia * vec;
    float dist = vec.norm();
    roughness += dist;
  }
  roughness /= cloud->points.size();

  // Surface Roughness 출력
  std::cout << "Surface Roughness: " << roughness << std::endl;

  return 0;
}
