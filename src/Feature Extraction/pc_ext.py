import open3d as o3d
import numpy as np

# .pcd 파일 경로 설정
file_path = "/media/rtlink/JetsonSSD-256/socap_dataset/pc/1026_1581624733.968697.pcd"

# .pcd 파일에서 포인트 클라우드 읽기
pcd = o3d.io.read_point_cloud(file_path)

# Normal 벡터 추정
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.03, max_nn=30))

# Moment of Inertia 계산
min_bound, max_bound = pcd.get_axis_aligned_bounding_box()
center = (min_bound + max_bound) / 2.0
R = pcd.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 2))
pcd.rotate(R, center=center)
moi = o3d.geometry.MomentOfInertiaEstimation()
moi.estimate(pcd)
moment_of_inertia = moi.get_inertia_tensor()
evals, evecs = np.linalg.eig(moment_of_inertia)
radii = np.sqrt(evals)

# Surface Roughness 계산
roughness = 0
for i in range(len(pcd.points)):
    point = np.array(pcd.points[i])
    vec = point - center
    dist = np.linalg.norm(np.dot(vec, evecs) / radii)
    roughness += dist
roughness /= len(pcd.points)

# Surface Roughness 출력
print("Surface Roughness:", roughness)
