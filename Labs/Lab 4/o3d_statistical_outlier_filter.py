# o3d_statistical_outlier_filter.py

import open3d as o3d

pcloud = o3d.io.read_point_cloud("./point_clouds/table_scene_lms400.pcd")
filtered_pcloud, indices = pcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

inlier_cloud = pcloud.select_by_index(indices)
outlier_cloud = pcloud.select_by_index(indices, invert=True)

print("Showing outliers (red) and inliers (gray): ")
outlier_cloud.paint_uniform_color([1, 0, 0])
inlier_cloud.paint_uniform_color([0.8, 0.8, 0.8])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])

o3d.io.write_point_cloud("./point_clouds/table_scene_lms400_inliers.pcd", inlier_cloud)
o3d.io.write_point_cloud("./point_clouds/table_scene_lms400_outliers.pcd", outlier_cloud)
