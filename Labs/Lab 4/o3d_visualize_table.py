# o3d_visualize_table.py

import open3d as o3d

pcloud = o3d.io.read_point_cloud("./point_clouds/table_scene_lms400.pcd")
o3d.visualization.draw_geometries([pcloud])