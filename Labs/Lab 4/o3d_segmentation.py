# # o3d_segmentation.py

# Passthrough reworked from https://github.com/powersimmani/example_3d_pass_through-filter_guide

import numpy as np
import open3d as o3d
import pyransac3d as pyrsc

def pass_through_filter(bbox, pcloud):
    points = np.asarray(pcloud.points)
    colors = np.asarray(pcloud.colors)
    points[:,0]
    x_range = np.logical_and(points[:,0] >= bbox["x"][0] ,points[:,0] <= bbox["x"][1])
    y_range = np.logical_and(points[:,1] >= bbox["y"][0] ,points[:,1] <= bbox["y"][1])
    z_range = np.logical_and(points[:,2] >= bbox["z"][0] ,points[:,2] <= bbox["z"][1])

    enclosed = np.logical_and(x_range,np.logical_and(y_range,z_range))

    pcloud.points = o3d.utility.Vector3dVector(points[enclosed])
    pcloud.colors = o3d.utility.Vector3dVector(colors[enclosed])

    return pcloud

pcloud = o3d.io.read_point_cloud("./point_clouds/table_scene_mug_stereo_textured.pcd")

# Discard points outside the bounding box. This narrows down on the table and mug.
bounding_box = {"x":[-0.3,0.5], "y":[-0.3,0.5], "z":[0.5,1.0]}
pcloud = pass_through_filter(bounding_box, pcloud)
# Remove supurius points as well.
pcloud, _ = pcloud.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

# Downsample the pointcloud for the sake of speed if that's something you need.
# pcloud = pcloud.voxel_down_sample(voxel_size=0.004)

# Fit a plane to the filtered volume using Open3D.
plane_model, inliers = pcloud.segment_plane(distance_threshold=0.01,
                                         ransac_n=3,
                                         num_iterations=1000)
[a, b, c, d] = plane_model
print(f"Open3D plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# From the fitted plane, find the points that lie inside and outside the plane.
open3d_plane_pcloud = pcloud.select_by_index(inliers)
open3d_plane_pcloud.paint_uniform_color([1,0,0])  # Red
outlier_cloud = pcloud.select_by_index(inliers, invert=True)


# Fit a plane to the filtered volume using pyRANSAC-3D.
equation, inliers = pyrsc.Plane().fit(pts=np.asarray(pcloud.points), thresh=0.01, maxIteration=30)
[a, b, c, d] = equation
print(f"pyRANSAC plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
# From the fitted plane, find the points that lie inside and outside the plane.
pyransac_plane_pcloud = pcloud.select_by_index(inliers)
pyransac_plane_pcloud.paint_uniform_color([0,1,0])  # Green
outlier_cloud = pcloud.select_by_index(inliers, invert=True)
outlier_cloud.paint_uniform_color([0,1,1])  # Aqua


# Attempt to fit a cylinder to the outliers from the plane using pyRANSAC-3D
center, axis, radius, inliers = pyrsc.Cylinder().fit(np.asarray(outlier_cloud.points), thresh=0.01, maxIteration=1000)
print("Cylinder axis:", axis)
print("cylinder center:", center)
print("cylinder radius:", radius)
cylinder_pcloud = outlier_cloud.select_by_index(inliers)
cylinder_pcloud.paint_uniform_color([1,0,1])  # Purple

# Generate a mesh of the fitted cylinder.
cylinder_mesh = o3d.geometry.TriangleMesh.create_cylinder(radius=radius, height=0.1)
cylinder_mesh.compute_vertex_normals()
cylinder_mesh.paint_uniform_color([0, 0, 1])  # Purple
# rotation = pyrsc.get_rotationMatrix_from_vectors([0,0,1], [a,b,c])
rotation = pyrsc.get_rotationMatrix_from_vectors([0,0,1], axis)
cylinder_mesh = cylinder_mesh.rotate(rotation, center=[0,0,0])
cylinder_mesh = cylinder_mesh.translate(center)

# Plot some combination of meshes and point clouds.
o3d.visualization.draw_geometries([pyransac_plane_pcloud, cylinder_pcloud])
# o3d.visualization.draw_geometries([pyransac_plane_pcloud, cylinder_mesh])
