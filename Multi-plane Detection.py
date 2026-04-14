import open3d as o3d
import numpy as np
import os
import random

def detect_multiple_planes(pcd, max_planes=5, distance_threshold=0.01, min_points=300):
    plane_details = []
    planes = []
    colors = []

    remaining_cloud = pcd

    for i in range(max_planes):
        print(f"\n🔎 Detecting Plane {i+1}...")
        if len(remaining_cloud.points) < min_points:
            print("❌ Not enough points left for more planes.")
            break

        plane_model, inliers = remaining_cloud.segment_plane(distance_threshold=distance_threshold,
                                                              ransac_n=3,
                                                              num_iterations=1000)
        if len(inliers) < min_points:
            print("❌ Not enough inliers found for a new plane.")
            break

        inlier_cloud = remaining_cloud.select_by_index(inliers)
        remaining_cloud = remaining_cloud.select_by_index(inliers, invert=True)

        points = np.asarray(inlier_cloud.points)
        length = points[:, 0].max() - points[:, 0].min()
        width  = points[:, 1].max() - points[:, 1].min()
        height = points[:, 2].max() - points[:, 2].min()

        a, b, c, d = plane_model
        plane_info = {
            'plane_id': i+1,
            'equation': (a, b, c, d),
            'length_mm': length,
            'width_mm': width,
            'height_mm': height,
            'num_points': len(inliers)
        }
        plane_details.append(plane_info)

        # Assign random color
        color = [random.random(), random.random(), random.random()]
        inlier_cloud.paint_uniform_color(color)
        colors.append(color)
        planes.append(inlier_cloud)

        print(f"✅ Plane {i+1}: {len(inliers)} points")
        print(f"   Equation: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0")
        print(f"   Dimensions (mm) → L: {length:.2f}, W: {width:.2f}, H: {height:.2f}")

    return planes, remaining_cloud, plane_details

def analyze_multi_planes(file_path):
    if not os.path.exists(file_path):
        print("❌ File not found. Please check the path.")
        return

    print(f"📂 Loading file: {file_path}")
    mesh = o3d.io.read_triangle_mesh(file_path)
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_poisson_disk(number_of_points=8000)

    # Original cloud view
    o3d.visualization.draw_geometries([pcd], window_name='Original Point Cloud')

    # Detect multiple planes
    planes, leftovers, plane_info_list = detect_multiple_planes(pcd)

    # Visualize result
    all_clouds = planes + [leftovers.paint_uniform_color([0.6, 0.6, 0.6])]
    o3d.visualization.draw_geometries(all_clouds, window_name='Detected Planes')

    print("\n📊 Summary of Planes:")
    for info in plane_info_list:
        print(f"Plane {info['plane_id']}: L={info['length_mm']:.2f}mm, W={info['width_mm']:.2f}mm, H={info['height_mm']:.2f}mm, Points={info['num_points']}")

# ---------------------- MAIN ----------------------
if __name__ == "__main__":
    file_path = input("📤 Enter full path to your STL file: ").strip().strip('"')
    analyze_multi_planes(file_path)
