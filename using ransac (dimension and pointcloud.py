import open3d as o3d
import numpy as np
import os

def analyze_scaled_stl(file_path, target_dim_mm=32.0, point_count=5000):
    if not os.path.exists(file_path):
        print("❌ File not found. Please check the path.")
        return

    print(f"📂 Loading file: {file_path}")
    mesh = o3d.io.read_triangle_mesh(file_path)
    if not mesh.has_vertices():
        print("❌ Mesh is empty or invalid.")
        return
    mesh.compute_vertex_normals()

    vertices = np.asarray(mesh.vertices)
    print("🔍 Vertex Range Before Scaling:")
    print("Min:", vertices.min(axis=0))
    print("Max:", vertices.max(axis=0))
    print("Range:", vertices.max(axis=0) - vertices.min(axis=0))

    # Sample stable point cloud
    pcd = mesh.sample_points_uniformly(number_of_points=point_count)

    # Compute original bounding box
    aabb = pcd.get_axis_aligned_bounding_box()
    extent = aabb.get_extent()
    max_dim = max(extent)

    # Estimate scaling factor (based on assumption)
    scaling_factor = target_dim_mm / max_dim
    print(f"\n⚙️ Applying scaling factor: {scaling_factor:.4f}")

    # Scale the point cloud
    pcd.scale(scaling_factor, center=pcd.get_center())

    # Recalculate bounding box after scaling
    aabb_scaled = pcd.get_axis_aligned_bounding_box()
    aabb_scaled.color = (1, 0, 0)
    scaled_extent = aabb_scaled.get_extent()

    print("\n📏 Scaled Dimensions (in mm):")
    print(f"Length (X): {scaled_extent[0]:.2f} mm")
    print(f"Width  (Y): {scaled_extent[1]:.2f} mm")
    print(f"Height (Z): {scaled_extent[2]:.2f} mm")

    # Visualize
    o3d.visualization.draw_geometries([pcd, aabb_scaled], window_name="Scaled Mesh + Bounding Box")

# ---------------------- MAIN ----------------------
if __name__ == "__main__":
    file_path = input("📤 Enter full path to your STL file: ").strip().strip('"')
    analyze_scaled_stl(file_path)
