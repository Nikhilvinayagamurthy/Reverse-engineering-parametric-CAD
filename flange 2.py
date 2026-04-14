import open3d as o3d
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from skimage.measure import ransac, CircleModel

def fit_circle_ransac(points, min_samples=30, residual_threshold=1.0, max_trials=1000):
    if len(points) < min_samples:
        return None, None
    try:
        model_robust, inliers = ransac(
            points, CircleModel, min_samples=min_samples,
            residual_threshold=residual_threshold, max_trials=max_trials
        )
        xc, yc, r = model_robust.params
        return (xc, yc, r), inliers
    except Exception:
        return None, None

def analyze_flange_stl(file_path, point_count=10000, visualize=True):
    if not os.path.exists(file_path):
        print("❌ File not found. Please check the path.")
        return

    mesh = o3d.io.read_triangle_mesh(file_path)
    if not mesh.has_vertices():
        print("❌ Mesh is empty or invalid.")
        return
    mesh.compute_vertex_normals()

    # Sample point cloud
    print("📌 Sampling mesh into point cloud...")
    pcd = mesh.sample_points_uniformly(number_of_points=point_count)
    points = np.asarray(pcd.points)
    points_2d = points[:, :2]

    # Estimate centroid
    centroid = np.mean(points_2d, axis=0)

    # Calculate all radii from estimated center
    dists = np.linalg.norm(points_2d - centroid, axis=1)

    # Outer circle: take top 3% of radii
    outer_thresh = np.percentile(dists, 97)
    outer_candidates = points_2d[(dists > outer_thresh - 3) & (dists < outer_thresh + 3)]
    outer, out_inliers = fit_circle_ransac(outer_candidates, min_samples=30, residual_threshold=2.0)
    if outer is None:
        print("❌ Could not fit outer flange circle.")
        return
    xc, yc, flange_radius = outer

    # Recompute all radii from detected flange center
    dists = np.linalg.norm(points_2d - np.array([xc, yc]), axis=1)

    # Center hole: take lowest 3% of radii
    center_thresh = np.percentile(dists, 3)
    center_candidates = points_2d[(dists > center_thresh - 2) & (dists < center_thresh + 2)]
    center, center_inliers = fit_circle_ransac(center_candidates, min_samples=20, residual_threshold=1.0)
    if center is None:
        print("❌ Could not fit center hole.")
        return
    center_x, center_y, center_hole_radius = center

    # Bolt holes: look for clusters at "middle ring" radius (improved logic)
    bolt_ring_min = center_hole_radius * 1.5
    bolt_ring_max = flange_radius * 0.85
    bolt_ring = (dists > bolt_ring_min) & (dists < bolt_ring_max)
    bolt_candidates = points_2d[bolt_ring]
    if len(bolt_candidates) < 12:
        print("❌ Not enough points for bolt holes.")
        return

    # Use DBSCAN to find clusters for bolt holes (each cluster = one bolt hole)
    # eps: about the diameter of a bolt hole, so we use center_hole_radius*0.9 or flange_radius*0.04
    eps = max(center_hole_radius * 0.9, flange_radius * 0.04)
    clustering = DBSCAN(eps=eps, min_samples=6).fit(bolt_candidates)
    labels = clustering.labels_

    bolt_centers = []
    bolt_radii = []
    bolt_circle_distances = []

    for label in set(labels):
        if label == -1:
            continue
        cluster_points = bolt_candidates[labels == label]
        if len(cluster_points) < 6:
            continue
        bx, by = cluster_points.mean(axis=0)
        distances = np.linalg.norm(cluster_points - [bx, by], axis=1)
        br = np.mean(distances)
        bolt_centers.append((bx, by))
        bolt_radii.append(br)
        bolt_circle_distances.append(np.linalg.norm([bx - xc, by - yc]))

    num_bolt_holes = len(bolt_centers)
    bolt_circle_radius = np.mean(bolt_circle_distances) if num_bolt_holes > 0 else 0
    bolt_hole_radius = np.mean(bolt_radii) if num_bolt_holes > 0 else 0

    # Print results
    print(f"\n--- Optimized Flange Parameters ---")
    print(f"Flange Radius         = {flange_radius:.2f}")
    print(f"Center Hole Radius    = {center_hole_radius:.2f}")
    print(f"Bolt Hole Radius      = {bolt_hole_radius:.2f}")
    print(f"Bolt Circle Radius    = {bolt_circle_radius:.2f}")
    print(f"Number of Bolt Holes  = {num_bolt_holes}")

    np.savetxt("flange_pointcloud.xyz", points, fmt="%.5f")
    print("✅ Point cloud saved as flange_pointcloud.xyz")

    # Plot for visual debugging
    if visualize:
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.scatter(points_2d[:, 0], points_2d[:, 1], s=2, c='grey', alpha=0.3, label='Sampled Points')
        ax.scatter(centroid[0], centroid[1], color="c", s=60, marker="*", label="Estimated Centroid")
        ax.add_patch(plt.Circle((xc, yc), flange_radius, color='b', fill=False, lw=2, label='Flange'))
        ax.add_patch(plt.Circle((center_x, center_y), center_hole_radius, color='r', fill=False, lw=2, label='Center Hole'))
        for i, ((bx, by), br) in enumerate(zip(bolt_centers, bolt_radii)):
            ax.add_patch(plt.Circle((bx, by), br, color='g', fill=False, lw=2, alpha=0.7))
            ax.scatter(bx, by, color='g', marker="x", s=60)
            ax.text(bx, by, f"{i+1}", color="g")
        ax.set_aspect('equal')
        ax.set_title('Optimized Flange Feature Detection')
        ax.legend()
        plt.tight_layout()
        plt.savefig("flange_detected_optimized.png", dpi=300)
        print("✅ Visualization saved as flange_detected_optimized.png")
        plt.show()

if __name__ == "__main__":
    file_path = input("📤 Enter full path to your flange STL file: ").strip().strip('"')
    analyze_flange_stl(file_path)
