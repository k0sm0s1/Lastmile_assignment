#!/usr/bin/env python3
"""
Stage 1 of the pipeline: load the raw scanned point cloud, clean it up, and
re-express it in a coordinate frame whose z=0 plane is the true floor.

Why this step exists:
  The raw cloud (map_ros_cloud.ply, PCL/RTAB-Map output) has the floor sitting
  around z ~= 0.1-0.15m rather than exactly z=0, because that's wherever the
  SLAM system's map frame happened to end up. Both downstream consumers
  (the Gazebo mesh and the Nav2 occupancy grid) need to agree on what "floor"
  means, so we fix that once here with a RANSAC plane fit instead of doing it
  twice, differently, in each consumer.

Outputs (into data/):
  - cleaned_cloud.ply   : outlier-removed, voxel-downsampled, floor-aligned
  - floor_transform.json: the rigid transform applied, for reference/debugging
"""
import json
import sys
import numpy as np
import open3d as o3d

IN_PATH = "data/map_ros_cloud.ply"
OUT_CLOUD = "data/cleaned_cloud.ply"
OUT_TRANSFORM = "data/floor_transform.json"

VOXEL_SIZE = 0.03          # 3cm voxels: enough detail for walls/furniture, ~10x fewer points
NB_NEIGHBORS = 20
STD_RATIO = 2.0
RANSAC_DIST_THRESHOLD = 0.02
RANSAC_N = 3
RANSAC_ITERS = 2000


def fit_floor_plane(pcd):
    """RANSAC-fit the dominant horizontal plane in the lower portion of the cloud.

    We restrict the candidate set to points in the bottom 40% of the height
    range first, otherwise RANSAC on the full cloud can lock onto a wall
    (walls have far more surface area than the floor in a corridor scan).
    """
    pts = np.asarray(pcd.points)
    z = pts[:, 2]
    z_lo, z_hi = np.percentile(z, [1, 99])
    band_mask = z < (z_lo + 0.4 * (z_hi - z_lo))
    floor_candidates = pcd.select_by_index(np.where(band_mask)[0])

    plane_model, inliers = floor_candidates.segment_plane(
        distance_threshold=RANSAC_DIST_THRESHOLD,
        ransac_n=RANSAC_N,
        num_iterations=RANSAC_ITERS,
    )
    a, b, c, d = plane_model
    normal = np.array([a, b, c])
    if normal[2] < 0:  # make sure normal points "up"
        normal, d = -normal, -d
    normal /= np.linalg.norm(normal)
    return normal, d, len(inliers), len(floor_candidates.points)


def rotation_to_align_z(normal):
    """Rotation matrix that maps `normal` onto [0, 0, 1]."""
    z_axis = np.array([0.0, 0.0, 1.0])
    v = np.cross(normal, z_axis)
    s = np.linalg.norm(v)
    c = np.dot(normal, z_axis)
    if s < 1e-8:
        return np.eye(3)
    vx = np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0],
    ])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s ** 2))


def main():
    print(f"Loading {IN_PATH} ...")
    pcd = o3d.io.read_point_cloud(IN_PATH)
    print(f"  {len(pcd.points):,} points loaded")

    print(f"Voxel downsampling at {VOXEL_SIZE}m ...")
    pcd = pcd.voxel_down_sample(VOXEL_SIZE)
    print(f"  {len(pcd.points):,} points after downsample")

    print(f"Statistical outlier removal (nb={NB_NEIGHBORS}, std_ratio={STD_RATIO}) ...")
    pcd, inlier_idx = pcd.remove_statistical_outlier(nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO)
    print(f"  {len(pcd.points):,} points after outlier removal")

    print("Fitting floor plane (RANSAC on bottom 40% height band) ...")
    normal, d, n_inliers, n_candidates = fit_floor_plane(pcd)
    print(f"  normal={normal}, d={d:.4f}, inliers={n_inliers}/{n_candidates}")

    R = rotation_to_align_z(normal)
    pcd.rotate(R, center=(0, 0, 0))

    # after rotation, the plane's offset along z is what we subtract to put floor at z=0
    pts = np.asarray(pcd.points)
    floor_z_after_rotation = np.median(pts[np.argsort(np.abs(pts[:, 2]))[:max(1, n_inliers)], 2]) \
        if False else None
    # simpler & robust: recompute floor height as the mode of the lower point cluster post-rotation
    z = pts[:, 2]
    hist, edges = np.histogram(z[z < np.percentile(z, 40)], bins=200)
    floor_z = edges[np.argmax(hist)]
    pts[:, 2] -= floor_z
    pcd.points = o3d.utility.Vector3dVector(pts)

    z2 = np.asarray(pcd.points)[:, 2]
    print(f"  floor placed at z=0 (was z={floor_z:.4f}); new z range [{z2.min():.2f}, {z2.max():.2f}]")

    o3d.io.write_point_cloud(OUT_CLOUD, pcd, write_ascii=False)
    print(f"Wrote {OUT_CLOUD}")

    with open(OUT_TRANSFORM, "w") as f:
        json.dump({
            "floor_normal_original_frame": normal.tolist(),
            "rotation_matrix": R.tolist(),
            "floor_z_offset_after_rotation": float(floor_z),
            "voxel_size": VOXEL_SIZE,
            "note": "Apply rotation R about origin, then subtract floor_z_offset from z, "
                    "to go from the ORIGINAL map_ros_cloud.ply frame into the floor-aligned frame "
                    "used by every other script in this repo.",
        }, f, indent=2)
    print(f"Wrote {OUT_TRANSFORM}")


if __name__ == "__main__":
    sys.exit(main())
