#!/usr/bin/env python3
"""
Stage 1b: derive two meshes from the reconstructed mesh (environment.obj).

1. environment_collision.obj - collision only.
   The Poisson-reconstructed floor is a noisy trimesh with +/- ~8 cm bumps.
   Used as the drive surface, the TurtleBot3 came to rest on floor bumps with
   its wheels barely loaded and did not move under /cmd_vel (and earlier fell
   through gaps into free fall). So model.sdf collides with this mesh (walls /
   obstacles only) + a flat ground plane at z = 0.
   Removed: near-horizontal triangles (|n_z| > 0.7) entirely below 0.25 m
   (floor) and anything entirely below 0.12 m (floor clutter / skirt noise).
   Walls keep everything above that, so the lidar (~0.27 m) and the robot body
   still collide with them.

2. environment_visual.obj - what the Gazebo camera draws.
   The scan includes the ceiling, so from Gazebo's GUI camera the corridors
   look like a closed box and the robot is hidden inside. Triangles reaching
   above CUT_Z are dropped from the visual copy only.

Usage: python3 scripts/make_collision_mesh.py
"""
import numpy as np

MESH_DIR = "src/lastmile_description/models/scanned_environment/meshes"
IN_OBJ = f"{MESH_DIR}/environment.obj"
OUT_OBJ = f"{MESH_DIR}/environment_collision.obj"
OUT_VISUAL = f"{MESH_DIR}/environment_visual.obj"
CUT_Z = 2.0


def write_obj(path, V, Fk, header):
    used = np.unique(Fk)
    remap = -np.ones(len(V), int)
    remap[used] = np.arange(len(used))
    with open(path, "w") as f:
        f.write(header)
        for v in V[used]:
            f.write(f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n")
        for a, b, c in remap[Fk] + 1:
            f.write(f"f {a} {b} {c}\n")


def main():
    V, F = [], []
    for line in open(IN_OBJ):
        if line.startswith("v "):
            V.append(line.split()[1:4])
        elif line.startswith("f "):
            F.append([int(t.split("/")[0]) - 1 for t in line.split()[1:4]])
    V, F = np.array(V, float), np.array(F)
    T = V[F]
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    zmax = T[:, :, 2].max(1)

    drop = ((np.abs(n[:, 2]) > 0.7) & (zmax < 0.25)) | (zmax < 0.12)
    write_obj(OUT_OBJ, V, F[~drop],
              "# Collision-only mesh derived from environment.obj by scripts/make_collision_mesh.py:\n"
              "# near-horizontal floor triangles (<0.25 m) and everything below 0.12 m removed.\n"
              "# The floor is replaced by a flat ground plane in model.sdf for stable wheel contact.\n")
    print(f"{len(F)} triangles -> removed {int(drop.sum())} floor/low triangles, kept {int((~drop).sum())} -> {OUT_OBJ}")

    keep = zmax <= CUT_Z
    write_obj(OUT_VISUAL, V, F[keep],
              f"# Visual-only cut-away of environment.obj: triangles above {CUT_Z:.1f} m removed (ceiling).\n")
    print(f"visual cut-away: kept {int(keep.sum())} of {len(F)} triangles -> {OUT_VISUAL}")


if __name__ == "__main__":
    main()
