#!/usr/bin/env python3
"""
Stage 1b: derive a collision-only mesh from the reconstructed visual mesh.

The Poisson-reconstructed floor (environment.obj) is a noisy trimesh with
+/- ~8 cm bumps. Used as the drive surface, the TurtleBot3 came to rest on
floor bumps with its wheels barely loaded and did not move under /cmd_vel
(and earlier fell through gaps into free fall). So model.sdf uses:
  visual    = environment.obj (unchanged, looks like the scan)
  collision = this mesh (walls / obstacles only) + a flat ground plane at z=0

Removed triangles: near-horizontal ones (|n_z| > 0.7) entirely below 0.25 m
(floor), and anything entirely below 0.12 m (floor clutter / skirt noise).
Walls keep everything above that, so the lidar (~0.27 m) and the robot body
still collide with them.

Usage: python3 scripts/make_collision_mesh.py
"""
import numpy as np

MESH_DIR = "src/lastmile_description/models/scanned_environment/meshes"
IN_OBJ = f"{MESH_DIR}/environment.obj"
OUT_OBJ = f"{MESH_DIR}/environment_collision.obj"


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
    Fk = F[~drop]
    used = np.unique(Fk)
    remap = -np.ones(len(V), int)
    remap[used] = np.arange(len(used))
    with open(OUT_OBJ, "w") as f:
        f.write("# Collision-only mesh derived from environment.obj by scripts/make_collision_mesh.py:\n"
                "# near-horizontal floor triangles (<0.25 m) and everything below 0.12 m removed.\n"
                "# The floor is replaced by a flat ground plane in model.sdf for stable wheel contact.\n")
        for v in V[used]:
            f.write(f"v {v[0]:.5f} {v[1]:.5f} {v[2]:.5f}\n")
        for a, b, c in remap[Fk] + 1:
            f.write(f"f {a} {b} {c}\n")
    print(f"{len(F)} triangles -> removed {int(drop.sum())} floor/low triangles, kept {len(Fk)} -> {OUT_OBJ}")


if __name__ == "__main__":
    main()
