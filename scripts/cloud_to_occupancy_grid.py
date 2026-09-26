#!/usr/bin/env python3
"""
Stage 2b: project the scanned point cloud into a 2D occupancy grid for
nav2_map_server (.pgm + .yaml).

Why this version exists (v2)
----------------------------
The first version took a fixed global height slice (0.08-1.80 m above z=0) and
flood-filled free space from the origin. On this scan that produced a badly
wrong map:
  * the floor is not perfectly planar after SLAM (it varies by ~+/-10 cm across
    the site), so real floor near the scan origin fell inside the "obstacle"
    band;
  * the area around the scan start (0, 0) is full of sparse "ghost" points
    (people walking past, sensor noise accumulated while the scanner sat
    there), which a fixed slice + any-point-occupies rule turns into a solid
    block of "wall";
  * together these made (0, 0) - which is really open corridor, 0.86 m from
    the nearest wall - look occupied, pushed the spawn into an actual wall,
    and flood-filled "free" space out to the bounding box.

Method (v2)
-----------
1. Load the raw PLY (numpy only, no open3d needed) and apply the RANSAC
   floor alignment from data/floor_transform.json (scripts/preprocess_cloud.py).
2. Local floor height: 0.25 m tiles, 10th percentile of low points per tile,
   holes filled from the nearest tile, 3x3 median smoothing. Heights are then
   measured *above the local floor* instead of above z = 0.
3. Obstacles need real vertical structure, not just a stray point: a 5 cm cell
   is occupied if it has >= 40 points spanning >= 8 distinct 5 cm height bins
   in the 0.12-1.60 m body band (or >= 150 points). Walls have hundreds of
   points over the full height; ghost points don't.
4. Free = cells where floor was actually observed (closed/hole-filled), minus
   obstacles, keeping the component connected to the spawn point (0, 0).
   Everything never observed stays unknown - so the planner (allow_unknown:
   false) cannot route through unscanned space.

Output keeps the same grid frame as v1 (origin/size), so nothing else changes.
Usage: python3 scripts/cloud_to_occupancy_grid.py [raw.ply] [floor_transform.json]
"""
import json
import sys

import numpy as np
from PIL import Image
from scipy import ndimage as ndi

IN_PLY = sys.argv[1] if len(sys.argv) > 1 else "data/map_ros_cloud.ply"
IN_TF = sys.argv[2] if len(sys.argv) > 2 else "data/floor_transform.json"
OUT_PGM = "src/lastmile_navigation/maps/lastmile_map.pgm"
OUT_YAML = "src/lastmile_navigation/maps/lastmile_map.yaml"

RES = 0.05                       # m / cell
ORIGIN = (-5.1899, -6.2513)      # grid origin (kept from v1 so all frames line up)
W, H = 529, 367                  # grid size in cells
SPAWN = (0.0, 0.0)               # the assignment's origin - free corridor
FLOOR_TILE = 5                   # cells per floor tile (0.25 m)
BAND = (0.12, 1.60)              # obstacle band above local floor (m)
FLOOR_TOL = 0.06                 # |height above floor| counted as floor (m)


def load_ply(path):
    raw = open(path, "rb").read()
    head_end = raw.index(b"end_header\n") + len(b"end_header\n")
    header = raw[:head_end].decode("ascii", "replace").splitlines()
    n = next(int(l.split()[2]) for l in header if l.startswith("element vertex"))
    props = []
    in_vertex = False
    for l in header:
        if l.startswith("element"):
            in_vertex = l.startswith("element vertex")
        elif l.startswith("property") and in_vertex:
            _, t, name = l.split()
            props.append((name, {"float": "<f4", "uchar": "u1", "double": "<f8", "int": "<i4"}[t]))
    arr = np.frombuffer(raw, dtype=np.dtype(props), count=n, offset=head_end)
    return np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float64)


def main():
    pts = load_ply(IN_PLY)
    tf = json.load(open(IN_TF))
    pts = pts @ np.array(tf["rotation_matrix"]).T
    pts[:, 2] -= tf["floor_z_offset_after_rotation"]
    print(f"{len(pts):,} points, floor-aligned")

    col = ((pts[:, 0] - ORIGIN[0]) / RES).astype(int)
    row = ((pts[:, 1] - ORIGIN[1]) / RES).astype(int)
    ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
    pts, col, row = pts[ok], col[ok], row[ok]
    z = pts[:, 2]

    # 2. local floor height
    T = FLOOR_TILE
    TH, TW = (H + T - 1) // T, (W + T - 1) // T
    tile = (row // T) * TW + (col // T)
    low = z < 0.45
    idx, zl = tile[low], z[low]
    order = np.argsort(idx, kind="stable")
    idx, zl = idx[order], zl[order]
    fl = np.full(TH * TW, np.nan)
    uniq, start, cnt = np.unique(idx, return_index=True, return_counts=True)
    for k, s, c in zip(uniq, start, cnt):
        if c >= 15:
            fl[k] = np.percentile(zl[s:s + c], 10)
    fl = fl.reshape(TH, TW)
    nearest = ndi.distance_transform_edt(np.isnan(fl), return_distances=False, return_indices=True)
    fl = ndi.median_filter(fl[tuple(nearest)], size=3)
    floor = np.kron(fl, np.ones((T, T)))[:H, :W]
    hag = z - floor[row, col]

    # 3. obstacles with vertical structure
    band = (hag > BAND[0]) & (hag < BAND[1])
    count = np.zeros((H, W), int)
    np.add.at(count, (row[band], col[band]), 1)
    zbins = np.zeros((H, W, 30), bool)
    zbins[row[band], col[band], ((hag[band] - BAND[0]) / 0.05).astype(int).clip(0, 29)] = True
    nbins = zbins.sum(2)
    occ = ((count >= 40) & (nbins >= 8)) | (count >= 150)
    occ = ndi.binary_opening(occ, structure=np.ones((2, 2))) | ((count >= 80) & (nbins >= 12))
    occ = ndi.binary_closing(occ, iterations=1)

    # 4. observed floor -> free, connected to spawn
    fmask = np.abs(hag) < FLOOR_TOL
    fcount = np.zeros((H, W), int)
    np.add.at(fcount, (row[fmask], col[fmask]), 1)
    seen = ndi.binary_fill_holes(ndi.binary_closing(fcount > 0, iterations=3))
    free = seen & ~occ
    sr, sc = int((SPAWN[1] - ORIGIN[1]) / RES), int((SPAWN[0] - ORIGIN[0]) / RES)
    labels, _ = ndi.label(free)
    assert labels[sr, sc] != 0, "spawn point is not in observed free space"
    free = labels == labels[sr, sc]

    grid = np.full((H, W), 205, np.uint8)
    grid[free] = 254
    grid[occ] = 0
    Image.fromarray(np.flipud(grid)).save(OUT_PGM)
    with open(OUT_YAML, "w") as f:
        f.write(f"image: lastmile_map.pgm\nmode: trinary\nresolution: {RES}\n"
                f"origin: [{ORIGIN[0]:.4f}, {ORIGIN[1]:.4f}, 0.0]\nnegate: 0\n"
                f"occupied_thresh: 0.65\nfree_thresh: 0.25\n")
    clearance = ndi.distance_transform_edt(~occ)[sr, sc] * RES
    print(f"wrote {OUT_PGM}: free={int(free.sum())} occupied={int(occ.sum())} "
          f"unknown={grid.size - int(free.sum()) - int(occ.sum())}; spawn (0,0) clearance {clearance:.2f} m")


if __name__ == "__main__":
    sys.exit(main())
