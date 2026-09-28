#!/usr/bin/env python3
"""
make_nav_map.py: build the planner's "stay inside the building" map.

lastmile_map.pgm is the honest sensor map: AMCL localises against it, and
map_server loads its grey never-scanned pixels (205) as FREE (free_thresh
0.25), because some real floor was only partly scanned. The catch is that the
planner then also treats the unscanned space OUTSIDE the building as free. Where
a wall has a scan gap (the north wall of the main corridor between x=2..6 m, for
example), NavFn can cut through the gap and route the robot outside the floor
plan.

This script derives lastmile_nav_map.pgm, which only the costmaps use:
  inside the building  -> identical to lastmile_map.pgm (interior unscanned
                          pockets stay drivable)
  outside the building -> occupied
  corner keep-outs     -> occupied (see CORNER_KEEPOUTS)
"The building" is the same cleaned, regularised floor mask the delivery page
draws (floorplan.clean_floor + regularize), grown by MARGIN so real floor at the
edges is never clipped. The output is deterministic, so re-running it reproduces
the committed file byte for byte.

usage: python3 scripts/make_nav_map.py   (from the workspace root)
"""
import os
import sys

import numpy as np
from scipy import ndimage as ndi

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "src", "lastmile_dashboard", "lastmile_dashboard"))
import floorplan as fpm  # noqa: E402

MAPS = os.path.join(ROOT, "src", "lastmile_navigation", "maps")
MARGIN = 7  # cells (0.35 m) of slack around the floor mask: the regularised outline trims
            # some half-scanned real floor (west side of the north arm), and 0.15 m
            # left too narrow a gap beside the scanned clutter mid-arm
# Corner keep-outs, (x, y, radius) in metres: the two wall corners at the mouth of
# the north arm. DWB tends to cut a tight 90-degree turn, and at these corners
# that put the robot inside the wall's inscribed zone, where NavFn can't plan
# out again. A small round keep-out makes the turn wider.
CORNER_KEEPOUTS = [(8.0, 0.75, 0.25), (9.8, 0.75, 0.25)]


def main():
    meta = {}
    for line in open(os.path.join(MAPS, "lastmile_map.yaml")):
        if ":" in line and not line.strip().startswith("#"):
            k, v = line.split(":", 1)
            meta[k.strip()] = v.strip()
    res = float(meta["resolution"])
    ox, oy = [float(t) for t in meta["origin"].strip("[]").split(",")[:2]]
    pgm = fpm.read_pgm(os.path.join(MAPS, meta["image"]))
    H, W = pgm.shape
    dock_rc = (H - 1 - int((0.0 - oy) / res), int((0.0 - ox) / res))
    floor = fpm.regularize(fpm.clean_floor(pgm, dock_rc, 9, 3), dock_rc)
    inside = ndi.binary_dilation(floor, structure=fpm.disk(MARGIN))
    nav = pgm.copy()
    nav[~inside] = 0
    yy, xx = np.mgrid[0:H, 0:W]
    wx, wy = ox + (xx + 0.5) * res, oy + (H - 1 - yy + 0.5) * res
    for cx, cy, r in CORNER_KEEPOUTS:
        nav[(wx - cx) ** 2 + (wy - cy) ** 2 <= r * r] = 0
    with open(os.path.join(MAPS, "lastmile_nav_map.pgm"), "wb") as f:
        f.write(b"P5\n# lastmile_nav_map: lastmile_map.pgm with everything outside the building marked occupied\n")
        f.write(f"{W} {H}\n255\n".encode())
        f.write(nav.tobytes())
    with open(os.path.join(MAPS, "lastmile_nav_map.yaml"), "w") as f:
        f.write("image: lastmile_nav_map.pgm\nmode: trinary\n"
                f"resolution: {meta['resolution']}\norigin: {meta['origin']}\n"
                f"negate: 0\noccupied_thresh: {meta['occupied_thresh']}\nfree_thresh: {meta['free_thresh']}\n"
                "# Costmaps only (published on /nav_map by nav_map_server). AMCL keeps using lastmile_map.yaml.\n")
    print(f"nav map: {int((~inside).sum())} cells outside the building marked occupied, "
          f"{int(inside.sum())} inside ({inside.sum() * res * res:.1f} m^2)")


if __name__ == "__main__":
    main()
