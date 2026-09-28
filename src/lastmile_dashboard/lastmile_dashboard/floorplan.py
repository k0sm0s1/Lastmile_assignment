"""
floorplan: turn the navigation occupancy grid into a clean, presentation-only
floor plan for the delivery page.

The navigation map (lastmile_map.pgm) is honest sensor data: walls are a few
pixels of scan noise, doorways and unscanned pockets leave gaps, and the west
wing's outline is ragged. That's right for AMCL and Nav2, but it reads as
"raw data" to a person. This module derives a clean plan from it, used ONLY
for drawing and for snapping tapped stops. Navigation is unaffected.

  1. floor  = cells observed as free (254)
  2. seal   = morphological closing (~0.35 m) + hole filling
              -> closes gaps between wall fragments and unscanned pockets
  3. tidy   = opening (~0.15 m) + keep the component that contains the dock
              -> drops spurs, specks and isolated islands
  4. smooth = upsample x SCALE, Gaussian blur, threshold at 0.5
              -> rounded, continuous outline instead of stair-stepped pixels
  5. draw   = floor fill + a uniform wall stroke around it (+ soft shadow)
"""
import numpy as np
from scipy import ndimage as ndi

SCALE = 4  # output pixels per map cell (5 cm -> 1.25 cm per pixel)

# Apple-Maps-like light palette (RGBA)
FLOOR = (252, 252, 253, 255)
WALL = (199, 199, 204, 255)      # systemGray4
SHADOW = (0, 0, 0, 16)


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    tokens, i = [], 0
    while len(tokens) < 4:
        while data[i:i + 1].isspace():
            i += 1
        if data[i:i + 1] == b"#":
            while data[i:i + 1] not in (b"\n", b""):
                i += 1
            continue
        j = i
        while not data[j:j + 1].isspace():
            j += 1
        tokens.append(data[i:j])
        i = j
    w, h = int(tokens[1]), int(tokens[2])
    img = np.frombuffer(data, np.uint8, count=w * h, offset=i + 1).reshape(h, w)
    return img  # row 0 = top (max y)


def disk(r):
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return x * x + y * y <= r * r


def clean_floor(pgm, dock_rc, close_cells=7, open_cells=3):
    """pgm: image rows (row 0 = top). dock_rc: (row, col) in the same image. Returns bool mask."""
    free = pgm >= 250
    sealed = ndi.binary_fill_holes(ndi.binary_closing(free, structure=disk(close_cells)))
    tidy = ndi.binary_opening(sealed, structure=disk(open_cells))
    lab, _ = ndi.label(tidy)
    keep = lab == lab[dock_rc] if lab[dock_rc] else tidy
    return ndi.binary_fill_holes(keep)


def render(mask, scale=SCALE, wall_px=None, sigma=None):
    """mask: bool (row 0 = top). Returns RGBA uint8 image upscaled by `scale`."""
    wall_px = wall_px or max(3, int(round(scale * 1.6)))
    sigma = sigma or scale * 1.1
    hi = ndi.zoom(mask.astype(np.float32), scale, order=1)
    hi = ndi.gaussian_filter(hi, sigma) > 0.5
    ring = ndi.binary_dilation(hi, structure=disk(wall_px)) & ~hi
    shadow = ndi.gaussian_filter(ndi.binary_dilation(hi, structure=disk(wall_px + 2)).astype(np.float32), scale * 2.5)
    shadow = np.roll(shadow, int(scale * 1.2), axis=0)
    H, W = hi.shape
    img = np.zeros((H, W, 4), np.uint8)
    a = np.clip(shadow * SHADOW[3], 0, 255).astype(np.uint8)
    img[..., 3] = a
    img[ring] = WALL
    img[hi] = FLOOR
    return img, hi


def rect(k):
    return np.ones((k, k), bool)


def regularize(mask, dock_rc, close_k=13, open_k=9):
    """Axis-aligned tidy-up: closing/opening with square elements straightens walls."""
    m = ndi.binary_opening(ndi.binary_closing(mask, structure=rect(close_k)), structure=rect(open_k))
    lab, _ = ndi.label(m)
    m = lab == lab[dock_rc] if lab[dock_rc] else m
    return ndi.binary_fill_holes(m)


def outline(mask, eps_cells=2.5, snap_deg=14.0):
    """Trace the floor outline and return simplified, axis-snapped polygons in cell units (x=col, y=row)."""
    import cv2
    cs, hier = cv2.findContours(mask.astype(np.uint8), cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    polys = []
    for c in cs:
        if cv2.contourArea(c) < 40:
            continue
        p = cv2.approxPolyDP(c, eps_cells, True)[:, 0, :].astype(float)
        p = _orthogonalize(p, snap_deg)
        polys.append(p)
    return polys


def _orthogonalize(p, snap_deg):
    """Make near-horizontal / near-vertical edges exactly axis-aligned (a few passes)."""
    t = np.tan(np.radians(snap_deg))
    n = len(p)
    for _ in range(3):
        for i in range(n):
            a, b = p[i], p[(i + 1) % n]
            dx, dy = b[0] - a[0], b[1] - a[1]
            if abs(dx) > 1e-6 and abs(dy) <= t * abs(dx):      # ~horizontal
                y = (a[1] + b[1]) / 2
                a[1] = b[1] = y
            elif abs(dy) > 1e-6 and abs(dx) <= t * abs(dy):    # ~vertical
                x = (a[0] + b[0]) / 2
                a[0] = b[0] = x
    # drop duplicate / collinear points
    out = []
    for i in range(n):
        a, b, c = p[i - 1], p[i], p[(i + 1) % n]
        if np.hypot(*(b - a)) < 0.5:
            continue
        cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0])
        if abs(cross) < 1e-6:
            continue
        out.append(b.copy())
    return np.array(out) if len(out) >= 3 else p
