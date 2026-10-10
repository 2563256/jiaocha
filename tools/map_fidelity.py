#!/usr/bin/env python3
"""Score the occupancy map against the Gazebo top view, in three named defects.

RViz shows maps/geomap.pgm; Gazebo shows the coloured scan. The two disagree
whenever the map-marking pipeline drops, moves or invents geometry, and the
disagreement always shows up as one of three things a human spots instantly:

  1. SEALED PASSAGE  - the map blocks floor that Gazebo shows open, and doing
     so cuts the drivable space in two. (Counted only when the blob really
     separates free space: an obstacle in the middle of a room blocks nothing.)
  2. MISSING FURNITURE - Gazebo shows a solid object, the map is free there.
  3. HOLE IN FURNITURE - a furniture footprint the map does show, but with
     free cells enclosed inside it, so RViz renders a ring instead of a block.

Ground truth is the VISUAL mesh - the same one Gazebo renders and the 2D
lidars ray-cast against - projected top-down, with the floor and the geometry
that only exists above door height (`--top`, default 2.0 m) removed. That is
what the room looks like from above.

    python3 tools/map_fidelity.py                 # the shipped map
    python3 tools/map_fidelity.py --map /tmp/regen/geomap
    python3 tools/map_fidelity.py --png generated/map_coords/fidelity.png
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

import map_common as mc


def load_map(basename):
    base = Path(basename)
    meta = yaml.safe_load(base.with_suffix(".yaml").read_text())
    img = np.asarray(Image.open(base.with_suffix(".pgm")).convert("L"))
    return img < 128, float(meta["resolution"]), float(meta["origin"][0]), \
        float(meta["origin"][1])


def blobs_report(sel, res, ox, oy, H, top=8, min_area=0.05):
    lab, n = ndimage.label(sel, structure=np.ones((3, 3)))
    rows = []
    if n:
        objs = ndimage.find_objects(lab, max_label=n)
        for i in range(1, n + 1):
            sl = objs[i - 1]
            if sl is None:
                continue
            area = int((lab[sl] == i).sum()) * res * res
            if area < min_area:
                continue
            r = 0.5 * (sl[0].start + sl[0].stop)
            c = 0.5 * (sl[1].start + sl[1].stop)
            rows.append((area, ox + c * res, oy + (H - r) * res))
    rows.sort(key=lambda t: -t[0])
    return rows[:top]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="src/hetero_fleet_bringup/maps/geomap",
                    help="basename of the .pgm/.yaml pair to score")
    ap.add_argument("--model", default="geomap_combined12_noroof")
    ap.add_argument("--top", type=float, default=2.0,
                    help="geometry whose LOWEST point is above this hangs over "
                         "the robots (lintels, beams) and must not project")
    ap.add_argument("--floor", type=float, default=0.15,
                    help="geometry that never rises above this is floor/tape")
    ap.add_argument("--band-hi", type=float, default=0.45,
                    help="top of the robot's own extent, for the physics check")
    ap.add_argument("--top-n", type=int, default=8)
    ap.add_argument("--png", default=None)
    a = ap.parse_args()
    tree = Path(__file__).resolve().parents[1]

    occ, res, ox, oy = load_map(tree / a.map if not Path(a.map).is_absolute()
                                else a.map)
    H, W = occ.shape
    meshes = tree / "src/hetero_fleet_gz/models" / a.model / "meshes"
    V, F = mc.load_obj(meshes / f"{a.model}.obj")
    tri = V[F]
    fzmin = tri[:, :, 2].min(axis=1)
    fzmax = tri[:, :, 2].max(axis=1)
    _area, fnz, _cen = mc.face_geometry(V, F)
    floorish = (np.abs(fnz) > 0.7) & (fzmax < 0.20)

    # what Gazebo shows from above: everything above the floor that is not a
    # door lintel / beam / duct hanging over the fleet
    visible = (fzmax > a.floor) & (fzmin < a.top) & ~floorish
    gt = mc.rasterise_footprint(V, F[visible], res, ox, oy, (H, W))
    gt = ndimage.binary_fill_holes(
        ndimage.binary_closing(gt, structure=np.ones((3, 3))))
    # what the robots can actually hit (the lidars live in this band)
    band = (fzmin <= a.band_hi) & (fzmax >= 0.05) & ~floorish
    robot_h = mc.rasterise_footprint(V, F[band], res, ox, oy, (H, W))
    robot_h = ndimage.binary_closing(robot_h, structure=np.ones((3, 3)))
    # the scanned footprint: everything the scan covers, floor included. Used
    # to tell "the map blocks a wall where there is nothing" apart from "the
    # map blocks the ground outside the building", which is correct.
    cover = mc.rasterise_footprint(V, F, res, ox, oy, (H, W))
    k = max(1, int(round(0.5 / res)) | 1)
    cover = ndimage.binary_fill_holes(
        ndimage.binary_closing(cover, structure=np.ones((k, k))))

    free = ~occ
    lab_f, n_f = ndimage.label(free)
    sizes = ndimage.sum(free, lab_f, range(1, n_f + 1)) * res * res
    print(f"map {W}x{H} @ {res} m, origin ({ox:.3f}, {oy:.3f})")
    print(f"  free {sizes.sum():7.1f} m2, obstacle {occ.sum()*res*res:7.1f} m2, "
          f"free components {n_f} (largest {sizes.max():.1f} m2)")

    # ---- 1. sealed passages -------------------------------------------
    phantom = occ & ~gt & cover
    phantom_outside = occ & ~gt & ~cover
    lab_t, _ = ndimage.label(gt & ~robot_h)          # Gazebo-open floor
    sizes_t = ndimage.sum(gt & ~robot_h, lab_t, range(1, lab_t.max() + 1)) * res * res
    lab_p, n_p = ndimage.label(phantom, structure=np.ones((3, 3)))
    objs = ndimage.find_objects(lab_p, max_label=n_p) if n_p else []
    sealed = []
    for i in range(1, n_p + 1):
        sl = objs[i - 1]
        if sl is None:
            continue
        blob = lab_p[sl] == i
        if blob.sum() * res * res < 0.05:
            continue
        pad = 3
        r0, r1 = max(0, sl[0].start - pad), min(H, sl[0].stop + pad)
        c0, c1 = max(0, sl[1].start - pad), min(W, sl[1].stop + pad)
        ring = ndimage.binary_dilation(lab_p[r0:r1, c0:c1] == i,
                                       iterations=2) & ~(lab_p[r0:r1, c0:c1] == i)
        sides = set(np.unique(lab_t[r0:r1, c0:c1][ring])) - {0}
        big = sorted((sizes_t[s - 1] for s in sides), reverse=True)
        if len(big) >= 2 and big[1] >= 0.5:
            r = 0.5 * (sl[0].start + sl[0].stop)
            c = 0.5 * (sl[1].start + sl[1].stop)
            sealed.append((blob.sum() * res * res, ox + c * res, oy + (H - r) * res))
    sealed.sort(key=lambda t: -t[0])
    print(f"\n1) sealed passages: {phantom.sum()*res*res:.1f} m2 of map obstacle "
          f"inside the building shows open in Gazebo"
          f" (+{phantom_outside.sum()*res*res:.0f} m2 outside it, correctly "
          f"blocked); {len(sealed)} blobs cut drivable space in two")
    for area, x, y in sealed[:a.top_n]:
        print(f"     {area:5.2f} m2 sealing at ({x:6.2f},{y:7.2f})")

    # ---- 2. missing furniture -----------------------------------------
    missing = gt & free
    rows = blobs_report(missing, res, ox, oy, H, a.top_n)
    print(f"\n2) missing furniture: {missing.sum()*res*res:.1f} m2 that Gazebo "
          f"shows and the map leaves drivable")
    for area, x, y in rows:
        print(f"     {area:5.2f} m2 at ({x:6.2f},{y:7.2f})")

    # ---- 3. holes inside furniture ------------------------------------
    inside = ndimage.binary_fill_holes(gt) & ~gt
    holes = missing & ndimage.binary_dilation(gt, iterations=1)
    lab_h, n_h = ndimage.label(holes, structure=np.ones((3, 3)))
    enclosed = 0.0
    rows = []
    if n_h:
        objs_h = ndimage.find_objects(lab_h, max_label=n_h)
        for i in range(1, n_h + 1):
            sl = objs_h[i - 1]
            if sl is None:
                continue
            sel = lab_h[sl] == i
            # a hole is enclosed when its ring is entirely furniture
            ring = ndimage.binary_dilation(sel, iterations=2) & ~sel
            if ring.any() and gt[sl][ring].mean() > 0.8:
                area = int(sel.sum()) * res * res
                enclosed += area
                r = 0.5 * (sl[0].start + sl[0].stop)
                c = 0.5 * (sl[1].start + sl[1].stop)
                rows.append((area, ox + c * res, oy + (H - r) * res))
    rows.sort(key=lambda t: -t[0])
    print(f"\n3) holes inside furniture: {enclosed:.1f} m2 of drivable cells "
          f"enclosed by furniture ({len(rows)} blobs)")
    for area, x, y in rows[:a.top_n]:
        print(f"     {area:5.2f} m2 at ({x:6.2f},{y:7.2f})")

    # ---- physics consistency ------------------------------------------
    danger = free & robot_h
    print(f"\nphysics: {danger.sum()*res*res:.1f} m2 the map calls drivable "
          f"although the mesh has geometry at robot height")

    if a.png:
        rgb = np.zeros((H, W, 3), np.uint8)
        rgb[occ] = (70, 70, 70)
        rgb[free] = (245, 245, 245)
        rgb[gt & occ] = (40, 130, 40)
        rgb[missing] = (235, 60, 60)
        rgb[phantom] = (80, 130, 255)
        Image.fromarray(rgb).save(a.png)
        print(f"\nwrote {a.png}  (green agree, red missing furniture, "
              f"blue blocked-but-open)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
