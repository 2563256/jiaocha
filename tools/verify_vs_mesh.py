#!/usr/bin/env python3
"""Compare the occupancy grid with the Gazebo mesh, cell by cell, both ways.

The mesh is the ground truth: it is what Gazebo renders, what the 2D lidars
scan, and therefore what the robots can actually hit.

    A) map = obstacle, mesh has nothing at robot height
         -> a passage the map blocks that Gazebo leaves open. If freeing the
            blob reconnects drivable space, it is a wrongly sealed passage.
    B) map = free, mesh HAS geometry at robot height
         -> the map lets the fleet drive into something it can see. Should be
            empty; anything here is a hole in the map.

Robot height band: geometry whose z-range intersects [--band-lo, --band-hi]
(default 0.25 .. 0.45 m). Floor faces (near-horizontal, at floor level) are
not obstacles.

    python3 tools/verify_vs_mesh.py [--band-lo 0.25] [--band-hi 0.45]
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy import ndimage

import map_common as mc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="geomap_combined12_noroof")
    ap.add_argument("--band-lo", type=float, default=0.25,
                    help="lowest geometry height that counts as an obstacle")
    ap.add_argument("--band-hi", type=float, default=0.45,
                    help="geometry above this does not reach the robots")
    ap.add_argument("--top", type=int, default=12)
    a = ap.parse_args()
    tree = Path(__file__).resolve().parents[1]

    meta = yaml.safe_load((tree / "src/hetero_fleet_bringup/maps/geomap.yaml").read_text())
    img = np.asarray(Image.open(tree / "src/hetero_fleet_bringup/maps" / meta["image"]).convert("L"))
    occ = img < 128
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    H, W = occ.shape

    meshes = tree / "src/hetero_fleet_gz/models" / a.model / "meshes"
    V, F = mc.load_obj(meshes / f"{a.model}.obj")
    tri = V[F]
    zmin = tri[:, :, 2].min(axis=1)
    zmax = tri[:, :, 2].max(axis=1)
    _area, fnz, _cen = mc.face_geometry(V, F)
    floorish = (np.abs(fnz) > 0.7) & (zmax < 0.20)
    band = (zmin <= a.band_hi) & (zmax >= a.band_lo) & ~floorish
    solid = mc.rasterise_footprint(V, F[band], res, ox, oy, (H, W))
    solid = ndimage.binary_closing(solid, structure=np.ones((3, 3)))
    cover = mc.rasterise_footprint(V, F, res, ox, oy, (H, W))
    cover = ndimage.binary_fill_holes(
        ndimage.binary_closing(cover, structure=np.ones((11, 11))))
    print(f"mesh {a.model}: {len(F)} faces, {band.sum()} in the robot's height band "
          f"[{a.band_lo}, {a.band_hi}] m")
    print(f"map: {W}x{H} @ {res} m, free {((~occ).sum())*res*res:.1f} m2, "
          f"obstacle {occ.sum()*res*res:.1f} m2")

    free = ~occ
    lab, _ = ndimage.label(free)
    sizes = ndimage.sum(free, lab, range(1, lab.max() + 1)) * res * res
    main = int(np.argmax(sizes)) + 1
    # the mesh's own drivable space: inside the scanned footprint, away from
    # geometry the robots can hit. This is the reference for "did the map seal
    # a passage", not the map's own free space (which fills have already eaten)
    truth_free = cover & ~solid
    lab_t, _ = ndimage.label(truth_free)

    # ---- A: blocked but nothing at robot height -------------------------
    blocked_open = occ & ~solid & cover
    lab_a, n_a = ndimage.label(blocked_open, structure=np.ones((3, 3)))
    print(f"\nA) map blocks it, Gazebo has nothing at robot height: "
          f"{blocked_open.sum()*res*res:.1f} m2 in {n_a} blobs "
          f"({(occ & ~solid & ~cover).sum()*res*res:.1f} m2 more is unscanned ground "
          f"outside the building, which must stay blocked)")
    rows = []
    for i in range(1, n_a + 1):
        sel = lab_a == i
        area = sel.sum() * res * res
        if area < 0.05:
            continue
        # a blob "seals a passage" when the free space on two sides of it
        # belongs to two different components: freeing it reconnects them
        ring = ndimage.binary_dilation(sel, iterations=2) & truth_free
        sides = set(np.unique(lab_t[ring])) - {0}
        if len(sides) < 2:
            continue
        side_sizes = sorted((ndimage.sum(lab_t == c) * res * res for c in sides),
                            reverse=True)
        gain = side_sizes[0]
        rr, cc = np.where(sel)
        rows.append((gain, area, ox + cc.mean() * res, oy + (H - rr.mean()) * res))
    rows.sort(key=lambda t: -t[0])
    print(f"   blobs that SEPARATE two drivable components (freeing them joins "
          f"{len(rows)} passage(s)):")
    for gain, area, x, y in rows[:a.top]:
        print(f"     +{gain:6.2f} m2 if freed | blob {area:5.2f} m2 at ({x:6.2f},{y:7.2f})")
    if not rows:
        print("     none")

    # ---- B: free but the mesh has geometry there ------------------------
    free_solid = free & solid
    lab_b, n_b = ndimage.label(free_solid, structure=np.ones((3, 3)))
    print(f"\nB) map says free, Gazebo has robot-height geometry: "
          f"{free_solid.sum()*res*res:.1f} m2 in {n_b} blobs")
    rows_b = []
    for i in range(1, n_b + 1):
        sel = lab_b == i
        area = sel.sum() * res * res
        if area < 0.05:
            continue
        rr, cc = np.where(sel)
        rows_b.append((area, ox + cc.mean() * res, oy + (H - rr.mean()) * res))
    rows_b.sort(key=lambda t: -t[0])
    for area, x, y in rows_b[:a.top]:
        print(f"     {area:6.2f} m2 hole at ({x:6.2f},{y:7.2f})")
    if not rows_b:
        print("     none - the map covers everything the fleet can hit")
    return 0


if __name__ == "__main__":
    sys.exit(main())
