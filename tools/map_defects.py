#!/usr/bin/env python3
"""Report the three defects a scanned map most often has, before the fleet does.

  1. HEIGHT BAND. How much of the collision mesh lives *only* above the robots
     (door lintels, beams, ducts). Projected top-down those seal the doorway
     they hang over, which is what `prepare_map_mesh.py --drop-above` removes.
  2. GRID SPECKS. Obstacle islands inside drivable floor ("pits" in RViz) and
     free pockets inside obstacles ("pinpricks"), plus the drivable area lost to
     the min-width filter.
  3. DOORWAYS. With --z-limit, which areas the height filter would newly
     connect to the main region, and the bottleneck clearance of the doorway
     each one is reached through (a robot fits only if its footprint's
     inscribed radius + costmap inflation fits that number - 0.49 m for the
     light AMR, 0.51 heavy, 0.53 omni in this fleet).

    python3 tools/map_defects.py                       # report the shipped map
    python3 tools/map_defects.py --z-limit 1.5 --doorway
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml

import make_nav2_map as nav
import map_common as mc


def height_band(collision_obj):
    V, F = mc.load_obj(collision_obj)
    area, _, _ = mc.face_geometry(V, F)
    tri = V[F]
    zmin = tri[:, :, 2].min(axis=1)
    zmax = tri[:, :, 2].max(axis=1)
    for lim in (0.5, 1.0, 1.5, 1.8, 2.0):
        sel = zmin > lim
        print(f"   faces whose LOWEST point is above {lim:.1f} m: {sel.sum():7d} "
              f"({100*sel.mean():5.1f}% of faces, {area[sel].sum():7.1f} m2)")
    print(f"   mesh z range: {V[:, 2].min():+.2f} .. {V[:, 2].max():+.2f} m")


def grid_report(occupied, meta):
    res = meta["res"]
    free = ~occupied
    lab_f, n_f = ndimage.label(free)
    sizes_f = ndimage.sum(free, lab_f, range(1, n_f + 1)) * res * res
    main = int(np.argmax(sizes_f)) + 1
    print(f"   free {sizes_f.sum():7.1f} m2, components {n_f:3d}, "
          f"largest {sizes_f.max():7.1f} m2 ({100*sizes_f.max()/sizes_f.sum():.0f}%)")
    specks = (sizes_f < 0.5)
    print(f"   free pockets < 0.5 m2: {specks.sum()} "
          f"({sizes_f[specks].sum():.2f} m2)")
    unreachable = [(i + 1, sizes_f[i]) for i in range(n_f) if i + 1 != main]
    if unreachable:
        print(f"   unreachable free pockets: {len(unreachable)} blobs, "
              f"{sum(a for _, a in unreachable):.1f} m2, biggest "
              f"{max(a for _, a in unreachable):.1f} m2 "
              f"-> pass --fill-unreachable <m2> to make_nav2_map.py to fill them")
    else:
        print("   unreachable free pockets: 0")

    lab_o, n_o = ndimage.label(occupied)
    sizes_o = ndimage.sum(occupied, lab_o, range(1, n_o + 1)) * res * res
    border = set(lab_o[0, :]) | set(lab_o[-1, :]) | set(lab_o[:, 0]) | set(lab_o[:, -1])
    border.discard(0)
    islands = [(i + 1, sizes_o[i]) for i in range(n_o) if (i + 1) not in border]
    if islands:
        smallest = min(a for _, a in islands)
        print(f"   obstacle islands inside the map: {len(islands)}, "
              f"smallest {smallest:.2f} m2, islands < 0.5 m2: "
              f"{sum(1 for _, a in islands if a < 0.5)}")
    else:
        print("   obstacle islands inside the map: 0")
    return lab_f == main


def doorways(visual, collision, z_limit, need):
    occ0, meta = nav.build_map(visual, collision, res=0.05, margin=0.6,
                               z_limit=None, verbose=False)
    occ1, _ = nav.build_map(visual, collision, res=0.05, margin=0.6,
                            z_limit=z_limit, verbose=False)
    res = meta["res"]
    free0, free1 = ~occ0, ~occ1
    lab0, _ = ndimage.label(free0)
    old_main = lab0 == (np.argmax(ndimage.sum(free0, lab0, range(1, lab0.max() + 1))) + 1)
    dist1 = ndimage.distance_transform_edt(free1) * res
    lab1, _ = ndimage.label(free1)
    sizes1 = ndimage.sum(free1, lab1, range(1, lab1.max() + 1))
    new_main = lab1 == (int(np.argmax(sizes1)) + 1)
    opened = free1 & ~free0
    print(f"   cells that open up: {opened.sum()*res*res:.1f} m2; "
          f"cells that get blocked: {(free0 & ~free1).sum()*res*res:.1f} m2")
    were_other = new_main & ~old_main
    lab_r, n_r = ndimage.label(were_other)
    sizes_r = ndimage.sum(were_other, lab_r, range(1, n_r + 1)) * res * res
    print(f"   areas newly connected to the main region: {n_r} "
          f"({sizes_r.sum():.1f} m2)")
    for i in np.argsort(sizes_r)[::-1][:8]:
        sel = lab_r == i + 1
        lo, hi = 0.0, float(dist1[sel].max())
        for _ in range(20):
            mid = 0.5 * (lo + hi)
            ok = free1 & (dist1 >= mid)
            lab_t, _ = ndimage.label(ok)
            left = set(np.unique(lab_t[ok & old_main])) - {0}
            right = set(np.unique(lab_t[ok & sel])) - {0}
            if left & right:
                lo = mid
            else:
                hi = mid
        verdict = "usable" if lo >= need else "TOO TIGHT"
        print(f"     {sizes_r[i]:7.1f} m2 behind a doorway of clearance "
              f"{lo:5.2f} m (width {2*lo:5.2f} m) -> {verdict} "
              f"(fleet needs {need:.2f} m)")


def main():
    ap = argparse.ArgumentParser()
    tree = Path(__file__).resolve().parents[1]
    ap.add_argument("--model", default="geomap_combined12_noroof")
    ap.add_argument("--z-limit", type=float, default=1.5,
                    help="height whose overhead-only geometry is dropped")
    ap.add_argument("--need", type=float, default=0.53,
                    help="clearance the widest robot needs (inscribed + inflation)")
    ap.add_argument("--doorway", action="store_true",
                    help="also rebuild with --z-limit and report doorway widths")
    ap.add_argument("--passages", action="store_true",
                    help="also check that --fill-notches does not seal any "
                         "passage (it is supposed to guard against that)")
    a = ap.parse_args()

    meshes = tree / "src/hetero_fleet_gz/models" / a.model / "meshes"
    visual = meshes / f"{a.model}.obj"
    collision = meshes / f"{a.model}_collision.obj"
    print(f"model: {a.model}")
    print("\n1) collision mesh height band")
    height_band(collision)
    print("\n2) occupancy grid as Nav2 loads it")
    occ, meta = nav.build_map(visual, collision, res=0.05, margin=0.6,
                              z_limit=None, fill_pits=0.15, fill_pockets=0.5,
                              fill_unreachable=20.0, fill_notches=1.0,
                              free_floating=True,
                              edits=(yaml.safe_load(
                                  (tree / "src/hetero_fleet_bringup/maps/map_edits.yaml"
                                   ).read_text()) or {})
                              if (tree / "src/hetero_fleet_bringup/maps/map_edits.yaml"
                                  ).exists() else None,
                              verbose=False)
    grid_report(occ, meta)
    if a.doorway:
        print(f"\n3) what dropping everything above {a.z_limit} m would open")
        doorways(visual, collision, a.z_limit, a.need)
    if a.passages:
        print("\n4) does --fill-notches seal any passage?")
        edits_path = tree / "src/hetero_fleet_bringup/maps/map_edits.yaml"
        edits = (yaml.safe_load(edits_path.read_text()) or {}) if edits_path.exists() else None
        common = dict(res=0.05, margin=0.6, z_limit=None, smooth=1, min_width=0.60,
                      fill_pits=0.15, fill_pockets=0.5, fill_unreachable=0.0,
                      edits=edits, free_floating=True, verbose=False)
        occ0, meta = nav.build_map(visual, collision, fill_notches=0.0, **common)
        occ1, _ = nav.build_map(visual, collision, fill_notches=1.0, **common)
        res, ox, oy, H = meta["res"], meta["xmin"], meta["ymin"], meta["H"]
        added = (~occ0) & occ1
        lab_add, n_add = ndimage.label(added, structure=np.ones((3, 3)))
        free1, free0 = ~occ1, ~occ0
        lab1, _ = ndimage.label(free1)
        lab0, _ = ndimage.label(free0)
        sealed = 0
        for i in range(1, n_add + 1):
            sel = lab_add == i
            ring = ndimage.binary_dilation(sel, iterations=2) & free1
            sides1 = set(np.unique(lab1[ring])) - {0}
            if len(sides1) < 2:
                continue
            ring0 = ndimage.binary_dilation(sel, iterations=2) & free0
            sides0 = set(np.unique(lab0[ring0])) - {0}
            if len(sides0) > 1:
                continue
            sealed += 1
            rr, cc = np.where(sel)
            print(f"   SEALED passage: {sel.sum()*res*res:.2f} m2 plug at "
                  f"({ox+cc.mean()*res:6.2f},{oy+(H-rr.mean())*res:7.2f})")
        print(f"   closing added {(added.sum()*res*res):.1f} m2 of fill, "
              f"sealed passages: {sealed}"
              f"{'  <- should be 0' if sealed else '  (guard working)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
