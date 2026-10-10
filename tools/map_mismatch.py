#!/usr/bin/env python3
"""Draw where the occupancy map disagrees with the Gazebo mesh.

  BLUE  = the map blocks it, but the mesh has no geometry at robot height
          (0.25-0.45 m): a passage Gazebo leaves open. If a workstation sits
          there it is deliberate - that is why the picture exists.
  RED   = the map leaves it free, but the mesh has robot-height geometry:
          a hole in the map the fleet could drive into.
  GREEN = the mesh's own geometry at robot height (walls, furniture legs) -
          what the lidars see.

Writes one PNG per 17 m slice plus the full map, same grid as map_coords.py.

    python3 tools/map_mismatch.py [--out generated/map_coords]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw
from scipy import ndimage

import map_common as mc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="generated/map_coords")
    ap.add_argument("--model", default="geomap_combined12_noroof")
    ap.add_argument("--band-lo", type=float, default=0.25)
    ap.add_argument("--band-hi", type=float, default=0.45)
    a = ap.parse_args()
    tree = Path(__file__).resolve().parents[1]
    out = tree / a.out
    out.mkdir(parents=True, exist_ok=True)

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
    solid = ndimage.binary_closing(
        mc.rasterise_footprint(V, F[band], res, ox, oy, (H, W)),
        structure=np.ones((3, 3)))
    cover = mc.rasterise_footprint(V, F, res, ox, oy, (H, W))
    cover = ndimage.binary_fill_holes(
        ndimage.binary_closing(cover, structure=np.ones((11, 11))))

    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[occ] = (60, 60, 60)
    rgb[~occ] = (250, 250, 250)
    strict = occ & ~solid & cover
    loose = ~occ & solid
    rgb[strict] = (90, 140, 255)      # map too strict
    rgb[loose] = (230, 70, 70)        # map too loose
    rgb[solid & occ] = (40, 120, 40)  # both agree it blocks

    print(f"map obstacle {occ.sum()*res*res:.1f} m2 | mesh robot-height geometry "
          f"{solid.sum()*res*res:.1f} m2")
    print(f"  blue  (map blocks, Gazebo open): {strict.sum()*res*res:.1f} m2")
    print(f"  red   (map open, Gazebo blocked): {loose.sum()*res*res:.1f} m2")
    print(f"  green (both agree: wall/furniture at robot height): "
          f"{(solid & occ).sum()*res*res:.1f} m2")

    def save(scale, crop, name):
        if crop is None:
            c0, c1, r0, r1 = 0, W, 0, H
        else:
            x0, x1, y0, y1 = crop
            c0 = max(0, int((x0 - ox) / res))
            c1 = min(W, int((x1 - ox) / res) + 1)
            r0 = max(0, H - int((y1 - oy) / res) - 1)
            r1 = min(H, H - int((y0 - oy) / res) + 1)
        sub = rgb[r0:r1, c0:c1]
        im = Image.fromarray(sub).resize(
            ((c1 - c0) * scale, (r1 - r0) * scale), Image.NEAREST)
        d = ImageDraw.Draw(im)
        # 5 m grid
        v = np.ceil((ox + c0 * res) / 5) * 5
        while v <= ox + c1 * res:
            cx = int(((v - ox) / res - c0) * scale)
            d.line([(cx, 0), (cx, im.size[1])], fill=(255, 200, 80), width=1)
            d.text((cx + 2, 2), f"x={v:g}", fill=(180, 120, 0))
            v += 5
        v = np.ceil((oy + (H - r1) * res) / 5) * 5
        while v <= oy + (H - r0) * res:
            ry = int((H - (v - oy) / res - r0) * scale)
            d.line([(0, ry), (im.size[0], ry)], fill=(255, 200, 80), width=1)
            if 0 <= ry < im.size[1]:
                d.text((2, ry + 2), f"y={v:g}", fill=(180, 120, 0))
            v += 5
        d.rectangle([0, im.size[1] - 14, im.size[0], im.size[1]], fill=(255, 255, 255))
        d.text((2, im.size[1] - 12),
               "blue = map blocks it but Gazebo is open | red = map open but Gazebo blocks | "
               "green = both say blocked", fill=(0, 0, 0))
        im.save(out / name)
        print(f"wrote {out / name} ({im.size[0]}x{im.size[1]})")

    x1 = ox + W * res
    save(1, None, "mismatch_full.png")
    for (ylo, yhi), nm in (((-25.0, -8.0), "y-08_-25"), ((-42.0, -25.0), "y-25_-42"),
                           ((-59.0, -42.0), "y-42_-59"), ((-76.0, -59.0), "y-59_-76")):
        save(2, (ox, x1, ylo, yhi), f"mismatch_{nm}.png")
    return 0


if __name__ == "__main__":
    sys.exit(main())
