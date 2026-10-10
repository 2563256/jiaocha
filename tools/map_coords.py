#!/usr/bin/env python3
"""Turn the occupancy map into something a human can point at.

Writes a coordinate reference sheet for the scanned map: a full-map PNG with a
1 m / 5 m grid, four zoomed tiles, a text version with axis rulers, and the
table of zone / charger / spawn coordinates.

The map frame is the same one Nav2, RViz and the SDF world use: metres, x to the
east, y to the north, origin at the world origin. Every cell of maps/geomap.pgm
is 0.05 m, and the picture keeps that 1 px = 1 cell mapping, so a pixel read off
the PNG converts to world coordinates as

    x = origin_x + col * 0.05        y = origin_y + (H - row) * 0.05

Usage:
    python3 tools/map_coords.py [--out generated/map_coords]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw, ImageFont


def load(tree):
    meta = yaml.safe_load((tree / "src/hetero_fleet_bringup/maps/geomap.yaml").read_text())
    img = np.asarray(
        Image.open(tree / "src/hetero_fleet_bringup/maps" / meta["image"]).convert("L"))
    occupied = img < 128
    res = float(meta["resolution"])
    ox, oy = float(meta["origin"][0]), float(meta["origin"][1])
    return occupied, res, ox, oy


def zones_and_spawns(tree):
    """Read the zone markers out of the generated world and the spawns out of
    the robot profiles, so the reference sheet cannot drift from the world."""
    import re

    world = (tree / "src/hetero_fleet_gz/worlds/geomap_fleet.sdf").read_text()
    zones = {}
    for name, x, y in re.findall(
            r"<model name='zone_(\w+)'>\s*<static>true</static>\s*<pose>([-\d.]+) ([-\d.]+)",
            world):
        zones[name] = (float(x), float(y))
    profiles = yaml.safe_load(
        (tree / "src/hetero_fleet_description/config/robot_profiles.yaml").read_text())
    spawns = {r["name"]: (r["spawn"][0], r["spawn"][1]) for r in profiles["fleet"]}
    return zones, spawns


def render(occupied, res, ox, oy, zones, spawns, scale=1, crop=None,
           grid_minor=1.0, grid_major=5.0):
    """One PNG. `crop` = (x0, x1, y0, y1) world box, else the whole grid."""
    H, W = occupied.shape
    if crop is None:
        c0, c1, r1, r0 = 0, W, H, 0
        x0, x1 = ox, ox + W * res
        y1, y0 = oy + H * res, oy
    else:
        x0, x1, y0, y1 = crop
        c0 = max(0, int((x0 - ox) / res))
        c1 = min(W, int((x1 - ox) / res) + 1)
        r0 = max(0, H - int((y1 - oy) / res) - 1)
        r1 = min(H, H - int((y0 - oy) / res) + 1)
    sub = occupied[r0:r1, c0:c1]
    h, w = sub.shape
    px = max(1, int(round(scale)))
    arr = np.full((h * px, w * px, 3), 245, dtype=np.uint8)
    block = np.kron(sub, np.ones((px, px), bool))
    arr[block] = (35, 35, 35)

    # grid
    im = Image.fromarray(arr)
    draw = ImageDraw.Draw(im)
    wx0 = ox + c0 * res
    wy1 = oy + (H - r0) * res
    n_minor = n_major = 0
    v = np.ceil(wx0 / grid_minor) * grid_minor
    while v <= ox + c1 * res:
        cx = (v - ox) / res - c0
        if cx >= 0:
            major = abs(v / grid_major - round(v / grid_major)) < 1e-6
            colour = (200, 60, 60) if major else (180, 200, 220)
            x_px = int(cx * px)
            draw.line([(x_px, 0), (x_px, h * px)], fill=colour, width=1)
            if major:
                n_major += 1
                draw.text((x_px + 2, 2), f"x={v:g}", fill=(150, 0, 0))
        v += grid_minor
    v = np.ceil((oy + (H - r1) * res) / grid_minor) * grid_minor
    while v <= wy1:
        ry = (H - (v - oy) / res) - r0
        if 0 <= ry < h:
            major = abs(v / grid_major - round(v / grid_major)) < 1e-6
            colour = (200, 60, 60) if major else (180, 200, 220)
            y_px = int(ry * px)
            draw.line([(0, y_px), (w * px, y_px)], fill=colour, width=1)
            if major:
                draw.text((2, y_px + 2), f"y={v:g}", fill=(150, 0, 0))
        v += grid_minor

    def to_px(x, y):
        return ((x - ox) / res - c0) * px, (H - (y - oy) / res - r0) * px

    for name, (x, y) in zones.items():
        if not (ox + c0 * res <= x <= ox + c1 * res
                and oy + (H - r1) * res <= y <= wy1):
            continue
        cxp, cyp = to_px(x, y)
        colour = (20, 160, 60) if name.startswith("pickup") else (
            (30, 90, 220) if name.startswith("dropoff") else (230, 170, 10))
        draw.ellipse([cxp - 3 * px, cyp - 3 * px, cxp + 3 * px, cyp + 3 * px],
                     outline=colour, width=2 * px)
        draw.text((cxp + 4 * px, cyp - 4 * px), name, fill=colour)
    for name, (x, y) in spawns.items():
        if not (ox + c0 * res <= x <= ox + c1 * res
                and oy + (H - r1) * res <= y <= wy1):
            continue
        cxp, cyp = to_px(x, y)
        draw.rectangle([cxp - 2 * px, cyp - 2 * px, cxp + 2 * px, cyp + 2 * px],
                       outline=(160, 0, 160), width=2 * px)
        draw.text((cxp + 4 * px, cyp + 1), name, fill=(140, 0, 140))

    draw.text((w * px - 260, h * px - 12),
              f"free=white obstacle=black 1px={res/px:.3f} m "
              f"x[{wx0:.1f},{ox + c1*res:.1f}] y[{oy+(H-r1)*res:.1f},{wy1:.1f}]",
              fill=(90, 90, 90))
    return im


def ascii_map(occupied, res, ox, oy, step_x=0.5, step_y=1.0):
    """Text version with a ruler, for terminals / pasting into a message."""
    H, W = occupied.shape
    cols = int(round(W * res / step_x))
    rows = int(round(H * res / step_y))
    out = []
    header = "      "
    for c in range(cols):
        x = ox + (c + 0.5) * step_x
        header += "|" if abs(x % 10) < step_x / 2 else " "
    out.append(header + "   (每 10 m 一个 '|')")
    for r in range(rows):
        y = oy + H * res - (r + 0.5) * step_y
        r0 = int(r * step_y / res)
        r1 = int((r + 1) * step_y / res)
        line = f"{y:6.1f}"
        for c in range(cols):
            c0 = int(c * step_x / res)
            c1 = int((c + 1) * step_x / res)
            block = occupied[r0:max(r1, r0 + 1), c0:max(c1, c0 + 1)]
            frac = block.mean()
            line += "#" if frac > 0.6 else ("+" if frac > 0.25 else " ")
        out.append(line + "|")
    return "\n".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="generated/map_coords")
    tree = Path(__file__).resolve().parents[1]
    a = ap.parse_args()
    out = tree / a.out
    out.mkdir(parents=True, exist_ok=True)

    occupied, res, ox, oy = load(tree)
    zones, spawns = zones_and_spawns(tree)
    H, W = occupied.shape
    x1, y1 = ox + W * res, oy + H * res

    full = render(occupied, res, ox, oy, zones, spawns, scale=1)
    full.save(out / "map_full.png")

    # four tiles that actually cover the building (y -76 .. -8), 2 px/cell so
    # the 1 m gridlines stay readable; the rest of the grid is empty black
    # (y_low, y_high) - render() takes a box as (x0, x1, y_low, y_high)
    spans = [(-25.0, -8.0), (-42.0, -25.0), (-59.0, -42.0), (-76.0, -59.0)]
    names = ["y-08_-25", "y-25_-42", "y-42_-59", "y-59_-76"]
    for (ylo, yhi), name in zip(spans, names):
        render(occupied, res, ox, oy, zones, spawns, scale=2, crop=(ox, x1, ylo, yhi),
               grid_minor=1.0, grid_major=5.0).save(out / f"map_{name}.png")

    txt = []
    txt.append(f"地图坐标参照  origin=({ox:.3f}, {oy:.3f})  res={res} m  "
               f"grid={W}x{H}  x[{ox:.2f},{x1:.2f}] y[{oy:.2f},{y1:.2f}]")
    txt.append("\n== 任务点 / 充电位 ==")
    for name, (x, y) in zones.items():
        txt.append(f"  {name:10s} ({x:7.2f}, {y:7.2f})")
    txt.append("\n== 出生点（= 充电位） ==")
    for name, (x, y) in spawns.items():
        txt.append(f"  {name:10s} ({x:7.2f}, {y:7.2f})")
    txt.append("\n== 文本地图（每 0.5 m 一列、1 m 一行，# = 障碍，+ = 半格）==")
    txt.append(ascii_map(occupied, res, ox, oy))
    (out / "map_coords.txt").write_text("\n".join(txt))

    print(f"wrote {out}/map_full.png  ({full.size[0]}x{full.size[1]} px)")
    for name in names:
        print(f"wrote {out}/map_{name}.png")
    print(f"wrote {out}/map_coords.txt")
    print(f"地图范围 x[{ox:.2f}, {x1:.2f}] y[{oy:.2f}, {y1:.2f}]，"
          f"1 px = {res} m（放大图 1 px = {res/2:.3f} m）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
