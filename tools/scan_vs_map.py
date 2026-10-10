#!/usr/bin/env python3
"""Compare the robots' real Gazebo lidar scans with what the map predicts.

The occupancy grid is only "right" if it describes the same room the lidars
see: Nav2 builds its costmap from the scan and its global plan from the map, so
every disagreement is either a phantom obstacle (the map blocks a passage the
laser looks straight through) or a hole (the map is free where the laser hits
something). This script measures that, beam by beam, in the running simulation.

It runs the rays itself against maps/geomap.pgm - a Bresenham walk from the
robot's pose until it enters an occupied cell - and lines the result up with
the beam ranges the GPU lidar actually reported.

    # with the simulation up (ros2 launch hetero_fleet_bringup bringup.launch.py ...)
    python3 tools/scan_vs_map.py
    python3 tools/scan_vs_map.py --png generated/map_coords/scan_vs_map.png

The robots are assumed to be at their `spawn` pose from robot_profiles.yaml,
which is where they sit until the scheduler gives them work.
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from PIL import Image, ImageDraw

import map_common as mc


def load_map(tree, basename="src/hetero_fleet_bringup/maps/geomap"):
    base = Path(basename)
    if not base.is_absolute():
        base = tree / base
    meta = yaml.safe_load(base.with_suffix(".yaml").read_text())
    img = np.asarray(Image.open(base.with_suffix(".pgm")).convert("L"))
    return img < 128, float(meta["resolution"]), float(meta["origin"][0]), \
        float(meta["origin"][1])


def ray_cast(occ, res, ox, oy, x, y, ang, max_range, step=0.5):
    """Range to the first occupied cell along `ang`, or `max_range`."""
    H = occ.shape[0]
    dx, dy = math.cos(ang), math.sin(ang)
    n = int(max_range / (res * step)) + 1
    for i in range(1, n + 1):
        d = i * res * step
        px, py = x + dx * d, y + dy * d
        c = int((px - ox) / res)
        r = H - 1 - int((py - oy) / res)
        if not (0 <= r < H and 0 <= c < occ.shape[1]):
            return max_range
        if occ[r, c]:
            return d
    return max_range


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robots", default=None,
                    help="comma separated robot names (default: the whole fleet)")
    ap.add_argument("--map", default="src/hetero_fleet_bringup/maps/geomap",
                    help="basename of the .pgm/.yaml pair to test against")
    ap.add_argument("--tol", type=float, default=0.5,
                    help="range error (m) that counts as a disagreement")
    ap.add_argument("--timeout", type=float, default=60.0)
    ap.add_argument("--png", default=None)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--model", default="geomap_combined12_noroof")
    ap.add_argument("--neighbour", type=float, default=2.0,
                    help="beams that stop this close to another robot are that "
                         "robot, not the room (robots are not in the map)")
    ap.add_argument("--no-mesh", action="store_true",
                    help="skip the 'is the blocking geometry something the room "
                         "shows' test (slower but more informative)")
    a = ap.parse_args()
    tree = Path(__file__).resolve().parents[1]

    occ, res, ox, oy = load_map(tree, a.map)
    H, W = occ.shape
    # what the room shows from above: used to tell "the map blocks the furniture
    # footprint the scan sees" (intended - you cannot drive under a desk) apart
    # from "the map blocks empty air" (a sealed passage)
    visible = None
    if not a.no_mesh:
        V, F = mc.load_obj(tree / "src/hetero_fleet_gz/models" / a.model
                           / "meshes" / f"{a.model}.obj")
        tri = V[F]
        fzmin = tri[:, :, 2].min(axis=1)
        fzmax = tri[:, :, 2].max(axis=1)
        _area, fnz, _cen = mc.face_geometry(V, F)
        floorish = (np.abs(fnz) > 0.7) & (fzmax < 0.20)
        sel = (fzmax > 0.15) & (fzmin < 2.0) & ~floorish
        visible = mc.rasterise_footprint(V, F[sel], res, ox, oy, (H, W))
        import scipy.ndimage as ndi
        # everything the mesh has at all (any height), to tell "the map is
        # missing a piece of the scan" from "the lidar hit something that is
        # not in the visual mesh at all"
        any_geom = mc.rasterise_footprint(V, F, res, ox, oy, (H, W))
        band = (fzmin <= 0.45) & (fzmax >= 0.05) & ~floorish
        robot_h = mc.rasterise_footprint(V, F[band], res, ox, oy, (H, W))
        visible = ndi.binary_fill_holes(
            ndi.binary_closing(visible, structure=np.ones((3, 3))))
    profiles = yaml.safe_load(
        (tree / "src/hetero_fleet_description/config/robot_profiles.yaml").read_text())
    poses = {r["name"]: r["spawn"] for r in profiles["fleet"]}
    names = a.robots.split(",") if a.robots else list(poses)
    others = {n: (p[0], p[1]) for n, p in poses.items()}

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan

    rclpy.init()

    class Probe(Node):
        def __init__(self):
            super().__init__("scan_vs_map")
            self.scans = {}
            self.subs = [
                self.create_subscription(
                    LaserScan, f"/{n}/scan",
                    (lambda msg, n=n: self.scans.setdefault(n, msg)), 10)
                for n in names
            ]

    node = Probe()
    deadline = node.get_clock().now().nanoseconds * 1e-9 + a.timeout
    import time
    t0 = time.time()
    while len(node.scans) < len(names) and time.time() - t0 < a.timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
    rclpy.shutdown()

    missing = [n for n in names if n not in node.scans]
    if missing:
        print(f"no scan from: {', '.join(missing)} (is the simulation up?)")
    if not node.scans:
        return 1

    rgb = np.zeros((H, W, 3), np.uint8)
    rgb[occ] = (70, 70, 70)
    rgb[~occ] = (245, 245, 245)
    im = Image.fromarray(rgb)
    d = ImageDraw.Draw(im)

    print(f"map {W}x{H} @ {res} m, origin ({ox:.3f}, {oy:.3f})")
    print(f"{'robot':9s} {'beams':>6s} {'agree':>7s} {'hole':>6s} "
          f"{'sealed':>7s} {'furniture':>10s} {'robot':>6s}")
    total = dict(n=0, hole=0, phantom=0, furn=0, robot=0)
    holes, sealed = [], []
    for name in names:
        msg = node.scans.get(name)
        if msg is None:
            continue
        x, y = poses[name][0], poses[name][1]
        yaw = poses[name][2] if len(poses[name]) > 2 else 0.0
        meas = np.asarray(msg.ranges, dtype=np.float64)
        angs = msg.angle_min + np.arange(len(meas)) * msg.angle_increment
        rmax = min(float(msg.range_max), 20.0)
        pred = np.array([
            ray_cast(occ, res, ox, oy, x, y, yaw + t,
                     rmax if not np.isfinite(m) else min(rmax, max(m + 2.0, 1.0)))
            for t, m in zip(angs, meas)])
        ok = np.isfinite(meas) & (meas < rmax - 1e-3)
        # a beam that stops right on top of another robot is that robot, and
        # robots are deliberately absent from the static map
        is_robot = np.zeros(len(meas), bool)
        for t, m in zip(angs, meas):
            if not np.isfinite(m):
                continue
            ex, ey = x + math.cos(yaw + t) * m, y + math.sin(yaw + t) * m
            for oname, (ox2, oy2) in others.items():
                if oname != name and math.hypot(ex - ox2, ey - oy2) < a.neighbour:
                    is_robot[np.argmin(np.abs(angs - t))] = True
                    break
        # the map is free but the laser hit something  -> hole in the map
        hole = ok & (pred > meas + a.tol) & ~is_robot
        # the map blocks but the laser passes through -> phantom / sealed
        phantom = ok & (pred < meas - a.tol) & ~is_robot
        furn = np.zeros(len(meas), bool)
        if visible is not None:
            for t, m, ph in zip(angs, meas, phantom):
                if not ph:
                    continue
                # walk to the cell the map stopped at
                px = x + math.cos(yaw + t) * pred[np.argmin(np.abs(angs - t))]
                py = y + math.sin(yaw + t) * pred[np.argmin(np.abs(angs - t))]
                c = int((px - ox) / res)
                r = H - 1 - int((py - oy) / res)
                if 0 <= r < H and 0 <= c < W and visible[r, c]:
                    furn[np.argmin(np.abs(angs - t))] = True
        sealed_only = phantom & ~furn
        agree = ok & ~hole & ~phantom & ~is_robot
        err = meas - pred
        print(f"{name:9s} {int(ok.sum()):6d} {int(agree.sum()):7d} "
              f"{int(hole.sum()):6d} {int(sealed_only.sum()):7d} "
              f"{int(furn.sum()):10d} {int(is_robot.sum()):6d}")
        total["n"] += int(ok.sum())
        total["hole"] += int(hole.sum())
        total["phantom"] += int(sealed_only.sum())
        total["furn"] += int(furn.sum())
        total["robot"] += int(is_robot.sum())
        for t, m, h, ph in zip(angs, meas, hole, sealed_only):
            if not np.isfinite(m):
                continue
            ex, ey = x + math.cos(yaw + t) * m, y + math.sin(yaw + t) * m
            c = int((ex - ox) / res)
            r = H - 1 - int((ey - oy) / res)
            if not (0 <= r < H and 0 <= c < W):
                continue
            col = (230, 60, 60) if h else (80, 130, 255) if ph else (40, 160, 60)
            if h:
                holes.append((ex, ey, m, name))
            if ph:
                sealed.append((ex, ey, m, name))
            if not h and not ph:
                continue
            im.putpixel((c, r), col)
        r = H - 1 - int((y - oy) / res)
        c = int((x - ox) / res)
        d.ellipse([c - 3, r - 3, c + 3, r + 3], fill=(255, 160, 0))

    if total["n"]:
        n = total["n"]
        print(f"\n{n} beams: {100*(n-total['hole']-total['phantom']-total['robot'])/n:.1f}% "
              f"agree with the map")
        print(f"  {total['hole']:4d} HOLE      map says free, the lidar hits "
              f"something (the map is missing an obstacle)")
        print(f"  {total['phantom']:4d} SEALED    map blocks empty air "
              f"(the map invented an obstacle)")
        print(f"  {total['furn']:4d} FURNITURE map blocks a furniture footprint "
              f"the scan shows - intended, you cannot drive under a desk")
        print(f"  {total['robot']:4d} ROBOT     beam stopped on another robot, "
              f"which is not in the static map")
        for label, rows in (("HOLE", holes), ("SEALED", sealed)):
            if not rows:
                continue
            print(f"  {label} clusters:")
            seen = []
            for ex, ey, m, nm in sorted(rows):
                if any(math.hypot(ex - sx, ey - sy) < 2.0 for sx, sy, _, _ in seen):
                    continue
                seen.append((ex, ey, m, nm))
                c = int((ex - ox) / res)
                r = H - 1 - int((ey - oy) / res)
                tag = ""
                if 0 <= r < H and 0 <= c < W:
                    tag = ("scan has geometry here" if any_geom[r, c] else
                           "NOTHING in the visual mesh here")
                    if robot_h[r, c]:
                        tag += " (robot-height)"
                    if visible[r, c] if visible is not None else False:
                        tag += " (shown from above)"
                print(f"     ({ex:6.2f},{ey:7.2f})  range {m:.2f} m  {nm}  {tag}")
    if a.png:
        out = tree / a.png
        out.parent.mkdir(parents=True, exist_ok=True)
        im.save(out)
        print(f"wrote {out}  (green agree, red = map free but lidar hit, "
              f"blue = map blocks but lidar sees through)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
