#!/usr/bin/env python3
"""Turn "who is going where" into conflict-free routes, and notice when they rot.

`cbs.FleetPlanner` solves one snapshot of the fleet. This module is the small
amount of state the running scheduler needs on top of that:

* it knows the committed route of every robot, so a robot that is already
  driving can be re-tasked when somebody else's new leg invalidates its path
  (that is the difference between "one robot got a CBS plan" and "the fleet is
  running one" - the plan has to survive the next dispatch);
* it decides when a route has changed *enough* to be worth preempting a moving
  robot, because re-sending an almost identical path every few seconds makes
  MPPI stutter;
* it keeps the per-robot radius used for the MAPF inflation, so the heavy AMR
  is planned as the wider vehicle it is.
"""

from __future__ import annotations

import math
import threading

from .cbs import FleetPlanner


def _dist_point_segment(p, a, b):
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    if dx == 0.0 and dy == 0.0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def deviation(points, polyline):
    """How far `points` stray from `polyline` (max point-to-segment distance)."""
    if len(polyline) < 2:
        return math.inf
    worst = 0.0
    for p in points:
        d = min(_dist_point_segment(p, polyline[i], polyline[i + 1])
                for i in range(len(polyline) - 1))
        worst = max(worst, d)
    return worst


class TrafficPlanner:
    """CBS for the whole fleet, plus the bookkeeping the scheduler needs."""

    def __init__(self, map_yaml, radii, names, cell=0.20, time_limit=2.0,
                 max_expansions=1200, margin=0.06, retask_tol=0.80,
                 logger=None):
        self.radii = dict(radii)
        self.names = list(names)
        self.retask_tol = float(retask_tol)
        self.logger = logger
        self._lock = threading.Lock()
        self._planner = None
        self._map_yaml = map_yaml
        self._cell = cell
        self._time_limit = time_limit
        self._max_expansions = max_expansions
        self._margin = margin
        self.routes = {}          # name -> Route currently being executed
        self.stats = {"plans": 0, "fallbacks": 0, "retasks": 0, "seconds": 0.0}

    def _planner_once(self):
        if self._planner is None:
            self._planner = FleetPlanner(
                self._map_yaml, cell=self._cell, time_limit=self._time_limit,
                max_expansions=self._max_expansions, margin=self._margin)
        return self._planner

    def plan(self, poses, targets):
        """plan(poses, targets) -> (routes, info).

        poses:   {name: (x, y)} where each robot is right now
        targets: {name: Pose2D} for robots with an active leg (absent = parked)
        """
        fleet = []
        for name in self.names:
            if name not in poses:
                continue
            tgt = targets.get(name)
            goal = (tgt.x, tgt.y) if tgt is not None else poses[name]
            fleet.append((name, poses[name], goal, self.radii[name],
                          tgt is not None))
        with self._lock:
            planner = self._planner_once()
            routes, info = planner.plan(fleet)
            self.stats["plans"] += 1
            self.stats["seconds"] += float(info.get("seconds", 0.0))
            if info.get("fallback"):
                self.stats["fallbacks"] += 1
        return routes, info

    def retask_list(self, routes, exclude=None):
        """Which already-committed routes the new plan has invalidated."""
        out = []
        for name, route in routes.items():
            if name == exclude:
                continue
            old = self.routes.get(name)
            if old is None:
                continue
            if deviation(old.waypoints, route.waypoints) > self.retask_tol:
                out.append(name)
        return out

    def commit(self, routes):
        self.routes.update(routes)

    def clear(self, name):
        self.routes.pop(name, None)
