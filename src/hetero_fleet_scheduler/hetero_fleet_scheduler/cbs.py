#!/usr/bin/env python3
"""Conflict-Based Search (CBS) for the whole fleet, on the occupancy map.

WHY. Until now every robot planned for itself: Nav2's SMAC planned the shortest
path that avoided the *static* map, and teammates were handled afterwards, at
runtime, by two reactive mechanisms (other robots injected into the local
costmap as `robot_obstacles`, and a collision monitor that brakes). Reactive
avoidance cannot solve a *structural* conflict: two robots that need the same
1.0-1.6 m doorway from opposite sides will each stop in front of the other and
wait, and no amount of local re-planning gets them out - they have to *agree on
an order*. That is a multi-agent path finding (MAPF) problem, and CBS is the
standard optimal answer:

    high level  a constraint tree over conflicts: find two agents that collide,
                branch on "forbid agent A from being here at this time" and
                "forbid agent B ...", recursively, until no conflict is left
    low level   for each agent, A* over (cell, time) honouring its constraints

The result is one conflict-free, time-indexed plan per robot: who goes first
through the doorway, who waits, who takes the longer way round. This module
only *plans*; the robots still execute with MPPI and their costmaps, so the
safety net for the unexpected (a teammate that slipped, a moving obstacle) is
unchanged.

DELIBERATE SIMPLIFICATIONS, all of them about speed:

* The grid is DOWNSAMPLED (default 0.20 m) before anything else. CBS is
  exponential in the number of agents and the low-level search is per (cell,
  time) state, so planning on the 0.05 m Nav2 grid (1.09M cells here) is not
  affordable; 0.20 m gives ~18k free cells, which plans in tens of ms. A coarse
  cell is marked blocked if ANY fine cell inside it is blocked, so
  downsampling never opens a wall.
* Obstacles are inflated by each robot's own radius, so a robot is a point in
  the plan and "how much room does this robot need" is still per-type (the
  heavy AMR is 25% wider than the light one).
* Every robot is an agent, including the ones parked on a charger: their start
  equals their goal, which makes CBS treat them as obstacles that the movers
  have to route around. That is exactly the real constraint.
* When CBS cannot finish inside its budget (it is NP-hard), the caller gets the
  best plan found so far, and `plan_fleet` falls back to prioritized planning
  (plan one robot at a time, treating the already-planned routes as moving
  obstacles), which always returns something.

See tests/test_cbs.py for the swap / doorway / crossing cases.
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage

MOVE_COST = 1


@dataclass
class MapfGrid:
    """The occupancy map, downsampled for MAPF and inflated per robot type."""

    occupied: np.ndarray          # fine grid, True = obstacle
    res: float                    # fine grid resolution (m)
    origin: tuple                 # (x, y) of the fine grid's lower-left cell
    cell: float = 0.20            # coarse cell size (m)

    def __post_init__(self):
        h, w = self.occupied.shape
        ch = max(1, int(round(self.cell / self.res)))
        H = (h + ch - 1) // ch
        W = (w + ch - 1) // ch
        pad = np.zeros((H * ch, W * ch), bool)
        pad[:h, :w] = self.occupied
        # a coarse cell is blocked if anything inside it is blocked
        self.occ = pad.reshape(H, ch, W, ch).any(axis=(1, 3))
        self.H, self.W = H, W
        self._masks: dict[int, np.ndarray] = {}

    # ---- geometry ------------------------------------------------------
    def mask(self, radius_m: float) -> np.ndarray:
        """Traversability mask with obstacles grown by `radius_m`."""
        r = int(np.ceil(radius_m / self.cell - 1e-9))
        if r not in self._masks:
            if r <= 0:
                self._masks[r] = ~self.occ
            else:
                yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
                disk = (xx * xx + yy * yy) <= r * r
                self._masks[r] = ~ndimage.binary_dilation(self.occ, structure=disk)
        return self._masks[r]

    def to_cell(self, x: float, y: float) -> tuple:
        c = int((x - self.origin[0]) / self.cell)
        r = self.H - 1 - int((y - self.origin[1]) / self.cell)
        return int(np.clip(c, 0, self.W - 1)), int(np.clip(r, 0, self.H - 1))

    def to_world(self, c: int, r: int) -> tuple:
        x = self.origin[0] + (c + 0.5) * self.cell
        y = self.origin[1] + (self.H - r - 0.5) * self.cell
        return x, y

    def nearest_free(self, c: int, r: int, mask: np.ndarray, max_cells: int = 40):
        """Snap a pose to the closest cell the robot can actually occupy."""
        if mask[r, c]:
            return c, r
        for k in range(1, max_cells + 1):
            r0, r1 = max(0, r - k), min(self.H, r + k + 1)
            c0, c1 = max(0, c - k), min(self.W, c + k + 1)
            sub = mask[r0:r1, c0:c1]
            if sub.any():
                rr, cc = np.nonzero(sub)
                d = (rr + r0 - r) ** 2 + (cc + c0 - c) ** 2
                i = int(np.argmin(d))
                return int(cc[i] + c0), int(rr[i] + r0)
        return None


# ----------------------------------------------------------------------
# low level: A* over (cell, time) with vertex and edge constraints
# ----------------------------------------------------------------------
def _spatial_path(grid, start, goal, mask):
    """Static A*: avoid expanding equivalent waiting states before CBS starts."""
    heap = [(0, 0, start)]
    costs, parent = {start: 0}, {}
    while heap:
        _, cost, cell = heapq.heappop(heap)
        if cost != costs.get(cell):
            continue
        if cell == goal:
            path = [cell]
            while path[-1] != start:
                path.append(parent[path[-1]])
            return path[::-1]
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nxt = (cell[0]+dc, cell[1]+dr)
            if not (0 <= nxt[0] < grid.W and 0 <= nxt[1] < grid.H and mask[nxt[1],nxt[0]]):
                continue
            nc = cost+1
            if nc < costs.get(nxt, 1 << 30):
                costs[nxt], parent[nxt] = nc, cell
                heuristic = abs(nxt[0]-goal[0])+abs(nxt[1]-goal[1])
                heapq.heappush(heap, (nc+heuristic, nc, nxt))
    return None


def _astar(grid: MapfGrid, start, goal, mask, vcon, econ, horizon,
           earliest_goal=0, stats=None):
    """Shortest path from start to goal for one agent.

    vcon: set of (c, r, t) the agent may not OCCUPY
    econ: set of (c, r, c2, r2, t) moves the agent may not MAKE
    """
    H, W = grid.H, grid.W
    if start == goal and earliest_goal <= 0:
        return [start]

    def h(c, r):
        return abs(c - goal[0]) + abs(r - goal[1])

    def free(c, r):
        return 0 <= c < W and 0 <= r < H and mask[r, c]

    start_state = (start[0], start[1], 0)
    open_heap = [(h(*start), 0, start_state)]
    g = {start_state: 0}
    came = {}
    best_goal_state = None
    expanded = 0

    while open_heap:
        _f, cost, (c, r, t) = heapq.heappop(open_heap)
        if g.get((c, r, t), 1 << 30) < cost:
            continue
        expanded += 1
        if (c, r) == goal and t >= earliest_goal:
            best_goal_state = (c, r, t)
            break
        if t >= horizon:
            continue
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1), (0, 0)):
            c2, r2 = c + dc, r + dr
            if not free(c2, r2):
                continue
            if (c2, r2, t + 1) in vcon:
                continue
            if dc or dr:
                if (c, r, c2, r2, t) in econ:
                    continue
                # the other half of a swap is the same physical conflict
                if (c2, r2, c, r, t) in econ:
                    continue
            st2 = (c2, r2, t + 1)
            nc = cost + (MOVE_COST if (dc or dr) else MOVE_COST)
            if nc < g.get(st2, 1 << 30):
                g[st2] = nc
                came[st2] = (c, r, t)
                heapq.heappush(open_heap, (nc + h(c2, r2), nc, st2))

    if stats is not None:
        stats["expanded"] = stats.get("expanded", 0) + expanded
    if best_goal_state is None:
        return None
    path = [best_goal_state]
    while path[-1] != start_state:
        path.append(came[path[-1]])
    path.reverse()
    return [(c, r) for c, r, _t in path]


def _cheb(a, b):
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def _first_conflict(paths, min_sep=None):
    """First vertex or edge conflict between any two agents, or None.

    Paths shorter than the makespan are padded with their last cell: a robot
    that has arrived stays standing there, and that occupied pad has to be
    respected by whoever comes later.

    An edge conflict is reported as ("edge", i, j, t): the two agents swap
    cells between t and t+1. The caller derives each agent's own (from, to)
    pair from its own path - the two moves are different moves, and a
    constraint built from one agent's cells is meaningless for the other.

    min_sep[i][j] raises the bar from "not the same cell" to "at least this far
    apart, in cells". It is what makes the plan executable: the local stack
    inflates every teammate in the costmap, so a route that lets two robots
    brush past each other one cell apart is a route MPPI will refuse to drive.
    """
    makespan = max(len(p) for p in paths)
    padded = [p + [p[-1]] * (makespan - len(p)) for p in paths]
    for t in range(makespan):
        for i in range(len(paths)):
            for j in range(i + 1, len(paths)):
                need = min_sep[i][j] if min_sep else 1
                if _cheb(padded[i][t], padded[j][t]) < need:
                    return ("near", i, j, padded[j][t], t)
                if padded[i][t] == padded[j][t]:
                    return ("vertex", i, j, padded[i][t], t)
                if t + 1 < makespan and padded[i][t] == padded[j][t + 1] \
                        and padded[i][t + 1] == padded[j][t]:
                    return ("edge", i, j, None, t)
    return None


@dataclass(order=True)
class _Node:
    cost: int
    tie: int
    constraints: dict = field(compare=False)
    paths: list = field(compare=False)


def cbs(agents, time_limit=5.0, max_expansions=1500, horizon_slack=1.6,
        min_horizon=40, min_sep=None, verbose=False):
    """Optimal (budgeted) CBS.

    agents: list of dicts {mask, start, goal}. Returns (paths, info) where
    paths is a list of cell sequences (one per agent) or None, and info
    carries a few counters for the logs.

    min_sep: optional n x n matrix of the minimum Chebyshev distance (in
    cells) two agents must keep. See _first_conflict.
    """
    t0 = time.time()
    info = {"expansions": 0, "conflicts": 0, "fallback": False, "seconds": 0.0,
            "makespan": 0}
    n = len(agents)
    if n == 0:
        return [], info
    if n == 1:
        a = agents[0]
        p = _spatial_path(a["grid"], a["start"], a["goal"], a["mask"])
        info["seconds"] = time.time() - t0
        # A single robot can still be stuck (its own parked teammates may have
        # blocked every route), and the caller has to hear about that.
        return ([p] if p is not None else None), info

    # horizon: enough room for the longest independent path plus waiting
    indep = []
    horizon = min_horizon
    for a in agents:
        p = _spatial_path(a["grid"], a["start"], a["goal"], a["mask"])
        if p is None:
            info["fallback"] = True
            info["seconds"] = time.time() - t0
            return None, info
        indep.append(p)
        horizon = max(horizon, int(horizon_slack * (len(p)-1)) + min_horizon)

    root = _Node(cost=sum(max(1, len(p) - 1) for p in indep), tie=0,
                 constraints=[({}, {}) for _ in range(n)], paths=indep)
    open_list = [root]
    tie = 1
    best = None

    while open_list:
        if time.time() - t0 > time_limit or info["expansions"] > max_expansions:
            info["budget_hit"] = True
            break
        node = heapq.heappop(open_list)
        bypass_start = info.get("bypass", 0)
        while True:
            # The bypass descent below is still a search: it has to obey the
            # same budget, or an infeasible instance (two robots that need the
            # one cell they both already stand on) keeps generating same-cost
            # children forever instead of reporting "no solution".
            if time.time() - t0 > time_limit or info["expansions"] > max_expansions:
                info["budget_hit"] = True
                break
            info["expansions"] += 1
            conflict = _first_conflict(node.paths, min_sep)
            if conflict is None:
                best = node
                break
            info["conflicts"] += 1
            children, tie = _branch(node, conflict, agents, horizon, tie,
                                    min_sep)
            if not children:
                break
            # BYPASS: a child that costs the same as its parent is still
            # optimal, so descend into it directly instead of growing the open
            # list. On this map that is what turns a 40 m corridor head-on from
            # "thousands of queued nodes" into a handful of descents.
            cheap = [c for c in children if c.cost == node.cost]
            if cheap:
                node = cheap[0]
                info["bypass"] = info.get("bypass", 0) + 1
                # A bypass chain is monotone (same cost, strictly more
                # constraints) so it cannot cycle - unless a constraint failed
                # to bite, which is exactly the bug this guard caught once.
                # Bail out to the open list rather than trusting it blindly.
                if info["bypass"] - bypass_start > 200:
                    info["bypass_cap_hit"] = True
                    for c in children:
                        heapq.heappush(open_list, c)
                    break
                continue
            for c in children:
                heapq.heappush(open_list, c)
            break
        if best is not None:
            break
        if info.get("budget_hit"):
            break

    if best is None:
        info["seconds"] = time.time() - t0
        return None, info
    info["makespan"] = max(len(p) for p in best.paths)
    info["seconds"] = time.time() - t0
    return best.paths, info


def _branch(node, conflict, agents, horizon, tie, min_sep=None):
    """Children of a CBS node: one per agent involved in the conflict."""
    kind, i, j, cell, t = conflict
    children = []
    padded = [p + [p[-1]] * (max(len(q) for q in node.paths) - len(p))
              for p in node.paths]
    for who in (i, j):
        cons = [({k: set(v) for k, v in c[0].items()},
                 {k: set(v) for k, v in c[1].items()})
                for c in node.constraints]
        vcon, econ = cons[who]
        if kind == "near":
            # keep `who` at least min_sep cells away from the OTHER agent's
            # cell at this time step: block the square of that half-width
            other = padded[j if who == i else i][t]
            need = (min_sep[i][j] if min_sep else 1) - 1
            for dr in range(-need, need + 1):
                for dc in range(-need, need + 1):
                    vcon.setdefault(
                        ("v", (other[0] + dc, other[1] + dr)), set()).add(t)
        elif kind == "vertex":
            vcon.setdefault(("v", (cell[0], cell[1])), set()).add(t)
        else:
            # this agent's OWN move at time t - the swap is forbidden by
            # forbidding each half of it separately
            frm, to = padded[who][t], padded[who][t + 1]
            econ.setdefault(("e", (frm[0], frm[1], to[0], to[1])),
                            set()).add(t)
        a = agents[who]
        path = _astar(a["grid"], a["start"], a["goal"], a["mask"],
                      _flat_vertex(vcon), _flat_edge(econ), horizon=horizon,
                      earliest_goal=_earliest_goal(vcon, a["goal"]))
        if path is None:
            continue
        paths = list(node.paths)
        paths[who] = path
        tie += 1
        children.append(_Node(cost=sum(max(1, len(p) - 1) for p in paths),
                              tie=tie, constraints=cons, paths=paths))
    return children, tie


def _manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _flat_vertex(vcon):
    out = set()
    for (_kind, cell), times in vcon.items():
        for t in times:
            out.add((cell[0], cell[1], t))
    return out


def _flat_edge(econ):
    out = set()
    for (_kind, key), times in econ.items():
        for t in times:
            out.add((key[0], key[1], key[2], key[3], t))
    return out


def _earliest_goal(vcon, goal):
    times = vcon.get(("v", (goal[0], goal[1])), set())
    return (max(times) + 1) if times else 0


def prioritized(agents, horizon_slack=1.6, min_horizon=40, min_sep=None,
                time_limit=2.0):
    deadline = time.time() + time_limit
    """Fallback: plan one robot at a time around the routes already planned.

    Order matters and is chosen deliberately: the robots that cannot move
    (parked on a charger, start == goal) are planned FIRST, because they are
    immovable obstacles that everyone else has to route around; then the movers
    with the longest routes. A route that has already arrived keeps occupying
    its last cell for the whole horizon - that is what stops a later robot from
    planning a path straight through somebody's parking bay.
    """
    order = sorted(range(len(agents)),
                   key=lambda i: (not agents[i].get("moving", True), 0))
    # movers first by decreasing straight-line distance, parked ones in front
    order = sorted(range(len(agents)),
                   key=lambda i: (0 if not agents[i].get("moving", True)
                                  else 1,
                                  -_manhattan(agents[i]["start"], agents[i]["goal"])))
    independent = [_spatial_path(a["grid"], a["start"], a["goal"], a["mask"])
                   for a in agents]
    if any(path is None for path in independent):
        return independent
    horizon = max([min_horizon] + [int(horizon_slack*(len(path)-1))+min_horizon
                                   for path in independent])
    paths = []
    placed = {}
    n = len(agents)
    if min_sep is None:
        min_sep = [[1] * n for _ in range(n)]

    def replan(i, others):
        """A* for agent i around every path in `others` (index -> path)."""
        a = agents[i]
        vcon, econ = set(), set()
        for k, p in others.items():
            q = p + [p[-1]] * (horizon - len(p))
            need = max(0, min_sep[i][k] - 1)
            for t, (c, r) in enumerate(q):
                for dr in range(-need, need + 1):
                    for dc in range(-need, need + 1):
                        vcon.add((c + dc, r + dr, t))
            for t in range(len(q) - 1):
                econ.add((q[t][0], q[t][1], q[t + 1][0], q[t + 1][1], t))
        return _astar(a["grid"], a["start"], a["goal"], a["mask"], vcon, econ,
                      horizon=horizon)

    for i in order:
        placed[i] = replan(i, {k: v for k, v in placed.items()})
        if placed[i] is None:
            placed[i] = [agents[i]["start"]]

    # One greedy pass can leave a conflict behind: an agent whose A* found
    # nothing falls back to "stand still", and whoever was planned before it
    # knew nothing about that. A few Gauss-Seidel rounds fix most of it - each
    # agent is re-planned around everyone else's current route - and stop as
    # soon as the joint plan is clean.
    for _round in range(5):
        if time.time() > deadline:
            break
        if _first_conflict([placed[i] for i in range(n)], min_sep) is None:
            break
        for i in order:
            if time.time() > deadline:
                break
            others = {k: v for k, v in placed.items() if k != i}
            p = replan(i, others)
            if p is not None:
                placed[i] = p
    return [placed[i] for i in range(n)]


def simplify(path, grid):
    """Drop the redundant cells of a 4-connected route (keeps the corners)."""
    if len(path) <= 2:
        return [grid.to_world(*p) for p in path]
    keep = [path[0]]
    for i in range(1, len(path) - 1):
        a, b, c = path[i - 1], path[i], path[i + 1]
        if (b[0] - a[0], b[1] - a[1]) != (c[0] - b[0], c[1] - b[1]):
            keep.append(b)
    keep.append(path[-1])
    return [grid.to_world(*p) for p in keep]


# ----------------------------------------------------------------------
# the fleet-facing entry point
# ----------------------------------------------------------------------
@dataclass
class Route:
    """One robot's conflict-free route, in world metres."""

    name: str
    waypoints: list          # [(x, y), ...] including the start
    cells: list              # [(c, r), ...] one per time step
    waited: int              # time steps spent standing still
    timed_waypoints: list = field(default_factory=list)

    @property
    def length(self) -> float:
        return sum(((self.waypoints[i + 1][0] - self.waypoints[i][0]) ** 2
                    + (self.waypoints[i + 1][1] - self.waypoints[i][1]) ** 2) ** 0.5
                   for i in range(len(self.waypoints) - 1))


class FleetPlanner:
    """CBS over the shipped occupancy map, for the whole fleet at once.

    Build it once from maps/geomap.yaml (.pgm); then call `plan()` with every
    robot - movers and parked ones alike - every time the fleet's set of active
    legs changes.
    """

    def __init__(self, map_yaml: str, cell: float = 0.20, time_limit: float = 5.0,
                 max_expansions: int = 1500, margin: float = 0.35,
                 sep_margin: float = 0.50, verbose: bool = False):
        import yaml
        from PIL import Image

        meta = yaml.safe_load(open(map_yaml))
        img = np.asarray(Image.open(map_yaml.replace(".yaml", ".pgm")).convert("L"))
        self.grid = MapfGrid(occupied=img < 128, res=float(meta["resolution"]),
                             origin=(float(meta["origin"][0]), float(meta["origin"][1])),
                             cell=cell)
        # Keep at least the costmap's own inflation between the plan and the
        # walls. The local costmap inflates obstacles by inflation_radius
        # (0.35 m in nav2_params_*.yaml); a route planned with less air than
        # that runs through cells the controller sees as expensive or lethal,
        # and the robot crawls, trips the collision monitor's slowdown polygon
        # and finally reports "failed to make progress" - which is exactly what
        # the first version of this module did.
        self.margin = margin
        # Extra air between two robots on top of their two radii. It exists
        # because the plan has to be drivable by the local stack, not just
        # conflict-free on paper: the costmap inflates every teammate, so a
        # route that passes another robot with only its body width to spare is
        # one MPPI will refuse to follow (measured: a 0.76 m centre distance
        # when the costmap demanded ~0.8 m, and the robot simply stopped). At
        # the 0.20 m MAPF cell, 0.35 m of margin rounds two light robots up to
        # 4 cells = 0.80 m apart - and 0.50 m rounds up to 5 cells = 1.00 m,
        # which is what the measured stall actually needed: at 0.80 m the local
        # costmap still read the gap between two robots as a cost-100 channel
        # (the collision monitor slowed the robot to 40% and the progress
        # checker then aborted the goal). A metre of air is comfortably outside
        # the 0.35 m inflation on both sides.
        self.sep_margin = sep_margin
        self.time_limit = time_limit
        self.max_expansions = max_expansions
        self.verbose = verbose

    def plan(self, fleet):
        """fleet: list of (name, (x, y) start, (x, y) goal, radius_m, is_moving).

        Returns (routes_by_name, info).

        A robot with is_moving False is NOT planned: it is baked into every
        mover's traversability mask as an obstacle, inflated by the sum of the
        two radii. That distinction matters. If a parked robot were planned as
        an agent it would happily be told to step aside and come back - which
        is optimal on paper and useless in practice, because nobody sends a
        parked robot a goal, so the mover would drive through the space the
        planner had reserved for a robot that never moved.
        """
        movers, parked = [], []
        for name, start_xy, goal_xy, radius, moving in fleet:
            if moving:
                movers.append((name, start_xy, goal_xy, radius))
            else:
                parked.append((start_xy, radius))
        if not movers:
            return {}, {"seconds": 0.0, "makespan": 0, "expansions": 0,
                        "idle": True}

        def sep_cells(r1, r2):
            """Minimum centre-to-centre distance for two robots, in cells."""
            return max(1, int(np.ceil(
                (r1 + r2 + self.sep_margin) / self.grid.cell - 1e-9)))

        agents, names, radii = [], [], []
        # (cell, radius) of every goal already claimed. Two robots whose pads
        # end up closer than the separation rule make the instance INFEASIBLE,
        # not merely hard: whoever arrives first stays there, so the second can
        # never be within `sep` cells of it at any time step, and CBS spends
        # its whole budget proving it (measured: 6 s and a fallback on the
        # two-robots-one-pad case). Nudging the second pad out to the full
        # separation keeps the problem solvable.
        taken_goals: list = []
        for name, start_xy, goal_xy, radius in movers:
            # inflation the robot's centre must keep from a wall: the larger of
            # its own half-size and the local costmap's inflation radius
            mask = self.grid.mask(max(radius, self.margin)).copy()
            # parked robots: immovable, so keep the same separation clear of
            # their centres in this robot's mask
            for (px, py), p_radius in parked:
                self._block_box(mask, self.grid.to_cell(px, py),
                                sep_cells(radius, p_radius) - 1)
            s = self.grid.nearest_free(*self.grid.to_cell(*start_xy), mask)
            g = self.grid.nearest_free(*self.grid.to_cell(*goal_xy), mask)
            # The schedule does hand the same pickup pad to two robots at once,
            # so the goal is nudged to the nearest free cell that still leaves
            # the full separation - the robot ends up standing on the marker,
            # just not on top of its colleague.
            if g is not None and any(
                    _cheb(g, gc) < sep_cells(radius, gr)
                    for gc, gr in taken_goals):
                g = self._nudge(g, mask, taken_goals,
                                lambda gr: sep_cells(radius, gr))
            if g is not None:
                taken_goals.append((g, radius))
            if s is None or g is None:
                return {}, {"error": f"{name}: no free cell near start/goal",
                            "seconds": 0.0}
            agents.append({"name": name, "mask": mask, "start": s, "goal": g,
                           "grid": self.grid, "moving": True})
            names.append(name)
            radii.append(radius)

        min_sep = [[sep_cells(radii[i], radii[j]) for j in range(len(radii))]
                   for i in range(len(radii))]
        # Planner sees the coarse grid through each agent dict.
        for a in agents:
            a.setdefault("grid", self.grid)

        paths, info = cbs(agents, time_limit=self.time_limit,
                          max_expansions=self.max_expansions,
                          min_sep=min_sep)
        if paths is None:
            paths = prioritized(agents, min_sep=min_sep,
                                time_limit=self.time_limit)
            info["fallback"] = True
        if any(p is None for p in paths):
            paths = prioritized(agents, min_sep=min_sep,
                                time_limit=self.time_limit)
            info["fallback"] = True

        if not paths or any(path is None for path in paths):
            return {}, dict(info, error="fallback could not plan every agent")
        if _first_conflict(paths, min_sep) is not None:
            return {}, dict(info, error="fallback retained conflicts")
        if any(not path or path[0] != agent["start"] or path[-1] != agent["goal"]
               for path, agent in zip(paths, agents)):
            return {}, dict(info, error="fallback did not reach every goal")
        routes = {}
        for name, path in zip(names, paths):
            wp = simplify(path, self.grid)
            waited = sum(1 for i in range(1, len(path)) if path[i] == path[i - 1])
            routes[name] = Route(name=name, waypoints=wp, cells=list(path),
                                 waited=waited,
                                 timed_waypoints=[self.grid.to_world(*cell) for cell in path])
        if self.verbose:
            print(f"CBS: {info}")
        return routes, info

    def _block_box(self, mask, cell, half):
        """Mark the (2*half+1)^2 square of cells around `cell` impassable.

        A square, not a disc, on purpose: the separation constraint used by the
        planner between two moving robots is Chebyshev, so blocking the same
        shape around a parked robot keeps the two rules identical.
        """
        c0, r0 = cell
        for dr in range(-half, half + 1):
            for dc in range(-half, half + 1):
                c, row = c0 + dc, r0 + dr
                if 0 <= c < self.grid.W and 0 <= row < self.grid.H:
                    mask[row, c] = False
        return cell

    def _nudge(self, cell, mask, taken, need_fn, max_cells: int = 12):
        """Nearest free cell to `cell` that keeps the separation from the rest."""
        c0, r0 = cell
        for k in range(0, max_cells + 1):
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    if max(abs(dr), abs(dc)) != k:
                        continue
                    c, r = c0 + dc, r0 + dr
                    if not (0 <= c < self.grid.W and 0 <= r < self.grid.H):
                        continue
                    if not mask[r, c]:
                        continue
                    if any(_cheb((c, r), gc) < need_fn(gr)
                           for gc, gr in taken):
                        continue
                    return (c, r)
        return cell
