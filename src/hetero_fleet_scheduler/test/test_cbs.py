"""Tests for the CBS fleet planner.

These run without ROS and without the scanned map: the grid is built directly
as a numpy array, so a case is a few lines and the assertions are about the
algorithm, not about this building. The cases are the shapes that made the
reactive layer fail in the simulation - a corridor head-on, a doorway two
robots need at once, a crossing, and two robots sent to the same pad.

Run with:
    cd src/hetero_fleet_scheduler && /usr/bin/python3 -m pytest test/test_cbs.py -v
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from hetero_fleet_scheduler.cbs import (  # noqa: E402
    MapfGrid,
    cbs,
    _first_conflict,
    prioritized,
)


def grid(rows, cell=1.0):
    """Build a MapfGrid from ASCII art: '#' wall, '.' free. Row 0 is the top."""
    occ = np.array([[c == "#" for c in row] for row in rows], dtype=bool)
    g = MapfGrid(occupied=occ, res=cell, origin=(0.0, 0.0), cell=cell)
    return g


def make_agent(g, name, start_xy, goal_xy):
    return {"name": name, "grid": g, "mask": g.mask(0.0),
            "start": start_xy, "goal": goal_xy, "moving": True}


OPEN = [".......",
        ".......",
        "......."]


def test_conflict_detection_sees_vertex_and_swap():
    # same cell at the same time
    assert _first_conflict([[(0, 0), (1, 0)], [(0, 0), (0, 0)]]) is not None
    # head-on swap of two adjacent cells
    swap = _first_conflict([[(0, 0), (1, 0)], [(1, 0), (0, 0)]])
    assert swap is not None and swap[0] == "edge"
    # two robots walking side by side never conflict
    assert _first_conflict([[(0, 0), (1, 0)], [(0, 1), (1, 1)]]) is None


def test_corridor_head_on_is_resolved():
    """The case the reactive layer cannot solve: two robots, one aisle.

    The aisle is two cells wide, so the conflict is resolved by one robot
    stepping aside rather than by both stopping and staring at each other.
    """
    g = grid([".....",
              "....."])
    a = make_agent(g, "a", (0, 0), (4, 0))
    b = make_agent(g, "b", (4, 0), (0, 0))
    paths, info = cbs([a, b], time_limit=5.0)
    assert paths is not None
    assert _first_conflict(paths) is None
    for path, agent in zip(paths, (a, b)):
        assert path[0] == agent["start"]
        assert path[-1] == agent["goal"]


def test_one_cell_wide_corridor_head_on_is_reported_infeasible():
    """No passing place and no way to swap: CBS must say so, quickly."""
    g = grid(["....."])
    a = make_agent(g, "a", (0, 0), (4, 0))
    b = make_agent(g, "b", (4, 0), (0, 0))
    paths, info = cbs([a, b], time_limit=5.0)
    assert paths is None


def test_doorway_is_used_in_turn_not_simultaneously():
    """A 1-cell door: both robots may pass, but never at the same time."""
    g = grid(["...#...",
              "...#...",
              "......."])
    # the door is (3, 2); the two robots are either side of the wall
    a = make_agent(g, "a", (1, 0), (5, 1))
    b = make_agent(g, "b", (5, 1), (1, 0))
    paths, _info = cbs([a, b], time_limit=5.0)
    assert paths is not None
    assert _first_conflict(paths) is None


def test_crossing_robots_do_not_collide_at_the_centre():
    g = grid(["..#.#..",
              "..#.#..",
              ".......",
              "..#.#..",
              "..#.#.."])
    a = make_agent(g, "a", (3, 0), (3, 4))
    b = make_agent(g, "b", (0, 2), (6, 2))
    paths, _info = cbs([a, b], time_limit=5.0)
    assert paths is not None
    assert _first_conflict(paths) is None


# ------------------------------------------------------- the fleet interface


def write_map(tmp_path, rows, res=0.20):
    """A tiny geomap.yaml/.pgm pair so FleetPlanner can be exercised for real."""
    import numpy as np
    from PIL import Image

    arr = np.array([[0 if ch == "#" else 255 for ch in row] for row in rows],
                   dtype=np.uint8)
    Image.fromarray(arr).save(tmp_path / "geomap.pgm")
    (tmp_path / "geomap.yaml").write_text(
        f"image: geomap.pgm\nresolution: {res}\norigin: [0.0, 0.0, 0.0]\n"
        f"negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n")
    return str(tmp_path / "geomap.yaml")


def test_parked_robot_stays_put_and_is_driven_around(tmp_path):
    """A robot on its charger is an obstacle, not an agent that steps aside.

    Letting CBS move it is optimal on paper and wrong in practice: nothing
    sends a parked robot a goal, so the mover would drive through the space the
    plan had reserved for a robot that never moved.
    """
    from hetero_fleet_scheduler.cbs import FleetPlanner

    # 12 rows so there is room to go around the parked robot: the separation
    # rule keeps 5 cells (1.0 m) of air, which blocks most of a shorter grid.
    rows = ["." * 14] * 12
    yaml_path = write_map(tmp_path, rows)
    p = FleetPlanner(yaml_path, cell=0.20, time_limit=5.0)
    # row 4 is world y = 1.6 - (4+0.5)*0.2 = 0.7 ; col c is x = (c+0.5)*0.2
    mover_start = (0.1, 1.5)      # row 6, col 0
    mover_goal = (2.5, 1.5)       # row 6, col 12
    parked = (1.3, 1.5)           # row 6, col 6
    routes, info = p.plan([
        ("mover", mover_start, mover_goal, 0.19, True),
        ("parked", parked, parked, 0.19, False),
    ])
    assert "parked" not in routes, "a parked robot must not be given a route"
    assert "mover" in routes
    cells = routes["mover"].cells
    assert cells[0] == p.grid.to_cell(*mover_start)
    assert cells[-1] == p.grid.to_cell(*mover_goal)
    # it must not pass within the combined radius of the parked robot
    pc = p.grid.to_cell(*parked)
    assert all(max(abs(c - pc[0]), abs(r - pc[1])) > 1 for c, r in cells), \
        "the mover drove through the parked robot"


def test_two_robots_sent_to_the_same_pad_are_nudged_apart(tmp_path):
    from hetero_fleet_scheduler.cbs import FleetPlanner

    rows = ["." * 14] * 8
    p = FleetPlanner(write_map(tmp_path, rows), cell=0.20, time_limit=5.0)
    goal = (2.5, 0.7)
    routes, info = p.plan([
        ("a", (0.1, 0.1), goal, 0.19, True),
        ("b", (2.5, 1.5), goal, 0.19, True),
    ])
    assert len(routes) == 2
    ga = p.grid.to_cell(*routes["a"].waypoints[-1])
    gb = p.grid.to_cell(*routes["b"].waypoints[-1])
    assert ga != gb, "both robots were parked on the same cell"


def test_no_movers_means_no_plan(tmp_path):
    from hetero_fleet_scheduler.cbs import FleetPlanner

    p = FleetPlanner(write_map(tmp_path, ["." * 8] * 4), cell=0.20)
    routes, info = p.plan([("a", (0.1, 0.1), (0.1, 0.1), 0.19, False)])
    assert routes == {}
    assert info.get("idle")


def test_infeasible_instance_reports_failure_instead_of_hanging():
    """No route exists: the planner must say so, not spin forever."""
    g = grid(["..#.."])
    a = make_agent(g, "a", (0, 0), (4, 0))
    paths, _info = cbs([a], time_limit=2.0)
    assert paths is None


def test_prioritized_fallback_is_conflict_free_and_complete():
    g = grid([".....",
              "....."])
    agents = [make_agent(g, "a", (0, 0), (4, 0)),
              make_agent(g, "b", (4, 0), (0, 0)),
              make_agent(g, "c", (2, 1), (2, 1))]
    agents[2]["moving"] = False
    paths = prioritized(agents)
    assert len(paths) == 3
    for path, agent in zip(paths, agents):
        assert path[0] == agent["start"]
    assert _first_conflict(paths) is None


def test_fleet_planner_rejects_prioritized_fallback_that_did_not_arrive(monkeypatch):
    import hetero_fleet_scheduler.cbs as module
    planner = module.FleetPlanner.__new__(module.FleetPlanner)
    planner.grid = grid(OPEN)
    planner.margin = 0.0
    planner.sep_margin = 0.0
    planner.time_limit = 0.1
    planner.max_expansions = 1
    planner.verbose = False
    monkeypatch.setattr(module, 'cbs', lambda *a, **k: (None, {'fallback': False}))
    monkeypatch.setattr(module, 'prioritized', lambda agents, **k: [[agent['start']] for agent in agents])
    start, goal = planner.grid.to_world(0, 1), planner.grid.to_world(6, 1)
    routes, info = planner.plan([('a', start, goal, 0.0, True)])
    assert routes == {}
    assert 'reach every goal' in info['error']


def test_fleet_planner_preserves_timed_wait_cells(monkeypatch):
    import hetero_fleet_scheduler.cbs as module
    planner = module.FleetPlanner.__new__(module.FleetPlanner)
    planner.grid = grid(OPEN)
    planner.margin = planner.sep_margin = 0.0
    planner.time_limit, planner.max_expansions, planner.verbose = .1, 1, False
    cells = [(0, 1), (0, 1), (1, 1), (2, 1)]
    monkeypatch.setattr(module, 'cbs', lambda *a, **k: ([cells], {'fallback': False}))
    start, goal = planner.grid.to_world(0, 1), planner.grid.to_world(2, 1)
    routes, _ = planner.plan([('a', start, goal, 0.0, True)])
    assert len(routes['a'].timed_waypoints) == 4
    assert routes['a'].timed_waypoints[0] == routes['a'].timed_waypoints[1]
    assert routes['a'].waited == 1


def test_single_agent_can_take_a_detour_longer_than_the_old_manhattan_horizon():
    g = grid(['.'*50, '#'*49+'.', '.'*50])
    a = make_agent(g, 'a', (0,0), (0,2))
    paths, _ = cbs([a], time_limit=1)
    assert paths is not None and len(paths[0]) == 101
    assert paths[0][-1] == (0,2)


def test_prioritized_horizon_uses_actual_detour_length():
    g = grid(['.'*50, '#'*49+'.', '.'*50])
    a = make_agent(g, 'a', (0,0), (0,2))
    paths = prioritized([a], time_limit=1)
    assert paths[0][-1] == (0,2)
