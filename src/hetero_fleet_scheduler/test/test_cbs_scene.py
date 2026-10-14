"""Every enlarged robot must leave its bay with all three teammates parked."""
from pathlib import Path
import math
import pytest
import yaml
from hetero_fleet_scheduler.cbs import FleetPlanner

ROOT=Path(__file__).resolve().parents[3]
TASKS=yaml.safe_load((ROOT/'src/hetero_fleet_scheduler/config/tasks/mixed_20.yaml').read_text())['tasks']
NORTH={(5.,-17.5),(12.,-12.5),(10.,-21.)}
GOALS=list(dict.fromkeys(tuple(t[k][:2]) for t in TASKS for k in ('pickup','dropoff')))

@pytest.fixture(scope='module')
def scene():
    profile=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    planner=FleetPlanner(str(ROOT/'src/hetero_fleet_bringup/maps/geomap.yaml'),margin=.30,time_limit=3)
    return profile,planner

@pytest.mark.parametrize('who',['light_1','light_2','heavy_1','omni_1','g1_1'])
@pytest.mark.parametrize('goal',GOALS)
def test_each_robot_can_reach_each_zone_while_teammates_remain_parked(scene,who,goal):
    if goal in NORTH and who not in ('light_1','light_2'):
        pytest.skip('North-room sites are restricted to light vehicles')
    p,planner=scene
    fleet=[(r['name'],r['spawn'][:2],goal,
            .5*math.hypot(*p['robot_types'][r['type']]['footprint'])+.06,r['name']==who)
           for r in p['fleet']]
    routes,info=planner.plan(fleet)
    assert who in routes,info
    start=next(r['spawn'][:2] for r in p['fleet'] if r['name']==who)
    assert math.dist(routes[who].timed_waypoints[0],start)<=planner.grid.cell
    assert math.dist(routes[who].timed_waypoints[-1],goal)<=planner.grid.cell


def test_twenty_tasks_have_distinct_indoor_pairs():
    from collections import Counter
    import runpy
    zones=runpy.run_path(str(ROOT/'src/hetero_fleet_gz/worlds/make_geomap_world.py'))['ZONES']
    picks={tuple(z[1:3]) for z in zones if z[0].startswith('pickup_')}
    drops={tuple(z[1:3]) for z in zones if z[0].startswith('dropoff_')}
    pairs=[(tuple(t['pickup'][:2]),tuple(t['dropoff'][:2])) for t in TASKS]
    assert len(picks)==7 and len(drops)==6
    assert len(pairs)==len(set(pairs))==20
    assert set(p for p,d in pairs)==picks and set(d for p,d in pairs)==drops
    for t in TASKS:
        if tuple(t['pickup'][:2]) in NORTH or tuple(t['dropoff'][:2]) in NORTH:
            assert t['required_capability']=='light_payload' and t['payload_kg']<=20
    # Moved pads are inside the eastern rooms, clear of the long western corridor.
    assert all(x>=2.9 for x,y in picks|drops)
    profile=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    chargers=[z for z in zones if z[0].startswith('charger_')]
    for robot,charger in zip(profile['fleet'],chargers):
        assert robot['spawn']==robot['charger']
        assert tuple(robot['charger'][:2])==tuple(charger[1:3])


def test_northern_tasks_reject_heavy_and_omni_in_actual_cost_filter():
    from hetero_fleet_scheduler.task_model import RobotState,Task,Pose2D
    from hetero_fleet_scheduler.cost import evaluate,CostWeights
    profile=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    affected=[t for t in TASKS if tuple(t['pickup'][:2]) in NORTH or tuple(t['dropoff'][:2]) in NORTH]
    assert len(affected)==6
    for t in affected:
        task=Task(id=t['id'],pickup=Pose2D(*t['pickup']),dropoff=Pose2D(*t['dropoff']),payload_kg=t['payload_kg'],required_capability=t['required_capability'])
        for entry in profile['fleet']:
            rt=profile['robot_types'][entry['type']]
            robot=RobotState(id=entry['name'],type=entry['type'],payload_capacity_kg=rt['payload_kg'],capabilities=rt['capabilities'])
            result=evaluate(robot,task,0.,CostWeights(),15.)
            assert result.feasible==(entry['name'] in ('light_1','light_2'))
