"""G1 task eligibility and balance health must remain honest."""
from pathlib import Path
from types import SimpleNamespace
import yaml
from hetero_fleet_scheduler.fleet_manager import FleetManager
from hetero_fleet_scheduler.task_model import RobotState, Pose2D, Task
from hetero_fleet_scheduler.cost import evaluate, CostWeights

ROOT = Path(__file__).resolve().parents[3]


def test_balance_readiness_expires_and_rejects_clock_reset():
    m = FleetManager.__new__(FleetManager)
    m.locomotion_health = {'g1_1': (True, 10.)}
    assert m._locomotion_available('light_1', 100.)
    assert m._locomotion_available('g1_1', 10.5)
    assert not m._locomotion_available('g1_1', 11.1)
    assert not m._locomotion_available('g1_1', 0.)
    m.locomotion_health['g1_1'] = (False, 10.)
    assert not m._locomotion_available('g1_1', 10.1)
    m._now = lambda: 10.1
    # Reject before touching the navigation client on an unsafe robot.
    assert m._send_goal(SimpleNamespace(id='g1_1'), Pose2D(0,0,0), 'test') == (None, 'locomotion_unavailable')


def test_g1_scenarios_are_only_eligible_for_g1_and_within_payload():
    profile = yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())
    for scenario, expected in [('demo_10', 2), ('mixed_20', 3)]:
        tasks = yaml.safe_load((ROOT/f'src/hetero_fleet_scheduler/config/tasks/{scenario}.yaml').read_text())['tasks']
        jobs = [t for t in tasks if t['required_capability'] == 'humanoid_transport']
        assert len(jobs) == expected
        for spec in jobs:
            assert spec['payload_kg'] <= 5
            assert spec['pickup'][1] < -21 and spec['dropoff'][1] < -21
            t = Task(id=spec['id'], pickup=Pose2D(*spec['pickup']), dropoff=Pose2D(*spec['dropoff']),
                     payload_kg=spec['payload_kg'], required_capability=spec['required_capability'])
            for entry in profile['fleet']:
                kind = profile['robot_types'][entry['type']]
                robot = RobotState(id=entry['name'], type=entry['type'], pose=Pose2D(*entry['spawn']),
                                   payload_capacity_kg=kind['payload_kg'], capabilities=kind['capabilities'])
                result = evaluate(robot, t, 0., CostWeights(), 15.)
                assert result.feasible == (entry['type'] == 'g1_humanoid')
