"""Synthetic teammates stay planar while G1's actual lidar follows body tilt."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import yaml
import pytest
from sensor_msgs.msg import LaserScan

ROOT = Path(__file__).resolve().parents[3]
path = Path(__file__).resolve().parents[1]/'scripts/scan_frame_relay.py'
spec = importlib.util.spec_from_file_location('scan_relay_g1_test', path)
relay = importlib.util.module_from_spec(spec)
spec.loader.exec_module(relay)


def test_g1_synthetic_frame_does_not_change_real_scan_frame():
    node = object.__new__(relay.ScanFrameRelay)
    node.max_range = 12.
    node.robots = {
        'g1_1': {'legged':True, 'pose':(0.,0.,0.), 'lidar_dx':0., 'half_length':.3, 'half_width':.26},
        'heavy_1': {'pose':(2.,0.,0.), 'lidar_dx':.12, 'half_length':.307413, 'half_width':.231674},
    }
    out = []
    node.obstacle_pubs = {'g1_1':SimpleNamespace(publish=out.append)}
    scan = LaserScan()
    scan.header.frame_id = 'g1_1/lidar_link'
    scan.range_max = 12.
    scan.angle_increment = .01
    scan.ranges = [float('inf')]
    node.publish_robot_obstacles(scan, 'g1_1')
    assert scan.header.frame_id == 'g1_1/lidar_link'
    assert out[0].header.frame_id == 'g1_1/base_footprint'
    assert out[0].ranges[0] == pytest.approx(2.-.307413)


def test_g1_navigation_uses_published_teammate_topics():
    cfg = yaml.safe_load((ROOT/'src/hetero_fleet_bringup/config/nav2_params_g1.yaml').read_text())
    for costmap in ['local_costmap','global_costmap']:
        params = cfg[costmap][costmap]['ros__parameters']
        assert params['robot_obstacle_layer']['robot_obstacles']['topic'] == '/<robot_name>/robot_obstacles'
    assert cfg['collision_monitor']['ros__parameters']['robot_obstacles']['topic'] == '/<robot_name>/robot_obstacles'
