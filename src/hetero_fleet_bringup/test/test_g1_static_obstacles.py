"""The high G1 lidar must not be the only source of wall collision constraints."""
from pathlib import Path
import yaml

ROOT=Path(__file__).resolve().parents[3]


def test_local_and_global_g1_maps_share_static_wall_constraints():
    cfg=yaml.safe_load((ROOT/'src/hetero_fleet_bringup/config/nav2_params_g1.yaml').read_text())
    profile=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())['robot_types']['g1_humanoid']
    for which in ['local_costmap','global_costmap']:
        params=cfg[which][which]['ros__parameters']
        assert 'static_layer' in params['plugins']
        assert params['plugins'].index('static_layer')<params['plugins'].index('inflation_layer')
        assert params['static_layer']['plugin']=='nav2_costmap_2d::StaticLayer'
        assert params['static_layer']['map_subscribe_transient_local']
        assert params['footprint_padding']==profile['footprint_padding']
        assert params['inflation_layer']['inflation_radius']==profile['inflation_radius']
    local=cfg['local_costmap']['local_costmap']['ros__parameters']
    assert not local['static_layer']['footprint_clearing_enabled']
    # Lidar and teammate obstacles remain additional constraints, never replacements for the map.
    assert 'obstacle_layer' in local['plugins'] and 'robot_obstacle_layer' in local['plugins']
