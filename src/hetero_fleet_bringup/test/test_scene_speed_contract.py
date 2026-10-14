"""Cruise limits and collider occupancy must agree across the actual stack."""
from pathlib import Path
import numpy as np
from PIL import Image
import yaml
import xml.etree.ElementTree as ET
import pytest

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('kind,typ', [('light','light_diff_amr'),('heavy','heavy_diff_amr'),('omni','omni_mecanum_amr'),('g1','g1_humanoid')])
def test_speed_limits_match_scheduler_controller_and_smoother(kind,typ):
    profile=yaml.safe_load((ROOT/'src/hetero_fleet_description/config/robot_profiles.yaml').read_text())['robot_types'][typ]
    cfg=yaml.safe_load((ROOT/f'src/hetero_fleet_bringup/config/nav2_params_{kind}.yaml').read_text())
    speed=profile['max_speed']
    follow=cfg['controller_server']['ros__parameters']['FollowPath']
    local=cfg['local_costmap']['local_costmap']['ros__parameters']
    assert local['width']/2 >= speed*follow['time_steps']*follow['model_dt']+profile['footprint'][0]/2
    if kind=='omni':
        assert follow['motion_model']=='Omni'
    assert cfg['controller_server']['ros__parameters']['FollowPath']['vx_max']==speed
    assert cfg['velocity_smoother']['ros__parameters']['max_velocity'][0]==speed
    if kind!='g1':
        control=yaml.safe_load((ROOT/f'src/hetero_fleet_description/config/{typ}_controllers.yaml').read_text())
        node='mecanum_drive_controller' if kind=='omni' else 'diff_drive_controller'
        assert control[f'/<robot_name>/{node}']['ros__parameters']['linear']['x']['max_velocity']==speed


def test_optimized_collider_cover_does_not_occupy_any_free_map_cell():
    meta=yaml.safe_load((ROOT/'src/hetero_fleet_bringup/maps/geomap.yaml').read_text())
    occupied=np.asarray(Image.open(ROOT/'src/hetero_fleet_bringup/maps'/meta['image']))<128
    covered=np.zeros_like(occupied)
    ox,oy,_=meta['origin'];res=meta['resolution']
    model=ET.parse(ROOT/'src/hetero_fleet_gz/models/geomap_combined12_noroof/model.sdf')
    for collision in model.findall('.//collision'):
        x,y,*_=map(float,collision.findtext('pose').split())
        sx,sy,*_=map(float,collision.findtext('geometry/box/size').split())
        c0,c1=round((x-sx/2-ox)/res),round((x+sx/2-ox)/res)
        r0,r1=round(occupied.shape[0]-(y+sy/2-oy)/res),round(occupied.shape[0]-(y-sy/2-oy)/res)
        covered[r0:r1,c0:c1]=True
    assert not np.any(covered & ~occupied)
    # This model was generated with --solid-occupied, so no obstacle cells disappear either.
    assert np.array_equal(covered,occupied)
