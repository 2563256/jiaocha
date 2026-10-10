"""Geometry regressions: smoothing must not narrow doors or erase furniture."""
from pathlib import Path
import importlib.util
import numpy as np
from scipy.ndimage import distance_transform_edt,label
ROOT=Path(__file__).resolve().parents[3]
spec=importlib.util.spec_from_file_location('wall_smoothing',ROOT/'tools/smooth_wall_map.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_wall_teeth_and_notches_are_smoothed_without_narrowing_aisle():
    wall=np.ones((80,100),bool);wall[25:55,5:95]=False
    wall[24:25,30:32]=False # shallow notch
    wall[25:27,60:62]=True # tooth inside the aisle
    result,report=module.smooth(wall,radius=.15)
    assert report['removed_cells']>0
    before=distance_transform_edt(~wall);after=distance_transform_edt(~result)
    ridges=module.ridge_mask(before,4.)
    assert np.all(after[ridges]>=before[ridges])
    assert np.count_nonzero(result[25:27,60:62])<=2  # reduced to at most one 5 cm row


def test_thin_wall_and_small_furniture_survive():
    wall=np.zeros((100,100),bool);wall[:5]=True;wall[-5:]=True;wall[:,:5]=True;wall[:,-5:]=True
    wall[20:80,45:47]=True;wall[20:24,70:74]=True
    result,_=module.smooth(wall)
    assert result[20:80,45:47].all()
    assert result[20:24,70:74].all()


def test_all_original_real_map_channels_keep_clearance_and_connectivity():
    from PIL import Image
    before=np.array(Image.open(ROOT/'tools/data/geomap_before_wall_smoothing.pgm'))<128
    after=np.array(Image.open(ROOT/'src/hetero_fleet_bringup/maps/geomap.pgm'))<128
    db=distance_transform_edt(~before)*.05;da=distance_transform_edt(~after)*.05
    ridges=module.ridge_mask(db,.20)
    assert np.all(da[ridges]+1e-6>=db[ridges])
    for gap in (.20,.30,.50,.55,.70):
        old,n=label(db>=gap);new,_=label(da>=gap)
        for i,sl in enumerate(__import__('scipy').ndimage.find_objects(old),1):
            values=new[sl][old[sl]==i]
            assert values.min()>0 and len(np.unique(values))==1
