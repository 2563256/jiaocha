#!/usr/bin/env python3
"""Smooth shallow wall teeth/notches while preserving original aisle ridges.

Input stays immutable. Regenerate collision boxes after writing the map.
Only small boundary edits are permitted; isolated furniture and thin long
walls are retained. Additions cannot decrease clearance at original medial
ridge samples, or disconnect the originally connected traversable cores.
"""
import argparse
from pathlib import Path
import numpy as np
from PIL import Image
from scipy import ndimage as ndi


def ridge_mask(distance, minimum):
    ridge=np.zeros(distance.shape,bool)
    for dr,dc in [(1,0),(0,1),(1,1),(1,-1)]:
        a=ndi.shift(distance,(dr,dc),order=0,mode='constant',cval=0)
        b=ndi.shift(distance,(-dr,-dc),order=0,mode='constant',cval=0)
        ridge|=(distance>=a)&(distance>=b)&((distance>a)|(distance>b))
    return ridge&(distance>=minimum)


def small_blobs(mask,max_cells):
    labels,n=ndi.label(mask,np.ones((3,3)))
    sizes=np.bincount(labels.ravel())
    valid=(sizes<=max_cells);valid[0]=False
    return valid[labels]


def smooth(occupied,res=.05,radius=.10,max_area=.15):
    r=max(1,int(round(radius/res)));yy,xx=np.mgrid[-r:r+1,-r:r+1];disk=xx*xx+yy*yy<=r*r
    before=ndi.distance_transform_edt(~occupied)
    labels,n=ndi.label(occupied,np.ones((3,3)));areas=np.bincount(labels.ravel())*res*res
    removed=occupied&~ndi.binary_opening(occupied,structure=disk,border_value=1)
    removed=small_blobs(removed,int(max_area/res**2))&(areas[labels]>=1.)
    removed&=ndi.distance_transform_edt(occupied)<=r+1
    removed[:r]=False;removed[-r:]=False;removed[:,:r]=False;removed[:,-r:]=False
    opened=occupied&~removed
    additions=ndi.binary_closing(opened,structure=disk,border_value=1)&~occupied
    additions=small_blobs(additions,int(max_area/res**2))
    additions&=before<=r+1
    ridges=ridge_mask(before,minimum=.20/res)
    # Remove each new cell responsible for narrowing an original ridge. Repeat
    # because another equally close new cell may become the nearest obstacle.
    while additions.any():
        after,nearest=ndi.distance_transform_edt(~(opened|additions),return_indices=True)
        bad=ridges&(after+1e-6<before)
        if not bad.any():break
        rr,cc=nearest[:,bad]
        offenders=additions[rr,cc]
        if not offenders.any():raise RuntimeError('opening unexpectedly narrowed free space')
        additions[rr[offenders],cc[offenders]]=False
    result=opened|additions
    after=ndi.distance_transform_edt(~result)
    assert np.all(after[ridges]+1e-6>=before[ridges])
    for clearance in (.20,.30,.50,.55,.70):
        original,n=ndi.label(before*res>=clearance)
        new,_=ndi.label(after*res>=clearance)
        for component,sl in enumerate(ndi.find_objects(original),1):
            oldcore=original[sl]==component
            survivors=new[sl][oldcore]
            if (survivors==0).any() or len(np.unique(survivors))!=1:
                # Very conservative fallback: trim recess filling, retain only
                # opening, which can never narrow or disconnect a channel.
                result=opened;additions[:]=False;break
        if not additions.any():break
    return result,dict(removed_cells=int(removed.sum()),filled_cells=int(additions.sum()),
                       protected_ridge_samples=int(ridges.sum()),radius_m=radius)


def main():
    root=Path(__file__).resolve().parents[1]
    ap=argparse.ArgumentParser();ap.add_argument('--input',type=Path,default=root/'tools/data/geomap_before_wall_smoothing.pgm');ap.add_argument('--output',type=Path,default=root/'src/hetero_fleet_bringup/maps/geomap.pgm');a=ap.parse_args()
    image=np.array(Image.open(a.input));assert set(np.unique(image))<={0,255},'binary map required'
    result,report=smooth(image<128)
    Image.fromarray(np.where(result,0,255).astype('uint8')).save(a.output)
    print(report)

if __name__=='__main__':main()
