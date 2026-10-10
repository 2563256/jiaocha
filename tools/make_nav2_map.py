#!/usr/bin/env python3
"""Rasterise the levelled scan mesh into a Nav2 occupancy map, and report on it.

POLARITY follows map_server's `negate: 0` convention, exactly like the
warehouse map this replaces: white (255) = drivable floor, black (0) =
obstacle. Getting this backwards makes every cell occupied, the costmap
inflates the whole world and Nav2 answers `START_OCCUPIED` to every goal - see
the note at the top of hetero_fleet_bringup/scripts/make_map.py.

What the map says:
  occupied = walls/furniture (the collision mesh, projected top-down)
             | everything the scan never saw
  free     = scanned floor that is not covered by structure

Everything outside the scanned footprint is marked occupied on purpose. The
scan is an island inside the world's 300 x 300 m ground plane, and a costmap
that reads "free" out there would happily plan a route off the edge of the
building.

With --report it prints how much of the free space is at least N metres wide
and ranks the roomiest spots - that is where the zone markers and the robot
spawn poses in robot_profiles.yaml come from.
"""

import argparse
import math
from pathlib import Path

import numpy as np
from PIL import Image

import map_common as mc


def build_map(visual_obj, collision_obj, res=0.05, margin=0.6, z_limit=None,
              cover_close=0.5, structure_close=0.1, smooth=1,
              structure_dilate=0, min_width=0.60, fill_pits=0.15,
              fill_pockets=0.50, fill_unreachable=0.0, fill_notches=0.0,
              edits=None, structure_band=None, free_floating=False,
              rectangularize=0.0, squash_floor=None, fill_outlines=False,
              squash_ceil=None,
              verbose=True):
    """Return (occupied_grid, meta) for a scan mesh pair.

    The collision mesh is expected to contain *only* what the robots cannot
    drive through: prepare_map_mesh.py already dropped the floor and every
    piece of clutter that is entirely below head height. So by default every
    face of it is projected onto the grid (z_limit=None).

    z_limit exists for experiments: if you pass a height, only triangles
    reaching below it are treated as obstacles. It is NOT what produces the
    ring corridor - that is decided when the collision mesh is built, so that
    the map and the physics keep describing the same room.

    cover_close closes the gaps in the scanned floor before they are treated
    as "unscanned = occupied". The scan simply never saw some floor cells
    (occlusion, dark corners); without this the aisle is cut into a dozen
    rooms by holes that are not physically there.

    smooth is a majority-vote (median) filter of that many cells on the final
    occupancy mask. Projecting a triangle mesh onto a 5 cm grid leaves the
    wall/free boundary as a staircase with single-cell teeth, and those teeth
    are what the costmap inflates and what make_collision_boxes.py turns into
    colliders - so the wall the robots see is jagged. One pass (smooth=1) is a
    3x3 vote: it removes 1-cell spikes and fills 1-cell notches, i.e. it
    smooths the outline, and moves the boundary by at most one cell (5 cm).
    Keep the value low: a large kernel starts eating the narrow passages.

    structure_dilate grows every wall by that many cells after rasterisation.
    It is 0 by default because it is pure lost drivable area: `structure_close`
    already makes thin walls solid, and the extra ring only made the map
    stricter than the room. Measured against the simulated lidar, the map was
    predicting obstacles a median 0.35 m closer than the scan actually saw
    them - i.e. the walls were drawn fatter than they are. Removing one cell of
    dilation gives ~13% more usable floor (largest connected region at the
    fleet's clearance requirement: 50.1 -> 56.4 m^2 on this map).

    fill_pits / fill_pockets (m^2) are the anti-speckle pass, and they only
    touch blobs that are completely enclosed:

      * `fill_pits` fills obstacles smaller than this that sit *inside*
        drivable floor with free space all the way around them - the single
        cells a noisy scan leaves in the middle of a room, which the costmap
        then inflates into a 0.7 m blob and which read as pits in RViz. An
        island bigger than this (a chair, a pillar) is kept: it is real.
        Keep the number SMALL: furniture the scan captured in fragments (the
        workbench at x 5..10, y -31..-32) also arrives as small islands, and
        filling those deletes the bench.
      * `fill_pockets` fills free pockets smaller than this that are enclosed
        by obstacles - pinpricks a few cells across, too small for any robot
        to enter and only visible as white dots in the wall.
      * `fill_unreachable` fills free pockets smaller than this that are NOT
        connected to the largest drivable region, whatever their shape: the
        white blobs a fragmented scan leaves scattered through the walls. They
        are real floor, but no robot can get to them, and in RViz they read as
        blank circles that hide the room behind. 0 (default) keeps them.

    `fill_notches` (metres) is a morphological closing of the obstacle mask:
    every concavity narrower than this is filled, so a furniture block that the
    scan left with a bite out of it becomes the solid rectangle it looks like
    in the room. It cannot change convex shapes, and it can only seal free gaps
    narrower than the value - keep it well under the fleet's ~1.1 m doorway
    budget (0.5 m is safe here; 0 disables it).

    `edits` is a dict of manual corrections, applied after the automatic
    passes and before the unreachable-pocket fill:

        {"erase": [(x0, y0, x1, y1), ...],   # force those cells to free
         "fill":  [(x0, y0, x1, y1), ...]}   # force those cells to obstacle

    It exists because a scan is not a CAD model: sometimes a real object is
    captured as a floating top surface (removing it is correct - the robot
    drives under it) and sometimes a doorway is recorded as solid. Every one of
    these boxes is listed in maps/map_edits.yaml with the reason, so the map
    stays reproducible from the scripts alone.

    Both run LAST, after the median filter: the 3x3 vote that smooths the wall
    outline also re-creates a few single-cell specks at corners, so cleaning
    before it leaves them behind. Both act on the same grid the colliders are
    built from, so physics and costmap stay identical.
    """
    from scipy import ndimage

    Vv, Fv = mc.load_obj(visual_obj)
    Vc, Fc = mc.load_obj(collision_obj)

    xmin = min(Vv[:, 0].min(), Vc[:, 0].min()) - margin
    xmax = max(Vv[:, 0].max(), Vc[:, 0].max()) + margin
    ymin = min(Vv[:, 1].min(), Vc[:, 1].min()) - margin
    ymax = max(Vv[:, 1].max(), Vc[:, 1].max()) + margin
    W = int(math.ceil((xmax - xmin) / res))
    H = int(math.ceil((ymax - ymin) / res))

    k = max(1, int(round(cover_close / res)) | 1)
    cover = mc.rasterise_footprint(Vv, Fv, res, xmin, ymin, (H, W))
    cover = ndimage.binary_fill_holes(
        ndimage.binary_closing(cover, structure=np.ones((k, k))))

    if z_limit is None:
        faces = Fc
    else:
        zmin = Vc[Fc][:, :, 2].min(axis=1)
        faces = Fc[zmin < z_limit]
    structure = mc.rasterise_footprint(Vc, faces, res, xmin, ymin, (H, W))
    if squash_floor is not None:
        # "PRESS THE GAZEBO MAP FLAT": every face of the *visual* mesh that
        # stands above the floor is projected, whatever height it lives at -
        # walls, furniture tops, lintels, clutter. That is what makes the map
        # look like the Gazebo view from above instead of a rubble of the few
        # surfaces the collision mesh happens to keep. The floor itself is
        # excluded (near-horizontal faces at floor level), or it would cover
        # the whole building. The passages are opened afterwards by the
        # floating-geometry test, not by filtering heights here.
        tri = Vv[Fv]
        fzmin = tri[:, :, 2].min(axis=1)
        fzmax = tri[:, :, 2].max(axis=1)
        _area, fnz, _cen = mc.face_geometry(Vv, Fv)
        floorish = (np.abs(fnz) > 0.7) & (fzmax < 0.20)
        sel = (fzmax > squash_floor) & ~floorish
        if squash_ceil is not None:
            # ...but the projection still cannot tell a wall from a door
            # lintel, and a lintel hanging over a doorway projects onto the
            # floor exactly like a wall, sealing the opening it hangs over.
            # Faces that live ENTIRELY above `squash_ceil` (beams, transoms,
            # ducts, signs) therefore hang over the robots and must not be
            # pressed flat - same rule as prepare_map_mesh.py's --drop-above,
            # applied here to the visual mesh instead of the collision mesh.
            sel &= fzmin < squash_ceil
        structure = mc.rasterise_footprint(Vv, Fv[sel], res, xmin, ymin, (H, W))
    if structure_band is not None:
        # THE ROBOT-HEIGHT MAP. Everything above is a *top-down* projection of
        # the collision mesh, and that projection cannot tell "a wall" from
        # "the top of a desk whose legs the scan never saw": the scan of this
        # building records whole furniture clusters as a floating surface at
        # 0.5-1.5 m, and projecting those seals every passage under them (the
        # map was 637 m^2 of obstacle in the building, 1.8x more than what the
        # fleet can actually hit).
        #
        # With structure_band=(lo, hi) the obstacles come from the *visual*
        # mesh faces that intersect the robot's own vertical extent - the same
        # geometry the 2D lidars at 0.26-0.36 m scan - so map, physics and
        # lidar all describe the same world: walls and furniture legs block,
        # table tops you can drive under do not. Floor faces (near-horizontal,
        # at floor level) are excluded so the floor does not block itself.
        lo, hi = structure_band
        tri = Vv[Fv]
        fzmin = tri[:, :, 2].min(axis=1)
        fzmax = tri[:, :, 2].max(axis=1)
        _area, fnz, _cen = mc.face_geometry(Vv, Fv)
        floorish = (np.abs(fnz) > 0.7) & (fzmax < 0.20)
        band = (fzmin <= hi) & (fzmax >= lo) & ~floorish
        structure = mc.rasterise_footprint(Vv, Fv[band], res, xmin, ymin, (H, W))
    structure_low = None
    if free_floating:
        # Obstacles that exist ONLY above the robots. The projection cannot
        # tell them from a wall, and on this scan whole furniture clusters are
        # recorded as a floating surface at 0.5-1.5 m, so those projections
        # seal every passage under them. The workbench the user wants to keep
        # is floating too - so instead of dropping all of it, a floating blob
        # is only dropped when it actually SEPARATES drivable space (same
        # passage test as the notch guard). Furniture standing in the middle
        # of a room stays on the map.
        tri = Vv[Fv]
        fzmin = tri[:, :, 2].min(axis=1)
        fzmax = tri[:, :, 2].max(axis=1)
        _area, fnz, _cen = mc.face_geometry(Vv, Fv)
        floorish = (np.abs(fnz) > 0.7) & (fzmax < 0.20)
        band = (fzmin <= 0.45) & (fzmax >= 0.05) & ~floorish
        structure_low = mc.rasterise_footprint(Vv, Fv[band], res, xmin, ymin, (H, W))
    # A wall one voxel thick projects to two parallel lines 10 cm apart; close
    # the gap so the wall is solid instead of a 2-cell free slot.
    ks = max(1, int(round(structure_close / res)) | 1)
    structure = ndimage.binary_closing(structure, structure=np.ones((ks, ks)))
    if fill_outlines:
        # The scan records most furniture as a *surface shell*: a desk arrives
        # as its outline, not as a filled block. Projected top-down that looks
        # like a scribble, while Gazebo shows a solid desk. Filling the holes
        # of every closed outline turns those shells into the solid blocks the
        # user sees in Gazebo - and anything enclosed by a closed outline is
        # unreachable anyway, so nothing drivable is lost.
        before = structure.sum()
        structure = ndimage.binary_fill_holes(structure)
        if verbose:
            print(f"  outlines filled: +{(structure.sum()-before)*res*res:.1f} m2 "
                  f"of solid furniture")
    if structure_dilate > 0:
        structure = ndimage.binary_dilation(
            structure, iterations=int(structure_dilate))

    occupied = structure | ~cover
    if free_floating and structure_low is not None:
        # The map is never allowed to be LOOSER than the mesh: every cell where
        # the mesh has geometry at robot height becomes an obstacle, even if
        # the collision-mesh projection missed it.
        occupied = occupied | (structure_low & cover)
        # A floating blob is only a problem where it cuts drivable space in
        # two. The reference is the mesh's own drivable space at robot height -
        # NOT this map's free space, which the projection has already polluted:
        # testing against the map would call every blob "harmless" as soon as
        # the map had a single free component.
        solid = ndimage.binary_closing(structure_low, structure=np.ones((3, 3)))
        floating = structure & ~solid
        truth_free = ~solid & cover
        lab_f, _ = ndimage.label(truth_free)
        lab_fl, n_fl = ndimage.label(floating, structure=np.ones((3, 3)))
        dropped = 0.0
        objs = ndimage.find_objects(lab_fl, max_label=n_fl)
        for i in range(1, n_fl + 1):
            sl = objs[i - 1]
            if sl is None:
                continue
            blob = lab_fl[sl] == i
            ring = ndimage.binary_dilation(blob, iterations=3) & ~blob
            sides = set(np.unique(lab_f[sl][ring])) - {0}
            if len(sides) >= 2:
                structure[sl][blob] = False
                occupied[sl][blob] = False
                dropped += blob.sum() * res * res
        if verbose:
            print(f"  floating geometry dropped: {dropped:.1f} m2 "
                  f"(it was sealing passages)")
    if min_width and min_width > 0:
        # Erase drivable space that is narrower than min_width. A robot can
        # physically squeeze into a 0.4 m slot but cannot turn or back out of
        # one, and the planner happily routes through it as a shortcut - that
        # is the "robot stuck on the edge" failure. Filling such slots here
        # fills them in the physics too, because make_collision_boxes.py builds
        # the colliders from this very grid.
        r = max(1, int(round(0.5 * min_width / res)))
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        free = ndimage.binary_opening(~occupied, structure=disk)
        occupied = ~free
    if smooth and smooth > 0:
        k = 2 * int(smooth) + 1
        occupied = ndimage.median_filter(
            occupied.astype(np.uint8), size=k, mode="nearest").astype(bool)
    if fill_pockets and fill_pockets > 0:
        # Free pockets smaller than this are enclosed by obstacles and too
        # small to hold a robot: filling them removes the white pinpricks a
        # fragmented scan leaves in the middle of a wall mass.
        limit = max(1, int(round(fill_pockets / (res * res))))
        lab_f, n_f = ndimage.label(~occupied)
        sizes_f = ndimage.sum(~occupied, lab_f, range(1, n_f + 1))
        for i in np.where(sizes_f <= limit)[0]:
            occupied[lab_f == i + 1] = True
    if fill_pits and fill_pits > 0:
        # Obstacle islands smaller than this, with free floor all the way
        # around them, are scan noise rather than furniture; keeping them cost
        # a 0.7 m inflation blob each in the middle of the aisle.
        limit = max(1, int(round(fill_pits / (res * res))))
        lab_o, n_o = ndimage.label(occupied)
        border = set(lab_o[0, :]) | set(lab_o[-1, :]) | set(lab_o[:, 0]) | set(lab_o[:, -1])
        sizes_o = ndimage.sum(occupied, lab_o, range(1, n_o + 1))
        for i in np.where(sizes_o <= limit)[0]:
            if i + 1 in border:
                continue          # touches the edge: part of the building shell
            sel = lab_o == i + 1
            ring = ndimage.binary_dilation(sel, iterations=1) & ~sel
            if ring.any() and not occupied[ring].any():
                occupied[sel] = False
    if fill_notches and fill_notches > 0:
        # Closing the obstacle mask fills every concavity narrower than the
        # kernel, i.e. the bites a fragmented scan leaves in furniture blocks,
        # without touching convex outlines. It can also seal free gaps that
        # narrow - harmless at 0.5 m, hopeless for a fleet that needs 1.1 m
        # doorways, so the value stays far below that.
        r = max(1, int(round(0.5 * fill_notches / res)))
        yy, xx = np.ogrid[-r:r + 1, -r:r + 1]
        disk = (xx * xx + yy * yy) <= r * r
        # border_value=1: outside the map is obstacle, not free. With the
        # default (0) the erosion step eats a 0.25 m band off the whole map
        # boundary, which silently *removes* tens of m^2 of wall.
        closed = ndimage.binary_closing(occupied, structure=disk,
                                        border_value=1)
        # ...but a concavity can also be a doorway: on this map the closing
        # plugged 7 passages, the worst a 0.11 m^2 blob that cut a 17.7 m^2
        # room off the main region. So every blob the closing adds is checked:
        # if it separates free space that used to be connected, it is a
        # passage, not a notch, and it goes back.
        added = closed & ~occupied
        if added.any():
            lab_free_before, _ = ndimage.label(~occupied)
            lab_free_after, _ = ndimage.label(~closed)
            lab_add, n_add = ndimage.label(added, structure=np.ones((3, 3)))
            objs = ndimage.find_objects(lab_add, max_label=n_add)
            for i in range(1, n_add + 1):
                sl = objs[i - 1]
                if sl is None:
                    continue
                pad = 3
                r0 = max(0, sl[0].start - pad)
                r1 = min(closed.shape[0], sl[0].stop + pad)
                c0 = max(0, sl[1].start - pad)
                c1 = min(closed.shape[1], sl[1].stop + pad)
                blob = lab_add[r0:r1, c0:c1] == i
                ring = ndimage.binary_dilation(blob, iterations=2) & ~blob
                sides_after = set(np.unique(lab_free_after[r0:r1, c0:c1][ring])) - {0}
                if len(sides_after) < 2:
                    continue
                sides_before = set(np.unique(lab_free_before[r0:r1, c0:c1][ring])) - {0}
                if len(sides_before) > 1:
                    continue          # those two sides were already separate
                closed[r0:r1, c0:c1][blob] = False
        occupied = closed
    if edits:
        def box_cells(box):
            x0, y0, x1, y1 = box
            c0 = max(0, int((min(x0, x1) - xmin) / res))
            c1 = min(W, int((max(x0, x1) - xmin) / res) + 1)
            r0 = max(0, H - int((max(y0, y1) - ymin) / res) - 1)
            r1 = min(H, H - int((min(y0, y1) - ymin) / res) + 1)
            return r0, r1, c0, c1

        for box in edits.get("fill", []):
            r0, r1, c0, c1 = box_cells(box)
            occupied[r0:r1, c0:c1] = True
        for box in edits.get("erase", []):
            r0, r1, c0, c1 = box_cells(box)
            occupied[r0:r1, c0:c1] = False
    if fill_unreachable and fill_unreachable > 0:
        # Free pockets that no robot can reach: keep only the largest drivable
        # region. They are real floor, but they are behind furniture the fleet
        # cannot pass, and in RViz the white blobs read as blank circles.
        limit = max(1, int(round(fill_unreachable / (res * res))))
        lab_f, n_f = ndimage.label(~occupied)
        free_mask = ~occupied
        sizes_f = ndimage.sum(free_mask, lab_f, range(1, n_f + 1))
        main = int(np.argmax(sizes_f)) + 1
        for i in range(1, n_f + 1):
            if i == main or sizes_f[i - 1] > limit:
                continue
            occupied[lab_f == i] = True
    if rectangularize and rectangularize > 0:
        # Turn every blobby furniture cluster into the solid rectangle it looks
        # like in the room: the scan leaves furniture as a rubble of small
        # patches, which reads as a pitted map. Thin, elongated blobs are left
        # alone - those are walls, and boxing them would eat the aisles. Any
        # rectangle that would cut drivable space in two is reverted, same
        # passage guard as everywhere else in this pipeline.
        lab_o, n_o = ndimage.label(occupied)
        border = set(lab_o[0, :]) | set(lab_o[-1, :]) | set(lab_o[:, 0]) | set(lab_o[:, -1])
        border.discard(0)
        objs = ndimage.find_objects(lab_o, max_label=n_o)
        # Only cells the fleet cannot occupy anyway may be filled: a cell with
        # less than `rectangularize` metres of clearance is not part of any
        # usable path, so boxing it can never take an aisle away. (Without this
        # the first version of the pass narrowed the corridor at pickup_C and
        # dropoff_B below the fleet's requirement.)
        dist_before = ndimage.distance_transform_edt(~occupied) * res
        fillable = dist_before < rectangularize
        candidates = []
        for i in range(1, n_o + 1):
            if i in border:
                continue
            sl = objs[i - 1]
            if sl is None:
                continue
            sel = lab_o[sl] == i
            if sel.sum() * res * res < 0.30:
                continue
            thickness = 2.0 * ndimage.distance_transform_edt(sel).max() * res
            if thickness < 0.20:
                continue          # a wall, not a piece of furniture
            r0, r1 = sl[0].start, sl[0].stop
            c0, c1 = sl[1].start, sl[1].stop
            original = occupied[r0:r1, c0:c1].copy()
            add = (~original) & fillable[r0:r1, c0:c1]
            occupied[r0:r1, c0:c1] |= add
            candidates.append((i, r0, r1, c0, c1, original, add))
        # The guard needs the free space as it was *before* the boxes, so undo
        # every box once, label, then redo them and label again.
        for _, r0, r1, c0, c1, original, _add in candidates:
            occupied[r0:r1, c0:c1] = original
        lab_free_before, _ = ndimage.label(~occupied)
        for _, r0, r1, c0, c1, original, add in candidates:
            occupied[r0:r1, c0:c1] = original | add
        lab_free_after, _ = ndimage.label(~occupied)
        kept = 0
        cells_added = 0
        for _i, r0, r1, c0, c1, original, add in candidates:
            ring_box = ndimage.binary_dilation(add, iterations=2) & ~add
            after = set(np.unique(lab_free_after[r0:r1, c0:c1][ring_box])) - {0}
            if len(after) >= 2:
                before = set(np.unique(lab_free_before[r0:r1, c0:c1][ring_box])) - {0}
                if len(before) <= 1:      # this box is what cut the passage
                    occupied[r0:r1, c0:c1] = original
                    continue
            kept += 1
            cells_added += int(add.sum())
        if verbose:
            print(f"  rectangularised: {kept} furniture blocks, "
                  f"+{cells_added*res*res:.1f} m2 of obstacle")
    meta = {
        "res": res,
        "xmin": xmin,
        "ymin": ymin,
        "W": W,
        "H": H,
        "z_limit": z_limit,
        "covered_m2": float(cover.sum()) * res * res,
        "structure_m2": float(structure.sum()) * res * res,
        "free_m2": float((~occupied).sum()) * res * res,
        "bbox_m2": W * H * res * res,
    }
    if verbose:
        print(f"grid {W}x{H} @ {res} m, origin ({xmin:.2f}, {ymin:.2f}), "
              f"covers {xmax-xmin:.1f} x {ymax-ymin:.1f} m")
        print(f"  obstacles: {len(faces)} collision triangles "
              f"(structure covers {meta['structure_m2']:.1f} m^2 of the grid)")
        print(f"  scanned floor {meta['covered_m2']:.1f} m^2 of {meta['bbox_m2']:.1f} m^2, "
              f"drivable {meta['free_m2']:.1f} m^2")
    return occupied, meta


def report(occupied, meta, min_spot_radius=0.6, separation=3.0):
    from scipy import ndimage

    res = meta["res"]
    free = ~occupied
    lab, n = ndimage.label(free)
    if n == 0:
        print("no free space at all - check the polarity!")
        return None
    sizes = ndimage.sum(free, lab, range(1, n + 1))
    order = np.argsort(sizes)[::-1]
    print(f"free components: {n}; largest {sizes[order[0]]*res*res:.1f} m^2 "
          f"({100*sizes[order[0]]/free.sum():.0f}% of all free space)")
    for i in order[:4]:
        r, c = np.argwhere(lab == i + 1)[0]
        print(f"   comp {i+1}: {sizes[i]*res*res:7.1f} m^2 at "
              f"({meta['xmin']+c*res:.1f}, {meta['ymin'] + (meta['H']-r)*res:.1f})")

    main = lab == (order[0] + 1)
    dist = ndimage.distance_transform_edt(main) * res
    print("clearance available over the largest free component:")
    for width in (0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0):
        frac = (dist >= width / 2).sum() / main.sum()
        print(f"   >= {width:.1f} m wide: {100*frac:5.1f}%  "
              f"({(dist >= width/2).sum()*res*res:6.1f} m^2)")

    peaks = np.argwhere(dist > min_spot_radius)
    order = np.argsort(dist[peaks[:, 0], peaks[:, 1]])[::-1]
    picked = []
    for idx in order:
        r, c = peaks[idx]
        x = meta["xmin"] + (c + 0.5) * res
        y = meta["ymin"] + (meta["H"] - 0.5 - r) * res
        if all((x - ox) ** 2 + (y - oy) ** 2 > separation ** 2 for ox, oy, _ in picked):
            picked.append((float(x), float(y), float(dist[r, c])))
    print(f"\nroomiest spots (clearance radius >= {min_spot_radius} m, "
          f">= {separation} m apart):")
    for x, y, d in picked:
        print(f"   ({x:7.2f}, {y:7.2f})  r={d:.2f} m  -> fits a "
              f"{2*d:.2f} m wide robot")
    return picked


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("visual_obj")
    ap.add_argument("collision_obj")
    ap.add_argument("out_basename", nargs="?")
    ap.add_argument("--wall-smooth-radius", type=float, default=0.10,
                    help="channel-preserving wall smoothing radius in metres; 0 disables")
    ap.add_argument("--res", type=float, default=0.05)
    ap.add_argument("--margin", type=float, default=0.6)
    ap.add_argument("--z-limit", type=float, default=None,
                    help="optional: only triangles dipping below this height "
                         "count as obstacles (default: all of them)")
    ap.add_argument("--cover-close", type=float, default=0.5)
    ap.add_argument("--structure-close", type=float, default=0.1)
    ap.add_argument("--smooth", type=int, default=1,
                    help="majority-vote passes over the occupancy mask; smooths "
                         "the staircase edge the mesh projection leaves behind "
                         "(0 disables, 1 = 3x3 vote, 2 = 5x5, ...)")
    ap.add_argument("--min-width", type=float, default=0.60,
                    help="drivable space narrower than this is filled in, so "
                         "the robots can never be sent into a slot they cannot "
                         "manoeuvre out of (0 disables)")
    ap.add_argument("--structure-dilate", type=int, default=0,
                    help="grow walls by this many cells after rasterisation "
                         "(0 = off; see build_map's docstring)")
    ap.add_argument("--fill-pits", type=float, default=0.15,
                    help="fill obstacle islands smaller than this (m^2) that "
                         "are completely surrounded by drivable floor - scan "
                         "noise that inflates into a blob in the middle of an "
                         "aisle. Keep it small: fragmented furniture arrives "
                         "as small islands too (0 disables)")
    ap.add_argument("--fill-pockets", type=float, default=0.50,
                    help="fill free pockets smaller than this (m^2) that are "
                         "enclosed by obstacles - white pinpricks inside a "
                         "wall (0 disables)")
    ap.add_argument("--fill-unreachable", type=float, default=0.0,
                    help="fill free pockets smaller than this (m^2) that are "
                         "NOT connected to the largest drivable region - the "
                         "blank circles a fragmented scan leaves in the walls "
                         "(0 disables)")
    ap.add_argument("--fill-notches", type=float, default=0.0,
                    help="fill concavities narrower than this (m) in the "
                         "obstacles, so a furniture block with a bite out of "
                         "it becomes solid (0 disables)")
    ap.add_argument("--edits", default=None,
                    help="YAML file of manual erase/fill boxes (default: "
                         "src/hetero_fleet_bringup/maps/map_edits.yaml when it "
                         "exists)")
    ap.add_argument("--structure-band", default=None, metavar="LO,HI",
                    help="build the obstacles from the VISUAL mesh faces that "
                         "intersect this height band (robot's own extent, "
                         "e.g. 0.05,0.45) instead of projecting every collision "
                         "face above 0.4 m top-down. This is what makes the map "
                         "match what the lidars see: walls and furniture legs "
                         "block, table tops you drive under do not")
    ap.add_argument("--free-floating", action="store_true",
                    help="drop only the pieces of geometry that exist solely "
                         "ABOVE the robots and that seal a passage (the scan "
                         "records whole furniture clusters as a floating "
                         "surface, and projecting those closes everything "
                         "underneath). Furniture that does not cut drivable "
                         "space in two stays on the map")
    ap.add_argument("--rectangularize", type=float, default=0.0, metavar="MIN_THICK",
                    help="replace every compact furniture blob with its solid "
                         "bounding rectangle (blobs thinner than this value in "
                         "metres are left alone: those are walls). 0 disables")
    ap.add_argument("--squash-mesh", type=float, default=None, metavar="FLOOR_CUT",
                    help="build the obstacles by pressing the WHOLE visual mesh "
                         "flat: every face standing more than FLOOR_CUT metres "
                         "above the floor is projected top-down (walls, "
                         "furniture tops, lintels - the map then looks like the "
                         "Gazebo view from above). Passages are opened "
                         "afterwards with --free-floating. Try 0.15")
    ap.add_argument("--squash-max", type=float, default=None, metavar="CEIL_CUT",
                    help="with --squash-mesh: also drop the faces that live "
                         "ENTIRELY above this height. Door lintels, beams and "
                         "signs hang over the fleet, and pressing them flat "
                         "seals the doorway they hang over. 2.0 keeps every "
                         "piece of furniture, including the ones the scan only "
                         "recorded as a top surface")
    ap.add_argument("--fill-outlines", action="store_true",
                    help="fill the inside of every closed outline in the "
                         "structure mask: the scan records furniture as a "
                         "surface shell, and projected top-down that reads as a "
                         "scribble instead of the solid desk Gazebo shows")
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--png", default=None)
    a = ap.parse_args()

    edits = None
    edits_path = Path(a.edits) if a.edits else Path(
        "src/hetero_fleet_bringup/maps/map_edits.yaml")
    if edits_path.exists():
        import yaml as _yaml
        edits = _yaml.safe_load(edits_path.read_text()) or {}
        print(f"edits      : {edits_path} "
              f"({len(edits.get('erase', []))} erase, "
              f"{len(edits.get('fill', []))} fill)")

    occupied, meta = build_map(a.visual_obj, a.collision_obj, a.res, a.margin,
                               a.z_limit, a.cover_close, a.structure_close,
                               a.smooth, a.structure_dilate, a.min_width,
                               a.fill_pits, a.fill_pockets, a.fill_unreachable,
                               a.fill_notches, edits,
                               tuple(float(v) for v in a.structure_band.split(","))
                               if a.structure_band else None,
                               a.free_floating,
                               a.rectangularize,
                               a.squash_mesh,
                               a.fill_outlines,
                               a.squash_max)
    if a.wall_smooth_radius > 0:
        from smooth_wall_map import smooth as smooth_walls
        occupied, smoothing_report = smooth_walls(occupied, a.res, a.wall_smooth_radius)
        print(f"wall smoothing: {smoothing_report}")
    if a.report:
        report(occupied, meta)

    if a.out_basename:
        im = Image.fromarray(np.where(occupied, 0, 255).astype(np.uint8))
        im.save(a.out_basename + ".pgm")
        Path(a.out_basename + ".yaml").write_text(
            f"image: {Path(a.out_basename).name}.pgm\n"
            f"resolution: {a.res}\n"
            f"origin: [{meta['xmin']:.6f}, {meta['ymin']:.6f}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
        print(f"wrote {a.out_basename}.pgm ({meta['W']}x{meta['H']}) and .yaml")

    if a.png:
        Image.fromarray(np.where(occupied, 40, 255).astype(np.uint8)).save(a.png)
        print(f"wrote {a.png}")


if __name__ == "__main__":
    main()
