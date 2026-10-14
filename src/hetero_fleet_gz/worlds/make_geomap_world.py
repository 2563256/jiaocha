#!/usr/bin/env python3
"""Build the fleet world on top of the scanned map.

The source model, vendored at worlds/source/geomap_combined12_noroof.sdf
(originally the desktop `~/Desktop/geomap_combined12_noroof.sdf`, model
`geomap_combined12_noroof`) is a *viewer* model: the coloured scan with a
triangle-mesh collider. It is missing everything a fleet simulation needs, and
one of the things it does have (mesh collision) does not survive contact with
the rest of this workspace. This script generates the fleet world from scratch
instead of editing the user's file, so the original stays untouched and the
derivation is repeatable:

  1. Physics with DART and NO triangle-mesh collision. The scan's collision
     geometry is a 297k-face mesh; DART/ODE segfault on mesh collisions in this
     Gazebo (that is exactly why the user's own world file pins Bullet - see
     its comment) and Bullet cannot turn a wheeled base on this map. The mesh
     is replaced by the axis-aligned boxes tools/make_collision_boxes.py
     derives from the occupancy grid (3.0k of them on this scan).

  2. Sensors + scene systems, which the source world lacks. Without the
     sensors system the GPU lidars on the robots never publish a scan and Nav2
     has nothing to build a costmap from.

  3. A ground plane at the height the robots drive on. The map model is
     levelled so its floor sits at z=0 (tools/prepare_map_mesh.py) and the floor
     faces were removed from its collision mesh, so this plane *is* the floor.

  4. Pickup / dropoff / charger markers, at coordinates verified against the
     occupancy grid by tools/pick_zones.py.

Usage:
    python3 make_geomap_world.py [output.sdf]
"""

import sys
from pathlib import Path

MODEL = "geomap_combined12_noroof"
SDF_VERSION = "1.11"

# Vertical placement. Three numbers, all in metres:
#
#   MAP_Z     the scan model is lifted by this much so its floor (which
#             prepare_map_mesh.py levels to z=0; this scan's floor is a single
#             flat plane, so it lands on exactly 0.000) sits clearly above the
#             world origin instead of straddling it.
#   GROUND_Z  the (invisible) plane the robots actually drive on. Keep it just
#             under the floor's surface: wheels then roll ON the blue floor.
#   GROUND_VISUAL_DZ  how far DOWN the grey 300x300 plane is drawn. The scan's
#             floor is still noisy at the centimetre level, so a grey plane
#             level with it wins the depth test in half the pixels and hides
#             the blue floor completely (exactly the "can't see the blue floor"
#             symptom). Sinking the *visual* only means the physics still uses
#             GROUND_Z while the scan's floor is never covered by anything
#             grey.
MAP_Z = 0.10
# The floor surface lands at MAP_Z + 0.000, so the driving plane sits 1.7 cm
# above it - just clear of the levelled floor's own +-2 cm of scan noise.
GROUND_Z = MAP_Z + 0.017
GROUND_VISUAL_DZ = -0.30

# Physics step. This is the knob that decides whether the window runs at
# real time, and it is dictated by the COLLIDER COUNT, not by the robots:
#
#   * the 26 x 10 m lab map this workspace used to hold produced 1.6k box
#     colliders; at a 1 ms step that measured window RTF 0.34, so it was moved
#     to 2.5 ms (400 Hz) and ran at ~1.0.
#   * this 25 x 84 m building produces 3.0k of them (its shell of obstacles is
#     ~2x longer), and the cost is close to linear in the box count: measured
#     on this machine (gz sim 9.6, headless, DART, 4 robots, 5 lidars)
#       3013 boxes, no robots, 2.5 ms step -> RTF 0.72
#       3013 boxes, 4 robots, 2.5 ms step -> RTF 0.35
#       0 boxes   (visual mesh only), 2.5 ms step -> RTF 1.00
#     so the boxes, not the scan mesh or the sensors, eat the step budget.
#
# 5 ms (200 Hz) is the compromise: still 2 physics steps per 100 Hz controller
# cycle (see the update_rate in hetero_fleet_description/config/*_controllers.yaml),
# and the fastest thing on the map is a 0.066 m wheel turning at 4.5 rad/s,
# i.e. 0.0225 rad - 1.5 mm of travel - per step. Raise it to 0.01 if you only
# care about wall-clock and not about contact fidelity; going the other way
# (back to 1 ms) needs fewer colliders first: rerun make_collision_boxes.py
# with a smaller --margin, or --block 2, which is cheaper but fattens every
# wall by up to 5 cm and can wedge a robot on a path the planner certified.
PHYSICS_STEP = 0.001  # G1 torque policy requires 1 ms integration.
PHYSICS_RATE = int(round(1.0 / PHYSICS_STEP))  # target steps per second
PHYSICS_NAME = f"{PHYSICS_STEP * 1000:g}ms"

# (name, x, y, sx, sy, rgba) - candidate coordinates come from
# tools/pick_zones.py, which chooses them on the occupancy grid the planner
# will load; every one of them was then re-checked by tools/verify_scene.py
# (free cell, clearance >= 0.53 m for the widest robot, all ten inside ONE
# connected region, so a task between any pickup and any dropoff is plannable).
#
# Dispersed indoor sites and charging bays.
ZONES = [
    ('pickup_A', 13.00, -30.00, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('pickup_B', 2.97, -30.25, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('pickup_C', 9.00, -53.50, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('pickup_D', 10.00, -33.80, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('pickup_E', 5.00, -67.00, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('dropoff_A', 8.00, -25.60, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    ('dropoff_B', 8.00, -43.60, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    ('dropoff_C', 10.22, -37.50, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    ('dropoff_D', 10.20, -49.20, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    ('dropoff_E', 6.00, -70.00, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    # North-room sites: tasks must require light_payload.
    ('pickup_F', 5.00, -17.50, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('pickup_G', 12.00, -12.50, 0.6, 0.6, '0.10 0.85 0.20 0.55'),
    ('dropoff_F', 10.00, -21.00, 0.6, 0.6, '0.15 0.35 0.90 0.55'),
    ('charger_A', 2.00, -11.50, 0.5, 0.5, '0.95 0.80 0.10 0.55'),
    ('charger_B', 2.00, -21.50, 0.5, 0.5, '0.95 0.80 0.10 0.55'),
    ('charger_C', 2.00, -38.00, 0.5, 0.5, '0.95 0.80 0.10 0.55'),
    ('charger_D', 2.00, -69.50, 0.5, 0.5, '0.95 0.80 0.10 0.55'),
    ('charger_E', 2.70, -32.00, 0.5, 0.5, '0.95 0.80 0.10 0.55'),
]

HEAD = f"""<?xml version='1.0'?>
<!-- generated by make_geomap_world.py - do not edit by hand -->
<sdf version='{SDF_VERSION}'>
  <world name='geomap_fleet'>
    <!-- DART, not Bullet.

         The world used to pin Bullet because DART segfaults on the map's
         209k-triangle collision mesh. But Bullet-Featherstone cannot rotate a
         wheeled base on this map at all: the drive wheels counter-rotate at
         exactly the commanded rate while the body yaw stays at zero, so every
         Nav2 goal that needs a turn dies with "Failed to make progress". The
         same thing happens to Gazebo's own reference diff-drive demo model
         when Bullet is forced, and the same robots turn correctly under DART -
         the engine is the variable, not the robot model.

         The mesh collision is gone: tools/make_collision_boxes.py now turns
         the Nav2 occupancy grid into 3.0k axis-aligned box colliders, which
         DART handles natively (and which match the costmap cell for cell).
         See that script's docstring for the full story, and PHYSICS_STEP
         below for what that many colliders costs.

         If you ever regenerate the map, keep the order:
           prepare_map_mesh.py -> make_nav2_map.py -> make_collision_boxes.py
           -> make_geomap_world.py -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics">
      <engine><filename>gz-physics-dartsim-plugin</filename></engine>
      <max_step_size>{PHYSICS_STEP}</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </plugin>
    <plugin filename="gz-sim-user-commands-system"
            name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system"
            name="gz::sim::systems::SceneBroadcaster"/>
    <!-- Missing from the viewer world. No sensors system = no lidar = Nav2
         has no costmap and every robot just sits there.
         render_engine: ogre2 draws mesh materials white in this install, so
         the coloured scan must be rendered by the ogre (1) engine; see
         gz_sim.launch.py, which passes the matching render-engine argument.
         (Written without the leading dashes on purpose: a double hyphen is
         illegal inside an XML comment and makes the file unparseable by
         xmllint / ElementTree.) -->
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>

    <!-- 5 ms steps (200 Hz), not the usual 1 ms - see PHYSICS_STEP. -->
    <physics name='{PHYSICS_NAME}' type='ignored'>
      <max_step_size>{PHYSICS_STEP}</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>{PHYSICS_RATE}</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.75 0.78 0.82 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 12 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>

    <!-- The driving surface. The scan's own floor mesh is visual-only; see
         tools/prepare_map_mesh.py for why. -->
    <model name='ground_plane'>
      <static>true</static>
      <pose>0 0 {GROUND_Z} 0 0 0</pose>
      <link name='link'>
        <collision name='collision'>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
            <bounce><restitution>0.0</restitution></bounce>
            <contact/>
          </surface>
        </collision>
        <visual name='visual'>
          <pose>0 0 {GROUND_VISUAL_DZ} 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>300 300</size></plane></geometry>
          <material>
            <ambient>0.35 0.35 0.36 1</ambient>
            <diffuse>0.35 0.35 0.36 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- The scan: coloured visual mesh + the same surface without its floor as
         collision. Both are levelled so the floor is at z=0. -->
    <include>
      <uri>model://{MODEL}</uri>
      <name>{MODEL}</name>
      <pose>0 0 {MAP_Z} 0 0 0</pose>
    </include>
"""


def zone_sdf() -> str:
    chunks = []
    for name, x, y, sx, sy, rgba in ZONES:
        chunks.append(f"""    <model name='zone_{name}'>
      <static>true</static>
      <pose>{x} {y} {GROUND_Z + 0.03:.3f} 0 0 0</pose>
      <link name='pad'>
        <visual name='pad_visual'>
          <pose>0 0 0.01 0 0 0</pose>
          <geometry><box><size>{sx} {sy} 0.02</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        </visual>
        <visual name='rim_visual'>
          <pose>0 0 0.05 0 0 0</pose>
          <geometry><box><size>{sx} {sy} 0.08</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
        </visual>
      </link>
    </model>
""")
    return "".join(chunks)


def main() -> int:
    here = Path(__file__).resolve().parent
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else here / "geomap_fleet.sdf"
    out.write_text(HEAD + zone_sdf() + "  </world>\n</sdf>\n")
    print(f"wrote {out}")
    print(f"  map model   : model://{MODEL} (visual + box colliders from the "
          f"occupancy grid)")
    print(f"  scan model   : z = {MAP_Z}  (floor lands at ~{MAP_Z:.3f} m)")
    print(f"  driving plane: z = {GROUND_Z} (visual sunk {GROUND_VISUAL_DZ})")
    print(f"  zones       : {len(ZONES)} markers")
    return 0


if __name__ == "__main__":
    sys.exit(main())
