from pathlib import Path
import yaml
ROOT=Path(__file__).resolve().parents[1]
def test_clock_bridge_uses_world_clock_with_stable_ros_name():
 entries=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
 clocks=[e for e in entries if e['ros_topic_name']=='/clock']
 assert len(clocks)==1 and clocks[0]['gz_topic_name']=='/world/geomap_fleet/clock'
def test_rviz_models_and_both_costmaps_cover_every_fleet_member():
 fleet=yaml.safe_load((ROOT.parent/'hetero_fleet_description/config/robot_profiles.yaml').read_text())['fleet']
 displays=yaml.safe_load((ROOT/'rviz/fleet.rviz').read_text())['Visualization Manager']['Displays']
 for robot in fleet:
  r=robot['name']
  for layer in ['local','global']:
   matches=[d for d in displays if d.get('Topic',{}).get('Value')==f'/{r}/{layer}_costmap/costmap']
   assert len(matches)==1 and matches[0]['Enabled']
  assert any(d.get('Description Topic',{}).get('Value')==f'/{r}/robot_description' and d['Enabled'] for d in displays)
