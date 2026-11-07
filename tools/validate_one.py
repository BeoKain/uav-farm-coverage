"""Offline reference checks; no ROS, Gazebo process or flight IO."""
import json,os,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'src/uav_farm'))
from fleet_plan import load
from mission_optimizer import validate_mission
from terrain_links import TerrainGrid,evaluate
from baseline_planner import poses
from probe_protocol import ProbeGate
from arm_deadline import assess
case=Path(os.environ['UAV_FARM_CASE_ROOT'])
plan=load(case/'config/fleet_drill.json',3)
terrain=TerrainGrid.from_csv(case/'config/terrain.csv')
checks=validate_mission(plan['mission'],plan['config'],terrain)
gate=ProbeGate(plan['ids'],poses)
for segment in plan['mission']['segments']:gate.select(segment,segment['start_s'])
sensors={s['id']:s for s in plan['config']['sensors']}
for task in plan['mission']['services']:
    x,y,z=task['position']
    for lift in (0.,.5,1.,1.5):
        assert evaluate(terrain,(x,y,z+lift),sensors[task['sensor']]['world_enu_m'],plan['policy']['collection_range_m'],plan['policy']['min_agl_m'])['geometry_eligible']
budget=json.loads((case/'config/arm_budget.json').read_text())
arm=assess(plan['mission'],plan['config'],budget)
assert arm['deadline_misses']==0
assert len({t['sensor'] for t in plan['mission']['services']})==30
assert max(s[k][2] for s in plan['mission']['segments'] for k in ('start','end'))+1.5<=plan['flight_ceiling_m']+1e-8
print(json.dumps(dict(scenario=case.name,reference_checks=checks,arm_predicted_minimum_slack_s=arm['minimum_slack_s'],served=30,scope='Offline reference only; not a new flight')))
