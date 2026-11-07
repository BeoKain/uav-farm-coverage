"""Inspect local dependencies without starting simulators."""
import importlib.util,json,os,shutil,subprocess
from pathlib import Path
repo=Path(__file__).resolve().parents[1]
px4=Path(os.environ.get('PX4_ROOT',Path.home()/'PX4-Autopilot')).expanduser()
ws=Path(os.environ.get('ROS2_WS',Path.home()/'ros2_px4_ws')).expanduser()
ns3=Path(os.environ.get('NS3_ROOT',Path.home()/'ns-allinone-3.43/ns-3.43')).expanduser()
checks={
 'px4_binary':(px4/'build/px4_sitl_default/bin/px4').is_file(),
 'px4_etc':(px4/'build/px4_sitl_default/etc').is_dir(),
 'px4_models':(px4/'Tools/simulation/gz/models').is_dir(),
 'ros_workspace_setup':(ws/'install/setup.bash').is_file(),
 **{cmd:bool(shutil.which(cmd)) for cmd in ('gz','ros2','MicroXRCEAgent')},
 **{mod:importlib.util.find_spec(mod) is not None for mod in ('numpy','pymavlink','rclpy','px4_msgs','sensor_msgs','rosgraph_msgs')},
}
bridge=subprocess.run(['ros2','pkg','prefix','ros_gz_bridge'],capture_output=True,text=True) if shutil.which('ros2') else None
checks['ros_gz_bridge']=bridge is not None and bridge.returncode==0
versions={}
for name,path in [('px4',px4),('px4_msgs',ws/'src/px4_msgs')]:
 r=subprocess.run(['git','-C',str(path),'rev-parse','HEAD'],capture_output=True,text=True)
 versions[name]=r.stdout.strip() if r.returncode==0 else None
lock=json.loads((repo/'configs/environment.lock.json').read_text())['observed_environment']
checks['px4_commit_matches_recorded']=versions['px4']==lock['px4_commit']
checks['px4_msgs_commit_matches_recorded']=versions['px4_msgs']==lock['px4_msgs_commit']
network_ready=(ns3/'VERSION').is_file() and (ns3/'VERSION').read_text().strip()=='3.43' and (ns3/'build/lib').is_dir()
print(json.dumps(dict(sitl_checks=checks,source_commits=versions,ns3_optional_ready=network_ready,simulators_started=0),indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
