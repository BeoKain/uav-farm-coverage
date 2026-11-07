#!/usr/bin/env python3
"""Isolated farm launcher. Never sends ARM, takeoff, or Offboard commands."""
import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from project_paths import case_root, px4_root
ROOT = case_root()
RUNTIME = ROOT / 'runtime'
STATE = RUNTIME / 'session.json'


def start_token(pid):
    try:
        data = Path(f'/proc/{pid}/stat').read_text().rsplit(')',1)[1].split()
        return data[19]  # Linux field 22 (starttime), after pid and comm.
    except (FileNotFoundError, ProcessLookupError):
        return None


def alive(p):
    return start_token(p['pid']) == p['start_token']


def stop():
    if not STATE.exists():
        print('No farm session recorded.'); return
    entries=json.loads(STATE.read_text())['processes']
    for p in reversed(entries):
        if alive(p):
            print('Stopping owned process:',p['name'],p['pid'])
            try: os.killpg(p['pid'],signal.SIGTERM)
            except ProcessLookupError: pass
    deadline=time.monotonic()+5
    while time.monotonic()<deadline and any(alive(p) for p in entries):
        time.sleep(.1)
    for p in reversed(entries):
        if alive(p):
            try: os.killpg(p['pid'],signal.SIGKILL)
            except ProcessLookupError: pass
    STATE.unlink(missing_ok=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',choices=['check','preview','start','status','stop'])
    parser.add_argument('--vehicles',type=int,choices=[1,2,3],default=3)
    parser.add_argument('--gui',action='store_true',help='Open Gazebo GUI (server is always started)')
    parser.add_argument('--duration',type=float,default=0,help='Stop after N seconds; 0 means until Ctrl+C')
    parser.add_argument('--verify-telemetry',action='store_true',help='Read-only ROS 2 smoke check in this session')
    parser.add_argument('--px4-root',type=Path,default=px4_root())
    args=parser.parse_args()
    if args.action=='stop': stop();return
    if args.action=='status':
        if not STATE.exists(): print('Farm simulation stopped.');return
        for p in json.loads(STATE.read_text())['processes']:
            print(p['name'], 'RUNNING' if alive(p) else 'STOPPED',p['pid'])
        return
    gz=shutil.which('gz'); agent=shutil.which('MicroXRCEAgent')
    build=args.px4_root/'build/px4_sitl_default'
    config=json.loads((ROOT/'config/farm.json').read_text())
    if not gz: raise RuntimeError('gz command not found; source /opt/ros/jazzy/setup.bash first')
    if args.action in ('check','start'):
        for path in [build/'bin/px4',build/'etc',args.px4_root/'Tools/simulation/gz/models']:
            if not path.exists(): raise RuntimeError(f'Missing PX4 file: {path}')
        if not agent: raise RuntimeError('MicroXRCEAgent not found')
    env=os.environ.copy()
    env['GZ_SIM_RESOURCE_PATH']=os.pathsep.join([str(ROOT/'models'),
        str(args.px4_root/'Tools/simulation/gz/models'),env.get('GZ_SIM_RESOURCE_PATH','')])
    env['SDF_PATH']=os.pathsep.join([str(ROOT/'models'),env.get('SDF_PATH','')])
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=os.pathsep.join([
        str(build/'src/modules/simulation/gz_plugins'),env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')])
    if args.action=='check':
        subprocess.run([gz,'sdf','-k',str(ROOT/'worlds/farm.sdf')],env=env,check=True)
        subprocess.run([gz,'sdf','-k',str(ROOT/'models/farm_terrain/model.sdf')],env=env,check=True)
        print('Configuration and SDF checks passed. No simulator or flight started.');return
    if STATE.exists():
        recorded=json.loads(STATE.read_text())['processes']
        if any(alive(p) for p in recorded): raise RuntimeError('Farm session already running; use status/stop')
        STATE.unlink()
    # Keep old simulation untouched. In addition, transport discovery below checks worlds.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():continue
        try: name=(proc/'comm').read_text().strip()
        except (OSError,PermissionError):continue
        if name in ('px4','MicroXRCEAgent'):
            raise RuntimeError(f'Existing {name} process detected; stop that session manually first')
    topics=subprocess.run([gz,'topic','-l'],capture_output=True,text=True,timeout=8)
    if any(x.startswith('/world/') and x.endswith('/clock') for x in topics.stdout.splitlines()):
        raise RuntimeError('A Gazebo world already exists; do not mix it with the farm session')
    RUNTIME.mkdir(parents=True,exist_ok=True)
    session=RUNTIME/time.strftime('%Y%m%d_%H%M%S')
    session.mkdir()
    env['GZ_LOG_PATH']=str(session/'gz_logs')
    env['ROS_LOG_DIR']=str(session/'ros_logs')
    entries=[]; handles=[]; children=[]

    def launch(name,command,custom_env=None,cwd=None):
        log=(session/f'{name}.log').open('w');handles.append(log)
        p=subprocess.Popen(command,env=custom_env or env,cwd=cwd or session,
                           stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                           start_new_session=True)
        entries.append(dict(name=name,pid=p.pid,start_token=start_token(p.pid)))
        children.append(p)
        STATE.write_text(json.dumps(dict(log_dir=str(session),processes=entries,
            gazebo_partition=env.get('GZ_PARTITION'),ros_domain_id=env.get('ROS_DOMAIN_ID')),indent=2))
        return p

    def wait_world():
        deadline=time.monotonic()+35
        while time.monotonic()<deadline:
            if children[0].poll() is not None: raise RuntimeError('Gazebo server exited; inspect log')
            r=subprocess.run([gz,'topic','-l'],env=env,capture_output=True,text=True,timeout=5)
            if '/world/farm/clock' in r.stdout.splitlines():return
            time.sleep(.5)
        raise RuntimeError('World startup timed out; inspect gazebo.log')

    def interrupted(signum,frame): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,interrupted)
    try:
        launch('gazebo',[gz,'sim','-r','-s','-v','2',str(ROOT/'worlds/farm.sdf')])
        wait_world()
        if args.action=='start':
            launch('agent',[agent,'udp4','-p','8888'])
            for spawn in config['uav_spawns'][:args.vehicles]:
                venv=env.copy();venv.update(PX4_SIM_MODEL='gz_x500',PX4_SYS_AUTOSTART='4001',
                    PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='farm',
                    PX4_GZ_MODELS=str(ROOT/'models'),
                    PX4_GZ_WORLDS=str(ROOT/'worlds'),
                    PX4_GZ_MODEL_POSE=','.join(map(str,spawn['world_enu_m']+[0,0,0])))
                work=session/spawn['id'];work.mkdir()
                launch(spawn['id'],[str(build/'bin/px4'),'-d','-i',str(spawn['instance']),
                       '-w',str(work),str(build/'etc')],custom_env=venv,cwd=work)
                time.sleep(2)
            if args.verify_telemetry:
                result=subprocess.run([sys.executable,str(ROOT/'scripts/check_telemetry.py'),
                    '--vehicles',str(args.vehicles),'--timeout','40'],env=env,
                    capture_output=True,text=True,timeout=50)
                (session/'telemetry_check.log').write_text(result.stdout+result.stderr)
                print(result.stdout,flush=True)
                if result.returncode:raise RuntimeError('Telemetry check failed; inspect telemetry_check.log')
        if args.gui:
            if args.action=='start':
                deadline=time.monotonic()+30
                while time.monotonic()<deadline:
                    scene=subprocess.run([gz,'service','-s','/world/farm/scene/info',
                        '--reqtype','gz.msgs.Empty','--reptype','gz.msgs.Scene',
                        '--timeout','3000','--req',''],env=env,capture_output=True,text=True,timeout=5)
                    expected=[f'x500_{spawn["instance"]}' for spawn in config['uav_spawns'][:args.vehicles]]
                    if scene.returncode==0 and all(('name: "'+name+'"') in scene.stdout for name in expected):
                        (session/'scene_before_gui.txt').write_text(scene.stdout)
                        print('Server scene ready before GUI:',expected,flush=True)
                        break
                    time.sleep(.25)
                else:raise RuntimeError('Not all UAV models appeared in server scene; GUI not started')
            launch('gui',[gz,'sim','-g','-v','3','--gui-config',str(ROOT/'gui_trails.config')])
        print(f'Farm world running; vehicles={args.vehicles if args.action=="start" else 0}',flush=True)
        print(f'Logs: {session}',flush=True)
        print('No ARM, takeoff, Offboard, or mission commands sent. Ctrl+C stops only this session.',flush=True)
        began=time.monotonic()
        while not args.duration or time.monotonic()-began<args.duration:
            for e,p in zip(entries,children):
                if e['name']!='gui' and p.poll() is not None:
                    raise RuntimeError(f'{e["name"]} exited ({p.returncode}); see logs')
            time.sleep(.25)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        for p in children:
            try:p.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        for f in handles:f.close()


if __name__=='__main__':
    try:main()
    except (RuntimeError,subprocess.SubprocessError) as exc:
        print(f'ERROR: {exc}',file=sys.stderr);sys.exit(1)
