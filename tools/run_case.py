"""Load or execute one disposable PX4 SITL case on the local simulator."""
import argparse,os,subprocess,sys
from prepare_case import prepare,SEEDS
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--seed',type=int,choices=SEEDS,default=202)
p.add_argument('--check-only',action='store_true',help='Load plan only; do not start simulators or send commands')
p.add_argument('--no-gui',action='store_true')
p.add_argument('--hold-uav2',action='store_true')
a=p.parse_args();case=prepare(a.seed)
env=dict(os.environ,UAV_FARM_CASE_ROOT=str(case),PYTHONDONTWRITEBYTECODE='1')
command=[sys.executable,str(case/'scripts/run_execution_sim.py'),'--vehicles','3','--plan',str(case/'config/fleet_drill.json')]
if not a.check_only:
    command+=['--execute','--simulation-only']
    if not a.no_gui:command+=['--gui']
if a.hold_uav2:command+=['--hold-uav2']
print('Case:',case,flush=True)
raise SystemExit(subprocess.call(command,env=env,cwd=case))
