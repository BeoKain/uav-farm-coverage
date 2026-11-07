#!/usr/bin/env python3
"""Own one/three-UAV collection SITL; recover before simulator cleanup."""
import argparse
import csv
import math
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time
from fleet_plan import ROOT,load,check_session
from sweep_interrupts import StopRequest,restore_handlers


class Cancelled(Exception):pass


def send_stop(process,signum=signal.SIGINT):
    if process is not None and process.poll() is None:
        try:process.send_signal(signum)
        except ProcessLookupError:pass


def wait_controller(process,stop,timeout=None,ready=None,interrupt_after=None,before_csv=None, observers=()):
    """Only one stop request; further Ctrl+C never tears down recovery services."""
    began=time.monotonic();forwarded=False;recovery_since=None
    while process.poll() is None:
        if any(child.poll() is not None for child in observers) and not stop.requested:
            print('Observer stopped, requesting coordinated recovery.',flush=True)
            stop.receive(signal.SIGINT)
        if interrupt_after is not None and not stop.requested:
            paths=set((ROOT/'runtime/collection_flight_tests').glob('*_flight.csv'))-(before_csv or set())
            if len(paths)==1:
                try:
                    with next(iter(paths)).open() as f:rows=list(csv.DictReader(f))
                    if any(r['stage']=='MISSION' and float(r['phase_time_s'])>=interrupt_after for r in rows):
                        print('SITL interruption drill threshold reached.',flush=True);stop.receive(signal.SIGINT)
                except (ValueError,KeyError):pass
        if stop.requested and recovery_since is None:recovery_since=time.monotonic()
        if stop.requested and not forwarded and (ready is None or ready.exists()):
            print('Stop requested: waiting for controller recovery; keeping Gazebo and GCS alive.',flush=True)
            send_stop(process);forwarded=True
        if ((timeout is not None and time.monotonic()-began>timeout) or
                (recovery_since is not None and time.monotonic()-recovery_since>85)):
            raise TimeoutError('Recovery not finished; SITL/GCS left running for inspection')
        time.sleep(.1)
    return process.returncode


def flight_summary(before):
    found=set((ROOT/'runtime/collection_flight_tests').glob('*_summary.json'))-before
    if len(found)!=1:return None,None
    path=found.pop()
    return path,json.loads(path.read_text())


def may_cleanup(summary):
    return bool(summary is not None and
                (not summary.get('arm_command_sent',True) or summary.get('landing_confirmed',False)))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--simulation-only',action='store_true')
    parser.add_argument('--gui',action='store_true',help='Open the Gazebo 3D window alongside the simulation server')
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--vehicles',type=int,choices=[1,3],default=1)
    parser.add_argument('--hold-uav2','--hold_uav2',action='store_true')
    parser.add_argument('--interrupt-after',type=float)
    args=parser.parse_args()
    if args.interrupt_after is not None and (not math.isfinite(args.interrupt_after) or args.interrupt_after<=0):parser.error('Positive interrupt time required')
    if args.hold_uav2 and args.vehicles!=3:parser.error('UAV2 hold requires three UAVs')
    plan=load(args.plan,args.vehicles)
    source_args=['--plan',str(args.plan)]
    source_args_extra=['--hold-uav2'] if args.hold_uav2 else []
    print('Validated team plan:',plan['summary'],flush=True)
    if not args.execute:return 0
    if not args.simulation_only:parser.error('Require --simulation-only')
    if (ROOT/'runtime/session.json').exists():raise RuntimeError('A farm session is recorded; inspect status/stop first')
    logs=ROOT/'runtime/collection_flight_sessions'/time.strftime('%Y%m%d_%H%M%S');logs.mkdir(parents=True,exist_ok=False)
    env=os.environ.copy();env['ROS_LOG_DIR']=str(logs/'ros_logs')
    # Isolate transport and ROS graph from an existing user's Gazebo world.
    env['GZ_PARTITION']='farm_arm_v7_'+str(os.getpid())+'_'+str(time.time_ns())
    env['ROS_DOMAIN_ID']='75'
    for name in ('GZ_PARTITION','ROS_DOMAIN_ID'):os.environ[name]=env[name]
    (logs/'isolation.json').write_text(json.dumps({k:env[k] for k in ('GZ_PARTITION','ROS_DOMAIN_ID')},indent=2)+'\n')
    stop=StopRequest();previous=stop.install()
    launcher=controller=heartbeat=network=clock_bridge=None
    observer_processes=[]
    mag=None
    before=set((ROOT/'runtime/collection_flight_tests').glob('*_summary.json'))
    before_csv=set((ROOT/'runtime/collection_flight_tests').glob('*_flight.csv'))
    outcome=0;preserve=False;summary_path=summary=None
    try:
        try:
            with (logs/'simulation.log').open('w') as log:
                launch_command=[sys.executable,str(ROOT/'scripts/farm_sim.py'),'start','--vehicles',str(args.vehicles)]
                if args.gui:launch_command.append('--gui')
                print('Gazebo view:', 'GUI + simulation server' if args.gui else 'headless simulation server',flush=True)
                launcher=subprocess.Popen(launch_command,
                                          env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                deadline=time.monotonic()+80
                while time.monotonic()<deadline:
                    if stop.requested:raise Cancelled()
                    if launcher.poll() is not None:raise RuntimeError('Farm startup failed; see '+str(logs/'simulation.log'))
                    try:check_session(vehicles=args.vehicles);break
                    except (OSError,ValueError,KeyError):time.sleep(.2)
                else:raise RuntimeError('SITL session startup timeout')
                if stop.requested:raise Cancelled()
                bridges=[dict(gz_topic_name='/world/farm/clock',ros_topic_name='/clock',
                              gz_type_name='gz.msgs.Clock',ros_type_name='rosgraph_msgs/msg/Clock',
                              direction='GZ_TO_ROS')]
                for i in range(3):
                    bridges.append(dict(
                        gz_topic_name=f'/world/farm/model/x500_{i}/link/terrain_lidar_link/sensor/terrain_lidar/scan/points',
                        ros_topic_name=f'/uav{i+1}/terrain_lidar/points',
                        gz_type_name='gz.msgs.PointCloudPacked',ros_type_name='sensor_msgs/msg/PointCloud2',
                        direction='GZ_TO_ROS',subscriber_queue=1,publisher_queue=1))
                bridge_config=logs/'bridge.json'
                bridge_config.write_text(json.dumps(bridges,indent=2)+'\n')
                clock_bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
                                               '--ros-args','-p',f'config_file:={bridge_config}'],
                                              env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                heartbeat=subprocess.Popen([sys.executable,str(ROOT/'scripts/sitl_gcs_heartbeat.py'),'--simulation-only','--vehicles',str(args.vehicles)],
                                           env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                until=time.monotonic()+2
                while time.monotonic()<until:
                    if stop.requested:raise Cancelled()
                    time.sleep(.1)
                if heartbeat.poll() is not None:raise RuntimeError('GCS heartbeat failed; see simulation log')
                if clock_bridge.poll() is not None:raise RuntimeError('Gazebo clock bridge failed; see simulation log')
                if stop.requested:raise Cancelled()
                mag=subprocess.Popen([sys.executable,str(ROOT/'scripts/fleet_mag.py'),str(logs/'mag_adapter.json')],
                                     env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                ids=('UAV1','UAV2','UAV3')
                for u in ids:
                    child=subprocess.Popen([sys.executable,str(ROOT/'scripts/collect_fleet_pose.py'),
                                            str(logs/f'{u}.json'),str(ROOT/'config/farm.json'),
                                            u,str(logs),'--flight'],
                                           env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                    observer_processes.append(child)
                truth=subprocess.Popen([sys.executable,str(ROOT/'scripts/record_fleet_truth.py'),
                                        str(logs/'gazebo_truth.csv'),str(logs/'truth.ready')],
                                       env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                observer_processes.append(truth)
                until=time.monotonic()+35.
                while not ((logs/'truth.ready').exists() and all((logs/f'{u}.ready').exists() for u in ids)):
                    if stop.requested:raise Cancelled()
                    if any(p.poll() is not None for p in [*observer_processes,mag,clock_bridge]):
                        raise RuntimeError('Observer exited before readiness')
                    if time.monotonic()>until:raise RuntimeError('Observer readiness timeout before ARM')
                    time.sleep(.05)
                (logs/'start').touch()
                print('Fleet pose ready; starting bounded three-UAV drill.',flush=True)
                ready=logs/'controller.ready'
                controller=subprocess.Popen(
                    [sys.executable,str(ROOT/'scripts/execution_flight_test.py'),'--execute','--simulation-only','--ready-file',str(ready),*source_args,*source_args_extra,'--vehicles',str(args.vehicles)],
                    env=env,start_new_session=True)
                code=wait_controller(controller,stop,ready=ready,interrupt_after=args.interrupt_after,before_csv=before_csv,
                                     observers=[*observer_processes,mag,clock_bridge])
                if code==130 or stop.requested:raise Cancelled()
                if code:raise RuntimeError('Formation flight did not complete; inspect summary')
        except TimeoutError as exc:
            preserve=True;outcome=1;print(str(exc),flush=True)
        except Cancelled:
            outcome=130;print('Run interrupted. No trajectory export or ns-3 replay will be started.',flush=True)
        except Exception as exc:
            outcome=1;print('ERROR:',exc,flush=True)
        finally:
            if controller is not None and controller.poll() is None and not preserve:
                send_stop(controller)
                try:wait_controller(controller,StopRequest(),timeout=85)
                except TimeoutError as exc:preserve=True;outcome=1;print(str(exc),flush=True)
            if controller is not None:
                summary_path,summary=flight_summary(before)
                if not may_cleanup(summary):
                    preserve=True;outcome=1
                    print('Landing/disarm not confirmed: preserving SITL and heartbeat for inspection.',flush=True)
            if not preserve:
                for child in [*observer_processes,mag]:
                    if child is not None:
                        send_stop(child,signal.SIGTERM)
                        child.wait(timeout=8)
                if heartbeat is not None:
                    send_stop(heartbeat,signal.SIGTERM);heartbeat.wait(timeout=5)
                if clock_bridge is not None:
                    send_stop(clock_bridge,signal.SIGTERM);clock_bridge.wait(timeout=5)
                if launcher is not None:
                    send_stop(launcher,signal.SIGTERM);launcher.wait(timeout=20)
        if preserve or outcome:return outcome
        if stop.requested:return 130
        if summary_path is None or summary.get('result')!='DONE':raise RuntimeError('Successful flight summary unavailable')
        from evaluate_fleet import analyze_flight
        checked=analyze_flight(summary_path,logs)
        print('Fleet coordination checks:',checked,flush=True)
        return 0 if checked['status']=='passed' else 1
    finally:restore_handlers(previous)


if __name__=='__main__':
    try:raise SystemExit(main())
    except Exception as exc:print('ERROR:',exc,flush=True);raise SystemExit(1)
