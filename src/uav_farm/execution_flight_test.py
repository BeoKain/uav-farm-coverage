#!/usr/bin/env python3
"""Telemetry-gated smooth-cruise formation execution; explicit PX4 SITL only."""
import argparse,csv,itertools,json,math,os,time
from pathlib import Path
from collections import deque
from telemetry_watchdog import freshness,landed_fresh,low_rate,POLICY as FRESHNESS_POLICY
from fleet_plan import ROOT,load,check_session,to_local,to_world,vector_to_ned
from baseline_planner import closest_pair as min_segment_distance
from fleet_guard import FleetGuard, LiftedSupervisor
from baseline_planner import poses
from probe_protocol import ProbeGate
def position_at(route,t):
    if t<=route[0][0]:return tuple(route[0][1:])
    for a,b in zip(route,route[1:]):
        if t<=b[0]:
            f=(t-a[0])/(b[0]-a[0]);return tuple(x+f*(y-x) for x,y in zip(a[1:],b[1:]))
    return tuple(route[-1][1:])
from sweep_interrupts import StopRequest,restore_handlers


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true');parser.add_argument('--simulation-only',action='store_true')
    parser.add_argument('--startup-check',action='store_true');parser.add_argument('--ready-file',type=Path,help=argparse.SUPPRESS)
    parser.add_argument('--plan',type=Path,required=True)
    parser.add_argument('--vehicles',type=int,choices=[1,3],default=1)
    parser.add_argument('--hold-uav2',action='store_true')
    args=parser.parse_args()
    if args.hold_uav2 and args.vehicles!=3:parser.error('UAV2 hold requires all three UAVs')
    plan=load(args.plan,args.vehicles)
    ids=plan['ids'];pairs=list(itertools.combinations(ids,2))
    print(json.dumps(dict(assignments=plan['assignments'],summary=plan['summary']),indent=2),flush=True)
    if args.execute and (not args.simulation_only or args.startup_check):parser.error('Require --execute --simulation-only')
    if not args.execute and not args.startup_check:return
    if args.execute:check_session(vehicles=args.vehicles)
    stop=StopRequest();previous=stop.install()
    if args.ready_file:args.ready_file.write_text('ready\n')
    logs=ROOT/'runtime/collection_flight_tests';logs.mkdir(parents=True,exist_ok=True)
    os.environ.setdefault('ROS_LOG_DIR',str(logs/'ros_logs'))
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import String
    from formation_timebase import SimulationClock,SampleGate
    from rclpy.signals import SignalHandlerOptions
    from rclpy.utilities import try_shutdown
    from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
    from px4_msgs.msg import (OffboardControlMode,TrajectorySetpoint,VehicleCommand,VehicleLocalPosition,VehicleStatus,VehicleLandDetected)
    terrain=__import__('terrain_links').TerrainGrid.from_csv(ROOT/'config/terrain.csv')

    class Swarm(Node):
        def __init__(self):
            super().__init__('farm_formation_prototype')
            self.stage='WAIT';self.begin=self.since=time.monotonic();self.last_command=0.;self.owned=False
            self.cancelled=False;self.landing_confirmed=False;self.failure=None;self.prestream=0
            self.phase_time=0.;self.last_clock=None;self.reached=None;self.closest=math.inf;self.closest_interval=math.inf
            self.formation_clock=LiftedSupervisor(plan,terrain)
            self.guard=FleetGuard(self.formation_clock)
            self.observations={}
            self.probes=ProbeGate(ids,poses)
            self.probe_publishers={u:self.create_publisher(String,f'/{u.lower()}/probe_direction',1) for u in ids}
            self.guard_rows=[]
            self.lidar_receipts={u:[] for u in ids}
            self.mission_sim_us=None;self.arm_sim_us=None
            self.sim_clock=SimulationClock()
            self.previous_positions=None;self.max_error=0.;self.min_clearance=math.inf;self.pause_ticks=0
            self.last_record=None;self.vehicles=[]
            self.freshness_failure=None;self.tick_history=deque(maxlen=120);self.previous_tick=None
            self.record_duration_s=0.;self.tick_gap_s=None
            stamp=time.strftime('%Y%m%d_%H%M%S')+f'_{time.time_ns()%1000000:06d}'
            self.csv_path=logs/(stamp+'_flight.csv');self.summary_path=logs/(stamp+'_summary.json')
            self.csv_file=self.csv_path.open('w',newline='');self.writer=csv.writer(self.csv_file)
            self.writer.writerow(['elapsed_wall_s','master_sample_us','uav','px4_sample_us','stage','phase_time_s',
                'world_est_e_m','world_est_n_m','world_est_u_m','command_e_m','command_n_m','command_u_m',
                'track_error_m','arming_state','nav_state','rx_age_s','xy_reset_counter','z_reset_counter','d12_m','d13_m','d23_m','master_raw_sample_us','clock_age_s','source_clock_domain','velocity_e_m_s','velocity_n_m_s','velocity_u_m_s','mission_elapsed_sim_s','arm_elapsed_sim_s','service_completed_count'])
            qos=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(Clock,'/clock',lambda m:self.sim_clock.update(int(m.clock.sec),int(m.clock.nanosec),time.monotonic()),qos)
            position_qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT,durability=DurabilityPolicy.VOLATILE)
            for s in plan['config']['uav_spawns']:
                v=dict(config=s,pos=None,status=None,landed=None,pos_rx=0.,status_rx=0.,land_rx=0.,origin=None,resets=None,command=None,pubs={},sample_gate=SampleGate(),status_gate=SampleGate(),landed_gate=SampleGate(),pos_sim_us=None,status_sim_us=None,landed_sim_us=None)
                self.vehicles.append(v)
                for key,cls,suffix in [('pos',VehicleLocalPosition,'vehicle_local_position_v1'),('status',VehicleStatus,'vehicle_status_v4'),('landed',VehicleLandDetected,'vehicle_land_detected')]:
                    self.create_subscription(cls,s['namespace']+'/out/'+suffix,lambda msg,v=v,k=key:self.receive(v,k,msg),position_qos if key=='pos' else qos)
                if args.execute:
                    for key,cls,suffix in [('mode',OffboardControlMode,'offboard_control_mode'),('point',TrajectorySetpoint,'trajectory_setpoint'),('cmd',VehicleCommand,'vehicle_command')]:
                        v['pubs'][key]=self.create_publisher(cls,s['namespace']+'/in/'+suffix,qos)
                self.create_subscription(String,f"/{s['id'].lower()}/corridor_decision",
                                         lambda msg,u=s['id']:self.lidar(msg,u),1)
            self.create_timer(.05,self.tick)
        def lidar(self,msg,u):
            receipt=time.monotonic()
            row=json.loads(msg.data)
            previous=self.observations.get(u)
            if previous is None or row['stamp_s']>previous['stamp_s']:
                self.observations[u]=row
                self.lidar_receipts[u].append(dict(stamp_s=row['stamp_s'],wall_s=receipt,
                    sim_s=self.sim_clock.us/1e6 if self.sim_clock.us is not None else None))
        def inputs(self):
            index=min(self.formation_clock.index,len(self.formation_clock.segments)-1)
            segment=self.formation_clock.segments[index]
            self.probes.select(segment,self.sim_clock.us/1e6)
            for u,command in self.probes.commands.items():
                self.probe_publishers[u].publish(String(data=json.dumps(command)))
            return self.probes.admit(self.observations,self.mission_sim_us/1e6)
        def receive(self,v,k,msg):
            receipt=time.monotonic()
            previous=v.get(k+'_packet_rx')
            v[k+'_packet_gap_s']=receipt-previous if previous is not None else None
            v[k+'_packet_rx']=receipt
            if k=='pos':
                if not v['sample_gate'].accept(int(msg.timestamp_sample)):return
                v['pos_sim_us']=self.sim_clock.us
            elif k in ('status','landed'):
                if not v[k+'_gate'].accept(int(msg.timestamp)):return
                v[k+'_sim_us']=self.sim_clock.us
            v[k]=msg.landed if k=='landed' else msg;v['land_rx' if k=='landed' else k+'_rx']=time.monotonic()
        def fresh(self,v):
            report=freshness(v,time.monotonic(),self.sim_clock.us,self.sim_clock.ready(time.monotonic()))
            v['freshness_checked']=report
            return report['ok']
        def freshness_snapshot(self,now):
            return dict(stage=self.stage,phase_time_s=self.phase_time,elapsed_wall_s=now-self.begin,
                sim_us=self.sim_clock.us,clock_age_s=now-self.sim_clock.received if self.sim_clock.received is not None else None,
                tick_gap_s=self.tick_gap_s,record_duration_s=self.record_duration_s,
                vehicles={v['config']['id']:dict(freshness(v,now,self.sim_clock.us,self.sim_clock.ready(now)),
                    last_guard_evaluation=v.get('freshness_checked'),
                    position_packet_age_s=now-v['pos_packet_rx'] if 'pos_packet_rx' in v else None,
                    position_packet_gap_s=v.get('pos_packet_gap_s'),status_packet_gap_s=v.get('status_packet_gap_s'),
                    sample_gate=v['sample_gate'].summary(),status_gate=v['status_gate'].summary(),landed_gate=v['landed_gate'].summary()) for v in self.vehicles})
        def world(self,v):
            p=v['pos'];return to_world((p.x,p.y,p.z),v['config']['world_enu_m'],v['origin'])
        def positions(self):return {v['config']['id']:self.world(v) for v in self.vehicles}
        def transition(self,stage):
            self.stage=stage;self.phase_time=0.;self.since=time.monotonic();self.reached=None
            if stage=='MISSION':self.mission_sim_us=self.sim_clock.us
            if stage=='ARM':self.arm_sim_us=self.sim_clock.us
            print(stage,flush=True)
        def commands(self,cmd,p1=0.,p2=0.):
            if not self.context.ok():raise RuntimeError('ROS context unavailable; recovery unconfirmed')
            for v in self.vehicles:
                m=VehicleCommand();m.timestamp=self.get_clock().now().nanoseconds//1000;m.command=cmd;m.param1=float(p1);m.param2=float(p2)
                m.target_system=v['config']['system_id'];m.target_component=1;m.source_system=245;m.source_component=1;m.from_external=True
                v['pubs']['cmd'].publish(m)
            self.last_command=time.monotonic()
        def stream(self):
            for v in self.vehicles:
                if v['command'] is None:continue
                if self.stage=='LAND' and v['status'] and v['status'].nav_state!=14:continue
                m=OffboardControlMode();m.timestamp=self.get_clock().now().nanoseconds//1000;m.position=True;v['pubs']['mode'].publish(m)
                p=TrajectorySetpoint();p.timestamp=m.timestamp;p.position=list(map(float,to_local(v['command'],v['config']['world_enu_m'],v['origin'])))
                p.velocity=vector_to_ned(self.guard.velocity[v['config']['id']]) if self.stage=='MISSION' else [math.nan]*3;p.acceleration=[math.nan]*3;p.jerk=[math.nan]*3;p.yaw=math.nan;p.yawspeed=math.nan;v['pubs']['point'].publish(p)
        def land(self,reason=None):
            if reason:self.failure=self.failure or reason;print('Group LAND:',reason,flush=True)
            if self.stage!='LAND':self.transition('LAND')
            self.commands(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        def fail(self,reason):
            self.failure=self.failure or reason
            if self.owned:self.land(reason)
            else:self.transition('FAILED');print(reason,flush=True)
        def distances(self,positions):
            return [math.dist(positions[a],positions[b]) for a,b in pairs]
        def record(self):
            master=self.vehicles[0]['pos']
            if master is None or self.sim_clock.us is None or self.sim_clock.us==self.last_record:return
            self.last_record=self.sim_clock.us
            valid=all(v['origin'] is not None and v['pos'] is not None for v in self.vehicles)
            positions=self.positions() if valid else None
            ds=self.distances(positions) if positions else []
            if valid and ds:self.closest=min(self.closest,*ds)
            csv_distances=ds if len(ds)==3 else [math.nan]*3
            for v in self.vehicles:
                p=v['pos'];s=v['status']
                if p is None:continue
                xyz=positions[v['config']['id']] if positions else (math.nan,)*3
                cmd=v['command'] or (math.nan,)*3
                self.writer.writerow([time.monotonic()-self.begin,self.sim_clock.us,v['config']['id'],p.timestamp_sample,self.stage,self.phase_time,
                    *xyz,*cmd,math.dist(xyz,cmd),s.arming_state if s else -1,s.nav_state if s else -1,time.monotonic()-v['pos_rx'],
                    p.xy_reset_counter,p.z_reset_counter,*csv_distances,master.timestamp_sample,time.monotonic()-self.sim_clock.received,v['sample_gate'].domain,p.vy,p.vx,-p.vz,
                    (self.sim_clock.us-self.mission_sim_us)/1e6 if self.mission_sim_us is not None else 0.,
                    (self.sim_clock.us-self.arm_sim_us)/1e6 if self.arm_sim_us is not None else 0.,
                    sum(x['completed'] for x in self.formation_clock.ledger.values())])
            self.csv_file.flush()
        def monitor(self):
            positions=self.positions();ds=self.distances(positions)
            policy=plan.get('safety_policy',{})
            mission=bool(plan.get('geometric') and self.stage=='MISSION')
            emergency=policy.get('mission_abort_separation_m',4.) if mission else 4.
            warning_distance=policy.get('prediction_warning_m',6.) if mission else 6.
            tracking_abort=policy.get('tracking_abort_m',4.) if mission else 4.
            tracking_pause=policy.get('tracking_pause_m',2.) if mission else 2.
            if ds and min(ds)<emergency:self.fail(f'Measured estimated separation <{emergency:g} m');return False
            if self.previous_positions:
                for i,a in enumerate(ids):
                    for b in ids[i+1:]:
                        d=min_segment_distance(self.previous_positions[a],positions[a],self.previous_positions[b],positions[b])
                        self.closest_interval=min(self.closest_interval,d)
                        if d<emergency:self.fail(f'Interpolated separation <{emergency:g} m');return False
            self.previous_positions=positions
            future={}
            for v in self.vehicles:
                u=v['config']['id'];p=v['pos'];xyz=positions[u]
                if not p.v_xy_valid or not p.v_z_valid or not all(math.isfinite(x) for x in (p.vx,p.vy,p.vz)):self.fail('Invalid velocity');return False
                future[u]=(xyz[0]+p.vy,xyz[1]+p.vx,xyz[2]-p.vz)
                if not (plan['flight_bounds_xy_m'][0]<=xyz[0]<=plan['flight_bounds_xy_m'][1] and plan['flight_bounds_xy_m'][2]<=xyz[1]<=plan['flight_bounds_xy_m'][3] and -.5<=xyz[2]<=(22 if plan.get('geometric') else 16)):self.fail('Envelope exceeded');return False
                err=math.dist(xyz,v['command']);self.max_error=max(self.max_error,err)
                if err>tracking_abort:self.fail(f'One UAV tracking error >{tracking_abort:g} m');return False
                if self.stage=='MISSION':
                    gap=xyz[2]-terrain.height(*xyz[:2]);self.min_clearance=min(self.min_clearance,gap)
                    if gap<plan['policy']['min_agl_m']:self.fail('Insufficient terrain clearance');return False
            warning=any(min_segment_distance(positions[a],future[a],positions[b],future[b])<warning_distance for a,b in pairs)
            skew=(max(v['pos_rx'] for v in self.vehicles)-min(v['pos_rx'] for v in self.vehicles))
            service_ok=True
            if mission and not plan.get('formation'):
                from geometric_flight_plan import actual_service_ok
                service_ok=actual_service_ok(plan,terrain,positions,self.phase_time)
            return service_ok and not warning and skew<=.2 and all(math.dist(positions[v['config']['id']],v['command'])<=tracking_pause for v in self.vehicles)
        def tick(self):
            entered=time.monotonic()
            self.tick_gap_s=entered-self.previous_tick if self.previous_tick is not None else None
            self.previous_tick=entered
            self.record();now=time.monotonic();self.record_duration_s=now-entered
            if self.stage in ('DONE','FAILED','INTERRUPTED'):return
            if stop.requested and not self.cancelled:
                self.cancelled=True
                if self.owned:self.land('Operator requested group stop')
                else:self.failure='Stopped before ARM';self.transition('INTERRUPTED');return
            if self.stage=='WAIT':
                if not self.sim_clock.ready(now) or not all(self.fresh(v) and landed_fresh(v,now,self.sim_clock.us,self.sim_clock.ready(now)) for v in self.vehicles):
                    if now-self.since>60:self.fail('Vehicle telemetry/landed timeout')
                    return
                for v in self.vehicles:
                    p=v['pos'];s=v['status']
                    if s.system_id!=v['config']['system_id'] or s.arming_state!=1:self.fail('Wrong system ID or already armed');return
                    if math.hypot(p.x,p.y)>.5 or abs(p.z)>.5:self.fail('Not at initial local origin');return
                if not all(v['status'].pre_flight_checks_pass for v in self.vehicles):
                    if now-self.since>60:self.fail('Preflight failed')
                    return
                check_session(vehicles=args.vehicles)
                for v in self.vehicles:
                    p=v['pos'];v['origin']=(p.x,p.y,p.z);v['resets']=(p.xy_reset_counter,p.z_reset_counter);v['command']=tuple(v['config']['world_enu_m'])
                self.sim_clock.step(now);self.transition('PRESTREAM');return
            if self.stage=='LAND':
                if all(landed_fresh(v,now,self.sim_clock.us,self.sim_clock.ready(now)) for v in self.vehicles):
                    self.landing_confirmed=True;self.transition('INTERRUPTED' if self.cancelled else 'FAILED' if self.failure else 'DONE');return
                if now-self.since>80:self.failure=self.failure or 'Group landing unconfirmed';self.transition('FAILED');return
                self.stream()
                if now-self.last_command>1:self.commands(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                return
            if not all(self.fresh(v) for v in self.vehicles):
                self.freshness_failure=self.freshness_snapshot(time.monotonic())
                print('FRESHNESS FAILURE:',json.dumps(self.freshness_failure),flush=True)
                self.fail('Stale fleet telemetry');return
            self.tick_history.append(self.freshness_snapshot(time.monotonic()))
            if any((v['pos'].xy_reset_counter,v['pos'].z_reset_counter)!=v['resets'] for v in self.vehicles):self.fail('One estimator frame reset');return
            if any(v['sample_gate'].fault for v in self.vehicles):self.fail('PX4 source timestamp fault');return
            try:dt=self.sim_clock.step(now)
            except ValueError as exc:self.fail(str(exc));return
            if self.stage in plan['phases']:
                if not all(v['status'].arming_state==2 and v['status'].nav_state==14 for v in self.vehicles):self.fail('One UAV left armed/Offboard');return
                advance=self.monitor()
                if self.stage=='LAND':return
                paths=plan['phases'][self.stage];duration=paths['UAV1'][-1][0]
                if self.stage=='MISSION':
                    try:
                        admitted=self.inputs()
                        points=self.guard.tick(dt,self.positions(),
                            {v['config']['id']:(v['pos'].vy,v['pos'].vx,-v['pos'].vz) for v in self.vehicles},
                            (self.sim_clock.us-self.mission_sim_us)/1e6,(self.sim_clock.us-self.arm_sim_us)/1e6,
                            admitted,allow=advance,age_s=max(now-v['pos_rx'] for v in self.vehicles))
                    except (ValueError,TimeoutError,RuntimeError) as exc:
                        self.fail('Formation supervisor: '+str(exc));return
                    self.phase_time=self.formation_clock.time
                    self.guard_rows.append(dict(sim_s=(self.sim_clock.us-self.mission_sim_us)/1e6,
                        state=self.guard.state, reference_s=self.phase_time, lift_m=self.formation_clock.lift_m,
                        tick_wall_s=now,probe_epoch=self.probes.epoch,
                        admitted_probe_uavs=sorted(admitted),
                        observations=self.observations.copy()))
                    for v in self.vehicles:v['command']=points[v['config']['id']]
                    if self.formation_clock.paused:self.pause_ticks+=1
                elif self.stage!='MISSION' and advance:
                    self.phase_time=min(duration,self.phase_time+dt)
                    for v in self.vehicles:v['command']=position_at(paths[v['config']['id']],self.phase_time)
                else:self.pause_ticks+=1
                # Overall wall watchdog below is independent of reference duration;
                # common pauses must not be mistaken for a short arrival deadline.
            self.stream()
            if self.stage=='PRESTREAM':
                self.prestream+=1
                if self.prestream>=40:self.commands(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1,6);self.transition('OFFBOARD')
            elif self.stage=='OFFBOARD':
                if all(v['status'].nav_state==14 for v in self.vehicles):
                    check_session(vehicles=args.vehicles)
                    if not all(v['status'].pre_flight_checks_pass for v in self.vehicles):self.fail('Preflight changed');return
                    self.owned=True;self.commands(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,1);self.transition('ARM')
                elif now-self.since>12:self.fail('Offboard timeout')
                elif now-self.last_command>1:self.commands(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,1,6)
            elif self.stage=='ARM':
                if all(v['status'].arming_state==2 for v in self.vehicles):self.transition('CLIMB')
                elif now-self.since>12:self.fail('Partial ARM timeout')
            elif self.stage in plan['phases']:
                paths=plan['phases'][self.stage];finished=(self.formation_clock.done and self.guard.state=='TRACK') if self.stage=='MISSION' else self.phase_time>=paths['UAV1'][-1][0]
                targets=self.formation_clock.targets() if self.stage=='MISSION' else {u:paths[u][-1][1:] for u in ids}
                arrived=all(math.dist(self.world(v),targets[v['config']['id']])<.3 for v in self.vehicles)
                if finished and arrived:
                    if self.reached is None:self.reached=now
                    if now-self.reached>=3:
                        phases=list(plan['phases']);i=phases.index(self.stage)+1
                        if i==len(phases):self.land()
                        else:self.transition(phases[i])
                else:self.reached=None
                if now-self.since>1200:self.fail('Phase timeout')
        def close(self):
            self.record();self.csv_file.close()
            diagnostics_path=self.summary_path.with_name(self.summary_path.name.replace('_summary.json','_freshness.json'))
            diagnostics_path.write_text(json.dumps(dict(failure=self.freshness_failure,recent_ticks=list(self.tick_history)),indent=2)+'\n')
            data=dict(probe_events=self.probes.events,lidar_receipts=self.lidar_receipts,guard_events=self.guard.events, guard_rows=self.guard_rows,
                warning_source='live_gazebo_lidar',
                freshness_policy=FRESHNESS_POLICY,controller_watchdog_version='v17_dual_clock',freshness_diagnostics=str(diagnostics_path),freshness_failure=self.freshness_failure,disturbance_hold_uav2=args.hold_uav2,timebase=self.sim_clock.summary(),timestamp_admission={v['config']['id']:v['sample_gate'].summary() for v in self.vehicles},formation_feedback=self.formation_clock.summary(),result=self.stage,failure=self.failure,execute=args.execute,csv=str(self.csv_path),interrupted=self.cancelled,
                arm_command_sent=self.owned,landing_confirmed=self.landing_confirmed,interrupt_count=stop.count,
                final_states={v['config']['id']:dict(arming_state=int(v['status'].arming_state) if v['status'] else None,landed=v['landed']) for v in self.vehicles},
                min_sampled_separation_m=self.closest if math.isfinite(self.closest) else None,
                min_interpolated_separation_m=self.closest_interval if math.isfinite(self.closest_interval) else None,
                max_tracking_error_m=self.max_error,min_cruise_clearance_m=self.min_clearance if math.isfinite(self.min_clearance) else None,
                pause_ticks=self.pause_ticks,plan=plan,note='master_sample_us is Gazebo simulation time; px4_sample_us and master_raw_sample_us are raw XRCE diagnostics. Position freshness/skew use local wall receive time; low-rate status/landed require simulation receive age<1s AND wall age<3s. Clock receipt/progress watchdog0.5wall s. Not source acquisition synchronization. Spawn-anchored estimates, not ground truth or formal safety proof')
            self.summary_path.write_text(json.dumps(data,indent=2)+'\n');print('Summary:',self.summary_path,flush=True)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:
        node=Swarm()
        if args.startup_check:
            assert all(not v['pubs'] for v in node.vehicles);node.stage='STARTUP_CHECK';print('STARTUP CHECK PASS: no control publishers',flush=True)
        while rclpy.ok() and node.stage not in ('DONE','FAILED','INTERRUPTED','STARTUP_CHECK'):rclpy.spin_once(node,timeout_sec=.1)
    except Exception as exc:
        if node is None:raise
        try:
            if node.owned and rclpy.ok():
                node.land('Controller exception: '+str(exc))
                while rclpy.ok() and node.stage not in ('DONE','FAILED','INTERRUPTED'):rclpy.spin_once(node,timeout_sec=.1)
            else:node.failure=str(exc);node.stage='FAILED'
        except Exception as recovery:node.failure=f'{exc}; recovery: {recovery}';node.stage='FAILED'
    finally:
        try:
            if node:
                if node.stage not in ('DONE','FAILED','INTERRUPTED','STARTUP_CHECK'):node.failure='ROS context lost; landing unconfirmed';node.stage='FAILED'
                try:node.close()
                finally:node.destroy_node()
        finally:try_shutdown();restore_handlers(previous)
    return 130 if node and node.stage=='INTERRUPTED' else 0 if node and node.stage in ('DONE','STARTUP_CHECK') else 1


if __name__=='__main__':raise SystemExit(main())
