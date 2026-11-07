"""Bounded westward formation relocation before the S18 geometric service."""
from baseline_planner import poses
from cruise_profile import make_profile
from terrain_links import evaluate


def relocate_service(supervisor):
    """Change only the unstarted final stop; return evidence or None if usable."""
    s=supervisor
    if s.index>=len(s.segments) or s.segments[s.index]['kind']!='SERVICE':
        return None
    segment=s.segments[s.index]
    task=next(task for task in s.services if task['sensor']=='S18')
    if (abs(task['start_s']-segment['start_s'])>1e-8
            or s.local or s.ledger['S18']['credited_s'] or s.lift_m<=0.):
        return None
    sensor=s.sensors['S18']['world_enu_m']
    nominal=s.targets()['UAV2']
    max_range=s.p['collection_range_m']
    clearance=s.p['min_agl_m']
    old=evaluate(s.terrain,nominal,sensor,max_range,clearance)
    if old['geometry_eligible']:
        return None
    state=segment['start']
    x0,x1,y0,y1=s.plan['flight_bounds_xy_m']
    for step in (.5,1.,1.5,2.):
        candidate=[state[0]-step,*state[1:]]
        fleet=poses(candidate)
        lifted=[(x,y,z+s.lift_m) for x,y,z in fleet]
        if not all(x0+.35<=x<=x1-.35 and y0+.35<=y<=y1-.35
                   and z<=s.plan['flight_ceiling_m'] for x,y,z in lifted):
            continue
        if not all(min(k['vertical_clearance_m'] for k in s.terrain.line_profile(a,b))>=clearance+.35
                   for a,b in zip(s.targets().values(),lifted)):
            continue
        link=evaluate(s.terrain,lifted[1],sensor,max_range-.5,clearance)
        if not link['geometry_eligible']:
            continue
        profile=make_profile(state,candidate,s.p)
        start=segment['start_s']
        transit_time=profile['duration_s']
        connector=dict(kind='TRANSIT',start=state,end=candidate,start_s=start,
                       end_s=start+transit_time,duration_s=transit_time,profile=profile)
        suffix=[]
        task_updates=[]
        cursor=connector['end_s']
        previous=candidate
        for i,old_segment in enumerate(s.segments[s.index:]):
            destination=candidate if i==0 else old_segment['end']
            if old_segment['kind']=='TRANSIT':
                next_profile=make_profile(previous,destination,s.p)
            else:
                next_profile=make_profile(previous,destination,s.p,old_segment['duration_s'])
            new_duration=next_profile['duration_s']
            suffix.append(dict(kind=old_segment['kind'],start=previous,end=destination,
                               start_s=cursor,end_s=cursor+new_duration,duration_s=new_duration,
                               profile=next_profile))
            if old_segment['kind']=='SERVICE':
                for job in s.services:
                    if old_segment['start_s']-1e-8<=job['start_s'] and job['end_s']<=old_segment['end_s']+1e-8:
                        task_updates.append((job,cursor+job['start_s']-old_segment['start_s'],
                                             cursor+job['end_s']-old_segment['start_s']))
            cursor+=new_duration
            previous=destination
        old_duration=s.duration
        for job,a,b in task_updates:
            job.update(start_s=a,end_s=b)
        s.segments[s.index:]=[connector,*suffix]
        s.duration=cursor
        s.settled=0.
        return dict(sensor='S18',lift_m=s.lift_m,from_position=nominal,to_position=lifted[1],
                    old_geometry_state=old['state'],old_distance_m=old['distance_3d_m'],
                    new_distance_m=link['distance_3d_m'],shift_m=step,reference_s=s.time,
                    suffix_delta_s=cursor-old_duration, connector_s=transit_time)
    raise RuntimeError('No eligible bounded alternative for S18')
