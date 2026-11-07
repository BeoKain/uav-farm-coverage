"""Direction epochs keep earlier scans out of a newly selected corridor."""
import math

from route_probe import changed, probe_plan


def parse_command(command):
    epoch=command['epoch']
    stamp=float(command['issued_sim_s'])
    direction=tuple(float(v) for v in command['direction_enu'])
    if (type(epoch) is not int or epoch<1 or not math.isfinite(stamp)
            or len(direction)!=2 or not all(map(math.isfinite,direction))
            or not math.isclose(math.hypot(*direction),1.,abs_tol=1e-7)):
        raise ValueError('Invalid direction command')
    return dict(epoch=epoch,issued_sim_s=stamp,direction_enu=direction)


class ProbeReceiver:
    def __init__(self):
        self.command=None

    def receive(self,payload):
        command=parse_command(payload)
        if self.command is None or command['epoch']>self.command['epoch']:
            self.command=command

    def for_scan(self,stamp):
        if self.command is None or stamp<self.command['issued_sim_s']:
            return None
        return self.command


class ProbeGate:
    def __init__(self,ids,poses):
        self.ids=ids
        self.poses=poses
        self.directions=None
        self.epoch=0
        self.commands={}
        self.events=[]

    def select(self,segment,stamp):
        ref=probe_plan(segment,self.ids,self.poses)
        if ref['mode']=='stationary' and self.commands:
            return False
        # During an ascent with fixed XY, continue observing the incoming corridor.
        # The caller still applies the same scan freshness and fleet guard.
        if (ref['mode']=='vertical' and self.commands
                and segment['end'][2]>segment['start'][2]):
            return False
        if ref['mode']!='directional':
            raise ValueError(f"Unsupported probe segment mode: {ref['mode']}")
        directions=ref['directions_enu']
        if not changed(self.directions,directions):
            return False
        self.epoch+=1
        self.directions=directions
        self.commands={u:dict(epoch=self.epoch,issued_sim_s=stamp,direction_enu=directions[u]) for u in self.ids}
        self.events.append(dict(epoch=self.epoch,issued_sim_s=stamp,segment_start_s=segment['start_s'],
                                directions_enu=directions))
        return True

    def admit(self,observations,mission_epoch):
        result={}
        for u,command in self.commands.items():
            d=observations.get(u)
            if d is None or d.get('probe_epoch')!=command['epoch']:
                continue
            if tuple(d.get('direction_enu',()))!=command['direction_enu']:
                continue
            if not math.isfinite(d['stamp_s']) or d['stamp_s']<command['issued_sim_s']:
                continue
            result[u]=dict(d,stamp_s=round((d['stamp_s']-mission_epoch)*1e6)/1e6)
        return result
