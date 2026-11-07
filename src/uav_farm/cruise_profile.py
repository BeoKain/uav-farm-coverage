"""Rest-to-rest acceleration / cruise / deceleration with continuous acceleration.

Normalized progress velocity ramps as 3u²-2u³. Its maximum derivative is 1.5,
so ramp duration >= 1.5 * peak_progress_rate / acceleration_limit.
All UAVs share scalar progress; spatial paths and formation checks are unchanged.
"""
import math
from baseline_planner import poses


def make_profile(a,b,policy,seconds=None):
    for key in ('horizontal_speed_m_s','vertical_speed_m_s','acceleration_m_s2'):
        if not math.isfinite(policy[key]) or policy[key]<=0:raise ValueError('Positive motion limits required')
    deltas=[tuple(y-x for x,y in zip(p,q)) for p,q in zip(poses(a),poses(b))]
    length=max(math.sqrt(sum(x*x for x in d)) for d in deltas)
    if length<1e-10:
        if seconds is None or not math.isfinite(seconds) or seconds<=0:raise ValueError('Hold requires a duration')
        return dict(kind='hold',duration_s=seconds)
    horizontal=max(math.hypot(*d[:2]) for d in deltas)
    vertical=max(abs(d[2]) for d in deltas)
    rate=min(policy['horizontal_speed_m_s']/horizontal if horizontal else math.inf,
             policy['vertical_speed_m_s']/vertical if vertical else math.inf)
    normalized_acceleration=policy['acceleration_m_s2']/length
    peak=min(rate,math.sqrt(normalized_acceleration/1.5))
    ramp=1.5*peak/normalized_acceleration
    cruise=max(0.,1/peak-ramp)
    minimum=2*ramp+cruise
    requested=minimum if seconds is None else seconds
    if not math.isfinite(requested) or requested<minimum-1e-8:raise ValueError('Duration violates motion limits')
    stretch=requested/minimum
    return dict(kind='smooth_cruise_v1',duration_s=requested,ramp_s=ramp*stretch,
                cruise_s=cruise*stretch,peak_progress_rate_s_inv=peak/stretch)


def duration(a,b,policy):
    return make_profile(a,b,policy)['duration_s']


def kinematics(profile,t):
    """Return scalar progress, progress rate and acceleration at elapsed t."""
    if not math.isfinite(t):raise ValueError('Finite elapsed time required')
    total=profile['duration_s']
    if profile['kind']=='hold':return 0.,0.,0.
    ramp,cruise,peak=profile['ramp_s'],profile['cruise_s'],profile['peak_progress_rate_s_inv']
    if t<=0:return 0.,0.,0.
    if t>=total:return 1.,0.,0.
    def acceleration_phase(x):
        u=x/ramp
        return (peak*ramp*(u**3-.5*u**4),peak*(3*u*u-2*u**3),peak/ramp*(6*u-6*u*u))
    if t<ramp:return acceleration_phase(t)
    if t<=ramp+cruise:return .5*peak*ramp+peak*(t-ramp),peak,0.
    position,velocity,acceleration=acceleration_phase(total-t)
    return 1-position,velocity,-acceleration


def segment_kinematics(segment,t):
    if 'profile' in segment:return kinematics(segment['profile'],t)
    # Legacy frozen v4/v5 plans retain their original quintic semantics.
    total=segment['duration_s'];u=max(0.,min(1.,t/total))
    return (10*u**3-15*u**4+6*u**5,(30*u*u-60*u**3+30*u**4)/total,
            (60*u-180*u*u+120*u**3)/total**2)


def validate_profile(segment,policy):
    expected=make_profile(segment['start'],segment['end'],policy,segment['duration_s'])
    actual=segment.get('profile')
    if actual is None or set(actual)!=set(expected):raise ValueError('Missing/invalid cruise profile')
    for key,value in expected.items():
        if isinstance(value,str):
            if actual[key]!=value:raise ValueError('Profile kind mismatch')
        elif not math.isfinite(actual[key]) or abs(actual[key]-value)>1e-7*max(1.,abs(value)):
            raise ValueError('Profile descriptor violates motion limits')
