"""Read-only diagnostics matching the pre-existing telemetry guard exactly."""
import math


def freshness(v, now):
    p=v['pos'];s=v['status']
    reasons=[]
    if p is None:reasons.append('missing_position')
    if s is None:reasons.append('missing_status')
    pa=now-v['pos_rx'];sa=now-v['status_rx']
    if not pa<.5:reasons.append('position_age_ge_0.5s')
    if not sa<1:reasons.append('status_age_ge_1s')
    if p is not None:
        if not p.xy_valid:reasons.append('xy_invalid')
        if not p.z_valid:reasons.append('z_invalid')
        if not all(math.isfinite(x) for x in (p.x,p.y,p.z)):reasons.append('position_nonfinite')
    return dict(ok=not reasons,reasons=reasons,evaluated_wall=now,
                position_age_s=pa,status_age_s=sa,
                xy_valid=bool(p.xy_valid) if p else None,z_valid=bool(p.z_valid) if p else None,
                position_sample_us=int(p.timestamp_sample) if p else None,
                status_timestamp_us=int(s.timestamp) if s else None,
                arming_state=int(s.arming_state) if s else None,nav_state=int(s.nav_state) if s else None)
