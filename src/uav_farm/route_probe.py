"""Reference-frame probe directions for affine virtual-structure segments."""
import math


def probe_plan(segment, ids, poses):
    directions={}
    for u,a,b in zip(ids,poses(segment['start']),poses(segment['end'])):
        dx,dy=b[0]-a[0],b[1]-a[1]
        norm=math.hypot(dx,dy)
        directions[u]=(dx/norm,dy/norm) if norm>1e-8 else None
    if all(d is None for d in directions.values()):
        mode='stationary' if abs(segment['end'][2]-segment['start'][2])<=1e-8 else 'vertical'
    elif any(d is None for d in directions.values()):
        mode='mixed'
    else:
        mode='directional'
    return dict(mode=mode,directions_enu=directions)


def changed(a,b):
    if a is None:
        return True
    return any((a[u] is None)!=(b[u] is None) or
               (a[u] is not None and b[u] is not None and math.dist(a[u],b[u])>1e-7) for u in b)
