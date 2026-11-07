"""Local ENU terrain evidence, without a terrain map or flight commands."""
from dataclasses import dataclass
import math
import numpy as np


@dataclass(frozen=True)
class Policy:
    lookahead_m: float = 24.
    half_width_m: float = 2.
    cell_m: float = 2.
    min_clearance_m: float = 5.
    margin_m: float = .75
    max_age_s: float = .6
    min_hits: int = 3
    min_range_m: float = 1.
    max_range_m: float = 30.


def assess(sensor_xyz, position, rotation, direction_en, stamp, now, policy=Policy()):
    """Classify surface samples under a level forward corridor.

    NO_RISE_OBSERVED is surface evidence only, never a free-space certificate.
    Unknown cells stay unknown: no ray extrapolation, filling, or stale map reuse.
    """
    if not (math.isfinite(stamp) and math.isfinite(now) and 0 <= now-stamp < policy.max_age_s):
        return dict(state='UNKNOWN', reason='invalid_or_stale_stamp', cells=[])
    p, r, d = map(lambda x: np.asarray(x, dtype=float), (position, rotation, direction_en))
    if (p.shape != (3,) or r.shape != (3, 3) or d.shape != (2,)
            or not all(np.isfinite(x).all() for x in (p, r, d))
            or not np.allclose(r.T @ r, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(r), 1.) or np.linalg.norm(d) < 1e-9):
        return dict(state='UNKNOWN', reason='invalid_pose_or_direction', cells=[])
    d = d / np.linalg.norm(d)
    points = np.asarray(sensor_xyz, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        return dict(state='UNKNOWN', reason='invalid_cloud', cells=[])
    # Keep the exact projection and cell edges, but crop the entire
    # corridor before repeated masks and distinct-return counts.
    with np.errstate(invalid='ignore', over='ignore'):
        relative = (points + [.35, 0., .05]) @ r.T
        forward = relative[:, :2] @ d
        lateral = relative[:, :2] @ [-d[1], d[0]]
    keep = ((forward >= 0.) & (forward < policy.lookahead_m)
            & (lateral >= -policy.half_width_m) & (lateral < policy.half_width_m)
            & np.isfinite(points).all(axis=1))
    ranges = np.linalg.norm(points[keep], axis=1)
    in_range = (ranges >= policy.min_range_m) & (ranges <= policy.max_range_m)
    indices = np.flatnonzero(keep)[in_range]
    relative = relative[indices]
    forward, lateral, z = forward[indices], lateral[indices], relative[:, 2]
    edges = np.arange(0., policy.lookahead_m, policy.cell_m)
    sides = np.arange(-policy.half_width_m, policy.half_width_m, policy.cell_m)
    threshold = -policy.min_clearance_m - policy.margin_m
    cells = []
    for x in edges:
        along = (forward >= x) & (forward < min(x+policy.cell_m, policy.lookahead_m))
        for y in sides:
            inside = along & (lateral >= y) & (lateral < min(y+policy.cell_m, policy.half_width_m))
            high = inside & (z >= threshold)
            n = len(np.unique(relative[inside], axis=0))
            h = len(np.unique(relative[high], axis=0))
            state = ('RISE' if h >= policy.min_hits else
                     'UNKNOWN' if h or n < policy.min_hits else 'LOW_RETURNS')
            cells.append(dict(forward_start_m=float(x), lateral_start_m=float(y),
                              hits=n, high_hits=h, state=state,
                              nearest_high_return_m=float(forward[high].min()) if h else None))
    rises = [c['nearest_high_return_m'] for c in cells if c['state'] == 'RISE']
    state = ('RISE' if rises else 'UNKNOWN' if any(c['state'] == 'UNKNOWN' for c in cells)
             else 'NO_RISE_OBSERVED')
    return dict(state=state, nearest_rise_return_m=min(rises) if rises else None, cells=cells)
