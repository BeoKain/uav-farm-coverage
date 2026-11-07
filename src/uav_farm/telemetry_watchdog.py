"""SITL receive watchdog. Simulation receipt age is not source acquisition age.

Slow PX4 publication follows simulation time. Never replace the independent wall
watchdogs for position, clock, or complete loss of low-rate telemetry.
"""
import math
from freshness_diagnostics import freshness as legacy_freshness

POLICY = dict(status_sim_s=1., low_rate_wall_s=3., position_wall_s=.5,
              clock_wall_s=.5, clock_progress_wall_s=.5)


def low_rate(v, key, now, sim_us, clock_ready):
    wall_key = 'land_rx' if key == 'landed' else key+'_rx'
    receipt = v.get(wall_key)
    stamp = v.get(key+'_sim_us')
    wall = now-receipt if receipt is not None else None
    sim = (sim_us-stamp)/1e6 if sim_us is not None and stamp is not None else None
    reasons = []
    if v.get(key) is None: reasons.append('missing_'+key)
    if not clock_ready: reasons.append('clock_unavailable_stale_or_stalled')
    if wall is None or not math.isfinite(wall) or not 0 <= wall < POLICY['low_rate_wall_s']:
        reasons.append(key+'_wall_age_ge_3s_or_invalid')
    if sim is None or not math.isfinite(sim) or not 0 <= sim < POLICY['status_sim_s']:
        reasons.append(key+'_sim_age_ge_1s_or_invalid')
    gate = v.get(key+'_gate')
    if gate is not None and gate.fault: reasons.append(key+'_timestamp_fault')
    return dict(ok=not reasons, reasons=reasons, wall_age_s=wall, sim_age_s=sim)


def freshness(v, now, sim_us, clock_ready):
    report = legacy_freshness(v, now)
    # Replace exactly the old status wall-age criterion, retain position checks.
    reasons = [r for r in report['reasons'] if r != 'status_age_ge_1s']
    if not math.isfinite(report['position_age_s']) or report['position_age_s'] < 0:
        reasons.append('position_receive_time_invalid')
    status = low_rate(v, 'status', now, sim_us, clock_ready)
    reasons.extend(r for r in status['reasons'] if r not in reasons)
    report.update(ok=not reasons, reasons=reasons, status_sim_age_s=status['sim_age_s'],
                  policy=dict(POLICY), age_semantics='since accepted receipt, not source acquisition')
    return report


def landed_fresh(v, now, sim_us, clock_ready):
    return (low_rate(v, 'status', now, sim_us, clock_ready)['ok'] and
            low_rate(v, 'landed', now, sim_us, clock_ready)['ok'] and
            v['status'].arming_state == 1 and v['landed'] is True)
