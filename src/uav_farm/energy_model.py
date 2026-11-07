"""Uncalibrated electrical-energy proxy; not a battery/SOC or flight dynamics model."""
import math
import numpy as np
from baseline_planner import IDS, poses


class EnergyModel:
    def __init__(self, config, quadrature_order=32):
        self.config = dict(config)
        keys = ('mass_kg', 'rotor_radius_m', 'rotor_count', 'thrust_constant_n_per_rad_s2',
                'moment_to_thrust_ratio_m', 'air_density_kg_m3', 'gravity_m_s2',
                'motor_efficiency', 'climb_efficiency', 'kinetic_efficiency')
        if any(not math.isfinite(config[k]) or config[k] <= 0 for k in keys):
            raise ValueError('Positive finite energy parameters required')
        if any(config[k] > 1 for k in ('motor_efficiency', 'climb_efficiency', 'kinetic_efficiency')):
            raise ValueError('Efficiency cannot exceed one')
        if not 0 < config['hover_profile_fraction'] < 1:
            raise ValueError('Hover power fraction must lie in (0,1)')
        for key in ('parasite_coefficient_w_per_m_s_cubed', 'avionics_w', 'radio_active_w'):
            if not math.isfinite(config[key]) or config[key] < 0:
                raise ValueError('Nonnegative finite power parameters required')
        budget = config['mission_energy_limit_wh_per_uav']
        if budget is not None and (not math.isfinite(budget) or budget <= 0):
            raise ValueError('Positive mission energy budget required')
        if not isinstance(quadrature_order, int) or quadrature_order < 8:
            raise ValueError('Quadrature order must be at least eight')
        c = self.config
        self.weight = c['mass_kg']*c['gravity_m_s2']
        self.area = c['rotor_count']*math.pi*c['rotor_radius_m']**2
        self.omega_hover = math.sqrt(self.weight/(c['rotor_count']*c['thrust_constant_n_per_rad_s2']))
        self.hover_w = self.weight*c['moment_to_thrust_ratio_m']*self.omega_hover/c['motor_efficiency']
        self.profile_w = c['hover_profile_fraction']*self.hover_w
        self.induced_w = self.hover_w-self.profile_w
        self.v0 = math.sqrt(self.weight/(2*c['air_density_kg_m3']*self.area))
        self.tip_speed = self.omega_hover*c['rotor_radius_m']
        nodes, weights = np.polynomial.legendre.leggauss(quadrature_order)
        self.u = (nodes+1)/2
        self.weights = weights/2
        self.progress_derivative = 30*self.u**2-60*self.u**3+30*self.u**4

    def level_power(self, speed):
        if not math.isfinite(speed) or speed < 0:
            raise ValueError('Finite nonnegative speed required')
        # Eq. (6) of Zeng et al.; rationalized inner expression avoids cancellation.
        x = speed**2/(2*self.v0**2)
        induced_factor = 1/math.sqrt(math.sqrt(1+x*x)+x)
        return (self.profile_w*(1+3*speed**2/self.tip_speed**2)
                + self.induced_w*induced_factor
                + self.config['parasite_coefficient_w_per_m_s_cubed']*speed**3)

    def segment(self, a, b, seconds, profile=None):
        if not math.isfinite(seconds) or seconds <= 0:
            raise ValueError('Positive finite duration required')
        c = self.config
        output = {}
        for name, x, y in zip(IDS, poses(a), poses(b)):
            distance = math.dist(x, y)
            horizontal = math.hypot(y[0]-x[0], y[1]-x[1])
            if profile is None:
                speeds = horizontal*self.progress_derivative/seconds
                level_j = seconds*sum(float(w)*self.level_power(float(v)) for w, v in zip(self.weights, speeds))
                peak_rate=1.875/seconds
            else:
                from cruise_profile import kinematics
                if abs(profile['duration_s']-seconds)>1e-7:raise ValueError('Energy/profile duration mismatch')
                cuts=sorted({0.,seconds,*([profile['ramp_s'],profile['ramp_s']+profile['cruise_s']] if profile['kind']!='hold' else [])})
                level_j=0.
                for lo,hi in zip(cuts,cuts[1:]):
                    level_j+=(hi-lo)*sum(float(w)*self.level_power(horizontal*kinematics(profile,lo+float(u)*(hi-lo))[1]) for u,w in zip(self.u,self.weights))
                peak_rate=profile.get('peak_progress_rate_s_inv',0.)
            # Explicit extensions, not equations from the level-flight paper:
            climb_j = self.weight*max(y[2]-x[2], 0)/c['climb_efficiency']
            kinetic_j = .5*c['mass_kg']*(peak_rate*distance)**2/c['kinetic_efficiency']
            components = dict(quasi_steady_j=level_j, positive_climb_work_j=climb_j,
                              positive_kinetic_work_j=kinetic_j, avionics_j=c['avionics_w']*seconds)
            components['total_j'] = sum(components.values())
            output[name] = components
        return output

    def audit(self, mission):
        totals = {u:dict(quasi_steady_j=0., positive_climb_work_j=0., positive_kinetic_work_j=0.,
                         avionics_j=0., radio_j=0.) for u in IDS}
        by_segment = []
        for seg in mission['segments']:
            value = self.segment(seg['start'], seg['end'], seg['duration_s'],seg.get('profile'))
            by_segment.append(dict(start_s=seg['start_s'], end_s=seg['end_s'], energy=value))
            for u in IDS:
                for key in totals[u]:
                    if key in value[u]:
                        totals[u][key] += value[u][key]
        for service in mission['services']:
            totals[service['uav']]['radio_j'] += (service['end_s']-service['start_s'])*self.config['radio_active_w']
        budget = self.config['mission_energy_limit_wh_per_uav']
        for u in IDS:
            totals[u]['total_j'] = sum(totals[u].values())
            totals[u]['total_wh'] = totals[u]['total_j']/3600
        return dict(status='uncalibrated_reference_estimate', per_uav=totals,
                    team_wh=sum(totals[u]['total_wh'] for u in IDS),
                    mission_energy_limit_wh_per_uav=budget,
                    budget_feasible=None if budget is None else all(totals[u]['total_wh'] <= budget+1e-9 for u in IDS),
                    derived=dict(hover_electrical_proxy_w=self.hover_w, induced_velocity_m_s=self.v0,
                                 rotor_tip_speed_m_s=self.tip_speed), segments=by_segment,
                    excludes=['takeoff', 'landing', 'feedback pauses', 'wind', 'battery/SOC dynamics'])
