"""Terrain-only geometric link screening; this is NOT a radio propagation model."""
import csv
import math
from pathlib import Path


class TerrainGrid:
    def __init__(self,xs,ys,heights):
        self.xs=xs;self.ys=ys;self.heights=heights
        if len(xs)<2 or len(ys)<2:raise ValueError('Terrain needs at least 2 x 2 vertices')
        self.dx=xs[1]-xs[0];self.dy=ys[1]-ys[0]
        if self.dx<=0 or self.dy<=0:raise ValueError('Grid step must be positive')
        if any(abs((b-a)-self.dx)>1e-7 for a,b in zip(xs,xs[1:])) or any(abs((b-a)-self.dy)>1e-7 for a,b in zip(ys,ys[1:])):
            raise ValueError('Expected uniform rectangular terrain grid')
        if len(heights)!=len(xs)*len(ys):raise ValueError('Missing terrain vertices')
        for x in xs:
            for y in ys:
                if (x,y) not in heights or not math.isfinite(heights[x,y]):
                    raise ValueError('Missing or invalid terrain vertex')

    @classmethod
    def from_csv(cls,path):
        heights={}
        with Path(path).open() as f:
            for row in csv.DictReader(f):
                x,y,z=[float(row[k]) for k in ('east_m','north_m','up_m')]
                if not all(math.isfinite(v) for v in (x,y,z)):raise ValueError('Non-finite terrain coordinate')
                if (x,y) in heights:raise ValueError('Duplicate terrain vertex')
                heights[x,y]=z
        return cls(sorted({x for x,y in heights}),sorted({y for x,y in heights}),heights)

    def height(self,x,y):
        if not math.isfinite(x) or not math.isfinite(y):raise ValueError('Non-finite query')
        if not self.xs[0]-1e-8<=x<=self.xs[-1]+1e-8 or not self.ys[0]-1e-8<=y<=self.ys[-1]+1e-8:
            raise ValueError('Query outside terrain')
        x=min(self.xs[-1],max(self.xs[0],x));y=min(self.ys[-1],max(self.ys[0],y))
        i=min(len(self.xs)-2,math.floor((x-self.xs[0])/self.dx))
        j=min(len(self.ys)-2,math.floor((y-self.ys[0])/self.dy))
        x0,x1=self.xs[i:i+2];y0,y1=self.ys[j:j+2]
        u=(x-x0)/self.dx;v=(y-y0)/self.dy
        h00=self.heights[x0,y0];h10=self.heights[x1,y0]
        h01=self.heights[x0,y1];h11=self.heights[x1,y1]
        if u+v<=1:return h00+u*(h10-h00)+v*(h01-h00)
        return h11+(1-u)*(h01-h11)+(1-v)*(h10-h11)

    def line_profile(self,a,b):
        """Exact clearance extrema for the piecewise-linear exported mesh.

        Cut at every X/Y grid boundary and normalized X+Y integer diagonal.
        On each resulting interval terrain and ray elevation are affine, so
        the minimum vertical clearance occurs at one of the interval endpoints.
        Returned clearance is vertical separation, not Euclidean obstacle distance.
        """
        if len(a)!=3 or len(b)!=3 or not all(math.isfinite(v) for v in (*a,*b)):
            raise ValueError('Expected finite 3D endpoints')
        self.height(a[0],a[1]);self.height(b[0],b[1])
        ts={0.,1.}
        def cuts(start,end,levels):
            if abs(end-start)<1e-12:return
            for level in levels:
                t=(level-start)/(end-start)
                if 0<t<1:ts.add(t)
        cuts(a[0],b[0],self.xs);cuts(a[1],b[1],self.ys)
        a_diag=(a[0]-self.xs[0])/self.dx+(a[1]-self.ys[0])/self.dy
        b_diag=(b[0]-self.xs[0])/self.dx+(b[1]-self.ys[0])/self.dy
        cuts(a_diag,b_diag,range(len(self.xs)+len(self.ys)-1))
        profile=[]
        for t in sorted(ts):
            x,y,z=[a[i]+t*(b[i]-a[i]) for i in range(3)]
            ground=self.height(x,y)
            profile.append(dict(t=t,x_m=x,y_m=y,ray_z_m=z,ground_z_m=ground,
                                vertical_clearance_m=z-ground))
        return profile


def evaluate(terrain,uav,sensor,max_distance=60.,minimum_uav_agl=2.,los_margin=0.):
    if max_distance<=0 or minimum_uav_agl<0 or los_margin<0:
        raise ValueError('Invalid screening thresholds')
    profile=terrain.line_profile(uav,sensor)
    agl=uav[2]-terrain.height(uav[0],uav[1])
    minimum=min(profile,key=lambda p:p['vertical_clearance_m'])
    distance=math.dist(uav,sensor)
    valid=agl>=minimum_uav_agl-1e-9 and profile[-1]['vertical_clearance_m']>0
    los=minimum['vertical_clearance_m']>los_margin+1e-9
    in_range=distance<=max_distance+1e-9
    state='invalid_endpoint' if not valid else 'terrain_blocked' if not los else 'out_of_range' if not in_range else 'geometry_eligible'
    return dict(state=state,uav_agl_m=agl,distance_3d_m=distance,los_clear=los,
        within_distance_limit=in_range,geometry_eligible=valid and los and in_range,
        min_vertical_clearance_m=minimum['vertical_clearance_m'],
        critical_point_world_m=[minimum['x_m'],minimum['y_m'],minimum['ground_z_m']],
        profile=profile)
