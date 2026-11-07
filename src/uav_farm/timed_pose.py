"""Timestamp interpolation without extrapolation; no ROS/control dependencies."""
import bisect,math
import numpy as np
from types import SimpleNamespace as O
from lidar_geometry import pose_matrix

def unit(q):
 q=np.asarray(q,float);n=np.linalg.norm(q)
 if q.shape!=(4,) or not np.isfinite(q).all() or n<1e-9:raise ValueError('Invalid quaternion')
 return q/n

def slerp(a,b,f):
 if not 0<=f<=1:raise ValueError('Invalid interpolation fraction')
 a,b=unit(a),unit(b);dot=float(np.dot(a,b))
 if dot<0:b=-b;dot=-dot
 if dot>.9995:return unit(a+f*(b-a))
 angle=math.acos(np.clip(dot,-1,1));return (math.sin((1-f)*angle)*a+math.sin(f*angle)*b)/math.sin(angle)

class PoseSeries:
 def __init__(self,records):
  # Last sample at an equal time wins; never bridge an unbounded data gap.
  records={float(t):(np.asarray(p,float),unit(q)) for t,p,q in records}
  if any(not math.isfinite(t) or p.shape!=(3,) or not np.isfinite(p).all() for t,(p,q) in records.items()):raise ValueError('Invalid pose sample')
  self.times=sorted(records);self.values=[records[t] for t in self.times]
 def at(self,t,max_gap=.12):
  i=bisect.bisect_left(self.times,t)
  if i<len(self.times) and abs(self.times[i]-t)<1e-8:return *self.values[i],0.
  if i==0 or i==len(self.times):raise ValueError('Pose extrapolation forbidden')
  a,b=self.times[i-1:i+1];gap=b-a
  if gap>max_gap:raise ValueError('Pose bracket gap too large')
  f=(t-a)/gap;p,q=self.values[i-1];r,s=self.values[i]
  return p+f*(r-p),slerp(q,s,f),gap

def quaternion_matrix(q):
 w,x,y,z=unit(q);return pose_matrix(O(position=O(x=0,y=0,z=0),orientation=O(w=w,x=x,y=y,z=z)))[:3,:3]

def px4_rotation(q):
 enu_from_ned=np.array([[0,1,0],[1,0,0],[0,0,-1]])
 frd_from_flu=np.diag([1,-1,-1])
 return enu_from_ned@quaternion_matrix(q)@frd_from_flu
