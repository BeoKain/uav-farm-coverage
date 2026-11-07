"""PointCloud2 decoding and evaluation transforms, no ROS/control dependencies."""
import numpy as np

def xyz_from_cloud(msg):
    if msg.height<=0 or msg.width<=0 or msg.point_step<=0 or msg.row_step<msg.width*msg.point_step:
        raise ValueError('Invalid PointCloud2 dimensions/stride')
    if len(msg.data)<msg.height*msg.row_step:raise ValueError('Truncated PointCloud2 payload')
    fields={f.name:f for f in msg.fields};columns=[]
    for name in ('x','y','z'):
        f=fields.get(name)
        if f is None or f.count!=1 or f.datatype not in (7,8):raise ValueError('Expected scalar float XYZ fields')
        kind=('>' if msg.is_bigendian else '<')+('f4' if f.datatype==7 else 'f8');dt=np.dtype(kind)
        if f.offset<0 or f.offset+dt.itemsize>msg.point_step:raise ValueError('XYZ outside point record')
        columns.append(np.ndarray((msg.height,msg.width),dtype=dt,buffer=bytes(msg.data),offset=f.offset,strides=(msg.row_step,msg.point_step)).reshape(-1))
    return np.column_stack(columns).astype(float)

def pose_matrix(pose):
    q=pose.orientation;quat=np.array([q.w,q.x,q.y,q.z],float);norm=np.linalg.norm(quat)
    if not np.isfinite(norm) or norm<1e-8:raise ValueError('Invalid pose quaternion')
    w,x,y,z=quat/norm
    r=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    t=np.eye(4);t[:3,:3]=r;t[:3,3]=[pose.position.x,pose.position.y,pose.position.z];return t

def transform(points,matrix):return np.asarray(points)@matrix[:3,:3].T+matrix[:3,3]

def terrain_hits(world,terrain):
    eligible=[];errors=[]
    for x,y,z in world:
        if terrain.xs[0]<=x<=terrain.xs[-1] and terrain.ys[0]<=y<=terrain.ys[-1]:
            h=terrain.height(float(x),float(y));errors.append(z-h)
            eligible.append(h>1. and z>1. and abs(z-h)<=.20)
        else:errors.append(float('nan'));eligible.append(False)
    return np.array(eligible),np.array(errors)

def xyz_from_scan(scan):
    if scan.count<=1 or scan.vertical_count<=1:raise ValueError('Expected full3D scan')
    if len(scan.ranges)!=scan.count*scan.vertical_count:raise ValueError('Native range array size mismatch')
    ranges=np.array(scan.ranges).reshape(scan.vertical_count,scan.count)
    h=scan.angle_min+np.arange(scan.count)*scan.angle_step
    v=scan.vertical_angle_min+np.arange(scan.vertical_count)*scan.vertical_angle_step
    hh,vv=np.meshgrid(h,v)
    with np.errstate(invalid='ignore'):
        return np.stack((ranges*np.cos(vv)*np.cos(hh),ranges*np.cos(vv)*np.sin(hh),ranges*np.sin(vv)),axis=-1).reshape(-1,3)
