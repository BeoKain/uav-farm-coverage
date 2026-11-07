"""Record native Gazebo model positions for evaluation only."""
import csv
from pathlib import Path
import signal
import sys
from threading import Lock
import time

from gz.transport13 import Node
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene


def main(output, ready):
    node=Node()
    ids={}
    deadline=time.monotonic()+30.
    expected={f'x500_{i}':f'UAV{i+1}' for i in range(3)}
    while time.monotonic()<deadline:
        ok,scene=node.request('/world/farm/scene/info',Empty(),Empty,Scene,1500)
        if ok:
            ids={m.id:expected[m.name] for m in scene.model if m.name in expected}
            if len(ids)==3:
                break
        time.sleep(.1)
    if len(ids)!=3:
        raise RuntimeError('Missing one or more x500 entities')
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop)
    signal.signal(signal.SIGINT,stop)
    topic='/world/farm/dynamic_pose/info'
    lock=Lock()
    with output.open('w',newline='') as f:
        writer=csv.writer(f)
        writer.writerow(('stamp_s','uav','x','y','z','qw','qx','qy','qz'))
        def receive(msg):
            stamp=msg.header.stamp.sec+msg.header.stamp.nsec/1e9
            rows=[(stamp,ids[p.id],p.position.x,p.position.y,p.position.z,
                   p.orientation.w,p.orientation.x,p.orientation.y,p.orientation.z)
                  for p in msg.pose if p.id in ids]
            if len(rows)==3:
                with lock:
                    if not stopping:
                        writer.writerows(rows)
                        f.flush()
                        if not ready.exists():
                            ready.touch()
        if not node.subscribe(Pose_V,topic,receive):
            raise RuntimeError('Gazebo pose subscribe failed')
        try:
            while not stopping:
                time.sleep(.05)
        finally:
            node.unsubscribe(topic)
            with lock:
                pass


if __name__=='__main__':
    main(*map(Path,sys.argv[1:]))
