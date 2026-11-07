"""Read-only ROS 2 collector for the three disarmed SITL vehicles."""
import json
from pathlib import Path
import sys
import time

import numpy as np
import rclpy
from px4_msgs.msg import VehicleStatus
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2

from lidar_geometry import xyz_from_cloud

COUNT = 45


class Collector(Node):
    def __init__(self, spawns):
        super().__init__('fleet_lidar_transport_check_'+spawns[0]['id'].lower())
        self.clock = None
        self.error = None
        self.status = {}
        self.rows = {s['id']: [] for s in spawns}
        self.create_subscription(Clock, '/clock', self.clock_callback, 10)
        status_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        for s in spawns:
            name = s['id']
            self.create_subscription(VehicleStatus, s['namespace']+'/out/vehicle_status_v4',
                                     lambda msg, spawn=s:self.status_callback(msg, spawn), status_qos)
            self.create_subscription(PointCloud2, f'/{name.lower()}/terrain_lidar/points',
                                     lambda msg, spawn=s:self.cloud_callback(msg, spawn), 1)

    def clock_callback(self, msg):
        self.clock = msg.clock.sec+msg.clock.nanosec/1e9

    def status_callback(self, msg, spawn):
        if msg.system_id!=spawn['system_id'] or msg.arming_state!=1:
            self.error = f"Unexpected system ID or arming state for {spawn['id']}"
        self.status[spawn['id']] = dict(system_id=msg.system_id, arming_state=msg.arming_state)

    def cloud_callback(self, msg, spawn):
        received_wall=time.monotonic()
        if self.clock is None:
            return
        name = spawn['id']
        model = f"x500_{spawn['instance']}"
        if msg.header.frame_id.split('/')[0]!=model or 'terrain_lidar_link' not in msg.header.frame_id.split('/'):
            self.error = f'Unexpected frame {msg.header.frame_id!r} on {name}'
            return
        stamp = msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        rows = self.rows[name]
        if rows and stamp<=rows[-1]['stamp_s']:
            return
        points = xyz_from_cloud(msg)
        finite = int(np.isfinite(points).all(axis=1).sum())
        if (msg.width,msg.height)!=(361,95) or finite<1000:
            self.error = f'Unexpected cloud geometry for {name}'
            return
        rows.append(dict(stamp_s=stamp, clock_s=self.clock, wall_s=time.monotonic(),
                         finite=finite, frame=msg.header.frame_id, payload_bytes=len(msg.data),
                         callback_wall_s=received_wall))
        return points


def main(output, spawns):
    rclpy.init()
    node = Collector(spawns)
    try:
        deadline = time.monotonic()+65.
        while time.monotonic()<deadline and not node.error:
            rclpy.spin_once(node,timeout_sec=.1)
            if len(node.status)==3 and all(len(rows)>=COUNT for rows in node.rows.values()):
                break
        metrics = {}
        for name,rows in node.rows.items():
            if len(rows)<2:
                continue
            gaps = np.diff([r['stamp_s'] for r in rows])
            ages = [r['clock_s']-r['stamp_s'] for r in rows]
            metrics[name]=dict(count=len(rows), frames=sorted({r['frame'] for r in rows}),
                               hz_sim=float(1/gaps.mean()), max_gap_sim_s=float(max(gaps)),
                               p95_age_sim_s=float(np.percentile(ages,95)), max_age_sim_s=max(ages))
        windows_overlap = (len(metrics)==3 and
                           max(rows[0]['stamp_s'] for rows in node.rows.values())<
                           min(rows[-1]['stamp_s'] for rows in node.rows.values()))
        passed = (not node.error and len(node.status)==3 and windows_overlap
                  and all(m['count']>=COUNT and m['hz_sim']>=4.5
                          and m['max_gap_sim_s']<.6 and m['max_age_sim_s']<.6
                          for m in metrics.values()))
        result=dict(passed=bool(passed), error=node.error, status=node.status,
                    windows_overlap=bool(windows_overlap), metrics=metrics, rows=node.rows)
        output.write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2),flush=True)
        return 0 if passed else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    raise SystemExit(main(Path(sys.argv[1]),json.loads(Path(sys.argv[2]).read_text())['uav_spawns']))
