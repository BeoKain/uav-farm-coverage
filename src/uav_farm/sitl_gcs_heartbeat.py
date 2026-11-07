#!/usr/bin/env python3
"""Heartbeat-only GCS for the verified local PX4 SITL. No vehicle commands."""
import argparse
import signal
import time
from execution_plan import check_session


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--simulation-only',action='store_true',required=True)
    parser.add_argument('--vehicles',type=int,choices=[1,3],default=1)
    args=parser.parse_args()
    check_fleet=lambda:check_session(vehicles=args.vehicles)
    check_fleet()
    from pymavlink import mavutil
    connections=[mavutil.mavlink_connection(f'udpout:127.0.0.1:{18570+i}',source_system=255,source_component=190)
                 for i in range(args.vehicles)]
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        while True:
            check_fleet()
            for connection in connections:
                connection.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,0,0,mavutil.mavlink.MAV_STATE_ACTIVE)
            time.sleep(1)
    except KeyboardInterrupt:pass
    finally:
        for connection in connections:connection.close()


if __name__=='__main__':main()
