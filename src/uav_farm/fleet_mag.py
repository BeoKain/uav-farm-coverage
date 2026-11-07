"""Convert the three simulated magnetometers for the legacy PX4 bridge."""
import json
from pathlib import Path
import signal
import sys
import time

from gz.transport13 import Node
from gz.msgs10.magnetometer_pb2 import Magnetometer

from mag_interface import encode_legacy_gauss
from execution_plan import check_session


def main(output):
    check_session(vehicles=3)
    node = Node()
    counts = [0,0,0]
    publishers, topics = [], []
    for i in range(3):
        base = f'/world/farm/model/x500_{i}/link/base_link/sensor/'
        publisher = node.advertise(base+'magnetometer_sensor/magnetometer', Magnetometer)
        publishers.append(publisher)
        topic = base+'magnetometer_enu/magnetometer'
        topics.append(topic)

        def receive(msg, index=i, pub=publisher):
            out = Magnetometer()
            out.CopyFrom(msg)
            f = msg.field_tesla
            out.field_tesla.x,out.field_tesla.y,out.field_tesla.z = encode_legacy_gauss((f.x,f.y,f.z))
            pub.publish(out)
            counts[index] += 1

        if not node.subscribe(Magnetometer, topic, receive):
            raise RuntimeError(f'Could not subscribe to {topic}')
    stop = False

    def cancel(*_):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, cancel)
    signal.signal(signal.SIGINT, cancel)
    while not stop:
        time.sleep(.1)
    for topic in topics:
        node.unsubscribe(topic)
    output.write_text(json.dumps(dict(topics=topics, forwarded=counts),indent=2)+'\n')


if __name__=='__main__':
    raise SystemExit(main(Path(sys.argv[1])))
