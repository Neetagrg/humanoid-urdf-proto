#!/usr/bin/env python3
import math
import time
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double

DT = 0.02

HIP_STAND = -0.09
KNEE_STAND = 0.35

HIP_DELTA = 0.06
KNEE_DELTA = 0.10
CYCLE_SEC = 6.0

node = Node()

pubs = {
    "l_hip": node.advertise("/l_hip_pitch/cmd", Double),
    "r_hip": node.advertise("/r_hip_pitch/cmd", Double),
    "l_knee": node.advertise("/l_knee/cmd", Double),
    "r_knee": node.advertise("/r_knee/cmd", Double),
}

def publish(topic_key: str, value: float):
    msg = Double()
    msg.data = float(value)
    pubs[topic_key].publish(msg)

def send_pose(l_hip: float, r_hip: float, l_knee: float, r_knee: float):
    publish("l_hip", l_hip)
    publish("r_hip", r_hip)
    publish("l_knee", l_knee)
    publish("r_knee", r_knee)

def smoothstep01(x: float) -> float:
    x = max(0.0, min(1.0, x))
    return x * x * (3.0 - 2.0 * x)

def main():
    print("Holding stand...")
    for _ in range(int(2.0 / DT)):
        send_pose(HIP_STAND, HIP_STAND, KNEE_STAND, KNEE_STAND)
        time.sleep(DT)

    print("Starting symmetric squat loop...")
    start = time.time()

    while True:
        t = time.time() - start
        phase = (t % CYCLE_SEC) / CYCLE_SEC

        if phase < 0.5:
            a = smoothstep01(phase / 0.5)
        else:
            a = smoothstep01((1.0 - phase) / 0.5)

        hip = HIP_STAND - HIP_DELTA * a
        knee = KNEE_STAND + KNEE_DELTA * a

        send_pose(hip, hip, knee, knee)

        if int(t * 10) % 10 == 0:
            print(f"\rhip={hip:+.3f} knee={knee:+.3f}", end="", flush=True)

        time.sleep(DT)

if __name__ == "__main__":
    main()
