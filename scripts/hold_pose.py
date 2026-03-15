#!/usr/bin/env python3
import subprocess, time

def send(topic, val):
    subprocess.run(
        f'gz topic -p {topic} -m gz.msgs.Double -v "data: {val}"',
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

print("Holding standing pose...")
while True:
    # Slight forward hip pitch to counter forward lean
    send('/l_hip_pitch/cmd', -0.05)
    send('/r_hip_pitch/cmd', -0.05)
    # Slight knee bend for stability
    send('/l_knee/cmd', 0.1)
    send('/r_knee/cmd', 0.1)
    # Ankle compensates
    send('/l_ankle_pitch/cmd', 0.05)
    send('/r_ankle_pitch/cmd', 0.05)
    # Roll neutral
    send('/l_hip_roll/cmd', 0.0)
    send('/r_hip_roll/cmd', 0.0)
    time.sleep(0.02)
