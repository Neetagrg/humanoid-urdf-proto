#!/usr/bin/env python3
"""Send zero position to all joints immediately on startup"""
import subprocess, time

def send(topic, val):
    subprocess.run(f'gz topic -t {topic} -m gz.msgs.Double -p "data: {val}"',
                   shell=True)

print("Locking joints to standing pose...")
for _ in range(20):
    send('/l_hip_pitch/cmd', 0.0)
    send('/r_hip_pitch/cmd', 0.0)
    send('/l_knee/cmd', 0.0)
    send('/r_knee/cmd', 0.0)
    time.sleep(0.05)
print("Done - joints locked")
