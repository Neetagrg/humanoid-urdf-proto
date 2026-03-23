#!/usr/bin/env python3
"""Send zero angles to all joints - test if robot stands upright"""
import subprocess, time

joints = [
    '/l_hip_pitch/cmd',
    '/r_hip_pitch/cmd', 
    '/l_knee/cmd',
    '/r_knee/cmd',
]

print("Sending zero to all joints for 10 seconds...")
t0 = time.time()
while time.time() - t0 < 10.0:
    for topic in joints:
        subprocess.Popen(
            ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', 'data: 0.0'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
    time.sleep(0.05)
print("Done.")
