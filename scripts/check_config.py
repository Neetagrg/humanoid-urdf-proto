#!/usr/bin/env python3
import subprocess
import sys

topics = [
    '/l_hip_pitch/cmd',
    '/r_hip_pitch/cmd', 
    '/l_knee/cmd',
    '/r_knee/cmd',
    '/l_ankle/cmd',
    '/r_ankle/cmd'
]

print("Checking joint topics...")
all_ok = True
for topic in topics:
    result = subprocess.run(
        ['gz', 'topic', '-i', '-t', topic],
        capture_output=True, text=True, timeout=3
    )
    has_sub = 'Subscribers' in result.stdout and 'gz.msgs.Double' in result.stdout.split('Subscribers')[-1]
    status = "OK" if has_sub else "NO SUBSCRIBER"
    if not has_sub:
        all_ok = False
    print(f"  {topic}: {status}")

print()
print("ALL OK" if all_ok else "SOME TOPICS MISSING - restart Gazebo")
