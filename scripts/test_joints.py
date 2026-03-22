#!/usr/bin/env python3
import subprocess, time

def pub(topic, val):
    subprocess.Popen(['gz','topic','-t',topic,'-m','gz.msgs.Double',
                      '-p',f'data: {val:.3f}'],
                     stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Holding standing pose for 10s - watch if robot stays still...")
t0 = time.time()
while time.time() - t0 < 10.0:
    pub('/l_hip_pitch/cmd', -0.09)
    pub('/r_hip_pitch/cmd', -0.09)
    pub('/l_knee/cmd', 0.35)
    pub('/r_knee/cmd', 0.35)
    time.sleep(0.02)
print("Done!")
