#!/usr/bin/env python3
"""
Balance controller for ArduBiped_Proto
Reads torso pitch from ArduPilot EKF3 via MAVLink
Sends joint corrections to Gazebo via gz topic
Usage: In MAVProxy console type: output add 127.0.0.1:14551
Then run: python3 scripts/balance_controller.py
"""
from pymavlink import mavutil
import time, math, subprocess

print("Connecting to ArduPilot on udp:14551...")
print("Make sure to run in MAVProxy: output add 127.0.0.1:14551")
mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
mav.wait_heartbeat()
print("Connected!")

Kp, Ki, Kd = 0.3, 0.005, 0.05
integral, last_error, last_time = 0, 0, time.time()

def send_joint(topic, angle):
    subprocess.Popen(
        f'gz topic -t {topic} -m gz.msgs.Double -p "data: {angle}"',
        shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Balance controller running...")
while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    if msg:
        pitch = msg.pitch
        now = time.time()
        dt = max(now - last_time, 0.01)
        error = pitch
        integral = max(-1.0, min(1.0, integral + error * dt))
        derivative = (error - last_error) / dt
        correction = max(-0.5, min(0.5, Kp*error + Ki*integral + Kd*derivative))
        send_joint('/l_hip_pitch/cmd', -correction)
        send_joint('/r_hip_pitch/cmd', -correction)
        last_error, last_time = error, now
        print(f"Pitch: {math.degrees(pitch):+.1f}° Correction: {correction:+.3f}")
    time.sleep(0.025)
