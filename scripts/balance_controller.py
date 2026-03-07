#!/usr/bin/env python3
"""
Basic balance controller for ArduBiped_Proto
"""
from pymavlink import mavutil
import subprocess
import time
import math

print("Connecting to ArduPilot...")
mav = mavutil.mavlink_connection('udp:127.0.0.1:14550')
mav.wait_heartbeat()
print("Connected!")

Kp = 0.3
Ki = 0.005
Kd = 0.05

integral = 0
last_error = 0
last_time = time.time()

def send_joint_command(topic, angle_rad):
    cmd = f'gz topic -t {topic} -m gz.msgs.Double -p "data: {angle_rad}"'
    subprocess.Popen(cmd, shell=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

print("Balance controller running...")
while True:
    msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
    if msg:
        pitch = msg.pitch
        roll = msg.roll
        now = time.time()
        dt = now - last_time if (now - last_time) > 0 else 0.01

        error = pitch
        integral = max(-1.0, min(1.0, integral + error * dt))
        derivative = (error - last_error) / dt
        correction = Kp * error + Ki * integral + Kd * derivative
        correction = max(-0.5, min(0.5, correction))

        send_joint_command('/l_hip_pitch/cmd', -correction)
        send_joint_command('/r_hip_pitch/cmd', -correction)

        last_error = error
        last_time = now

        print(f"Pitch: {math.degrees(pitch):+.1f}° Roll: {math.degrees(roll):+.1f}° Correction: {correction:+.3f}")
    time.sleep(0.025)
