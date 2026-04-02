#!/usr/bin/env python3
"""Bridge: reads SERVO_OUTPUT_RAW from MAVLink, publishes to Gazebo joint topics."""
import subprocess, time
from pymavlink import mavutil

# Channel index (0-based) -> (topic, multiplier, offset)
# Matches ArduPilotPlugin control config in the SDF
JOINTS = [
    (0, '/l_hip_pitch/cmd', 1.571, -0.5),
    (1, '/r_hip_pitch/cmd', 1.571, -0.5),
    (2, '/l_knee/cmd',      2.618, -0.5),
    (3, '/r_knee/cmd',      2.618, -0.5),
]

def pwm_to_rad(pwm, multiplier, offset, pmin=1000, pmax=2000):
    return ((pwm - pmin) / float(pmax - pmin) + offset) * multiplier

def gz_publish(topic, value):
    subprocess.Popen(
        ['gz', 'topic', '-t', topic, '-m', 'gz.msgs.Double', '-p', f'data: {value:.4f}'],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

print("Connecting to MAVLink on udp:14550...")
mav = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
mav.wait_heartbeat()
print(f"Connected — system {mav.target_system}")

# Request servo data at 20Hz
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 20, 1
)

print("Bridge running. Ctrl+C to stop.")
while True:
    msg = mav.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=1.0)
    if msg:
        servos = [msg.servo1_raw, msg.servo2_raw, msg.servo3_raw,
                  msg.servo4_raw, msg.servo5_raw, msg.servo6_raw]
        for ch, topic, mult, off in JOINTS:
            pos = pwm_to_rad(servos[ch], mult, off)
            gz_publish(topic, pos)
            print(f"  ch{ch} pwm={servos[ch]} -> {pos:.3f}rad -> {topic}")
        time.sleep(0.05)  # ~20Hz
