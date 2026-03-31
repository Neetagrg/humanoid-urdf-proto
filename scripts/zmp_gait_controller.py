#!/usr/bin/env python3
"""
ZMP Gait Controller
- Joint control: gz.transport (no subprocess overhead)
- Attitude feedback: ArduPilot MAVLink EKF3
- Standing pose: all joints = 0
"""
import time, math, threading
import numpy as np
from pymavlink import mavutil
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
import sys
sys.path.insert(0, '/home/neetamis/humanoid-ardupilot-sitl/scripts')
from preview_control import LIPMPreviewController, ZMPReferenceGenerator
from foot import SwingFootTrajectory, FootstepPlanner

# Parameters
DT = 0.02
COM_HEIGHT = 0.38
PREVIEW_HORIZON = 20
STEP_LENGTH = 0.08
FOOT_SEP = 0.11
STEP_DURATION = 0.8
DS_DURATION = 0.1
SWING_HEIGHT = 0.04
N_STEPS = 8
L = 0.20

# Setup gz transport node
node = Node()
publishers = {
    'l_hip': node.advertise('/l_hip_pitch/cmd', Double),
    'r_hip': node.advertise('/r_hip_pitch/cmd', Double),
    'l_knee': node.advertise('/l_knee/cmd', Double),
    'r_knee': node.advertise('/r_knee/cmd', Double),
}

def send_joint(name, angle_rad):
    msg = Double()
    msg.data = float(angle_rad)
    publishers[name].publish(msg)

def send_pose(l_hip, r_hip, l_knee, r_knee):
    send_joint('l_hip', l_hip)
    send_joint('r_hip', r_hip)
    send_joint('l_knee', l_knee)
    send_joint('r_knee', r_knee)

def ik(foot_x, foot_z=0.0):
    h = np.clip(COM_HEIGHT - foot_z, 0.01, 2*L - 0.001)
    q_hip = -np.arctan2(foot_x, h)
    return float(q_hip), 0.0

# Attitude from ArduPilot
pitch = 0.0
_lock = threading.Lock()

def mavlink_reader(mav):
    global pitch
    while True:
        msg = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1.0)
        if msg:
            with _lock:
                pitch = msg.pitch

def main():
    print("ZMP Gait Controller starting...")
    mav = mavutil.mavlink_connection('udp:0.0.0.0:14551')
    mav.wait_heartbeat()
    print("Connected to ArduPilot!")
    threading.Thread(target=mavlink_reader, args=(mav,), daemon=True).start()

    lipm_x = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HORIZON)
    lipm_y = LIPMPreviewController(DT, COM_HEIGHT, PREVIEW_HORIZON)
    zmp_gen = ZMPReferenceGenerator(DT, STEP_DURATION, DS_DURATION,
                                     STEP_LENGTH, FOOT_SEP)
    planner = FootstepPlanner(STEP_LENGTH, FOOT_SEP, N_STEPS)

    zmp_x_ref, zmp_y_ref = zmp_gen.generate(N_STEPS)
    footsteps = planner.plan()
    total = len(zmp_x_ref)
    pad = PREVIEW_HORIZON
    zmp_x_pad = np.concatenate([zmp_x_ref, np.full(pad, zmp_x_ref[-1])])
    zmp_y_pad = np.concatenate([zmp_y_ref, np.full(pad, zmp_y_ref[-1])])

    print("Holding stand 3s...")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        send_pose(-0.09, -0.09, 0.35, 0.35)
        time.sleep(DT)

    print(f"Walking {N_STEPS} steps...")
    Nss = int(STEP_DURATION / DT)
    Nds = int(DS_DURATION / DT)
    step_starts = []
    idx = 0
    for _ in range(N_STEPS):
        idx += Nds
        step_starts.append(idx)
        idx += Nss

    step_ptr = 0
    swing_traj = None
    swing_time = 0.0
    swing_side = None
    stance_x = {'left': 0.0, 'right': 0.0}
    swing_x = {'left': 0.0, 'right': 0.0}

    for k in range(total):
        loop_start = time.time()
        com_x = lipm_x.step(zmp_x_pad[k:k+PREVIEW_HORIZON])
        com_y = lipm_y.step(zmp_y_pad[k:k+PREVIEW_HORIZON])

        if step_ptr < len(step_starts) and k >= step_starts[step_ptr]:
            fs = footsteps[step_ptr]
            swing_side = fs[2]
            swing_traj = SwingFootTrajectory(
                swing_x[swing_side], fs[0], fs[1], STEP_DURATION, SWING_HEIGHT)
            swing_time = 0.0
            step_ptr += 1

        with _lock:
            p = pitch

        if swing_traj and swing_side:
            sx, sy, sz = swing_traj.at(swing_time)
            swing_time += DT
            if swing_side == 'left':
                swing_x['left'] = sx
                l_hip, l_knee = ik(sx - com_x, sz)
                r_hip, r_knee = ik(stance_x['right'] - com_x, 0.0)
            else:
                swing_x['right'] = sx
                r_hip, r_knee = ik(sx - com_x, sz)
                l_hip, l_knee = ik(stance_x['left'] - com_x, 0.0)
        else:
            l_hip, l_knee = ik(stance_x['left'] - com_x, 0.0)
            r_hip, r_knee = ik(stance_x['right'] - com_x, 0.0)

        send_pose(l_hip, r_hip, l_knee, r_knee)

        if k % 25 == 0:
            print(f"\rk={k}/{total} com_x={com_x:+.3f} "
                  f"pitch={math.degrees(p):+.1f}deg step={step_ptr}/{N_STEPS}",
                  end='', flush=True)

        elapsed = time.time() - loop_start
        if elapsed < DT:
            time.sleep(DT - elapsed)

    print("\nDone. Returning to stand...")
    for _ in range(50):
        send_pose(-0.09, -0.09, 0.35, 0.35)
        time.sleep(DT)

if __name__ == '__main__':
    main()
