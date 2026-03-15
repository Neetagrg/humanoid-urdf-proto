#!/bin/bash
pkill -f mavproxy
pkill -f arducopter
pkill -f sim_vehicle
pkill -f gait_controller
pkill -f gz
sleep 3
echo "Now manually run in 3 terminals:"
echo "1: gz sim ~/humanoid-ardupilot-sitl/worlds/ardupilot_humanoid.sdf"
echo "2: cd ~/ardupilot && sim_vehicle.py -v ArduCopter -f JSON --out=udp:127.0.0.1:14551 --no-rebuild"
echo "3: cd ~/humanoid-ardupilot-sitl && python3 scripts/gait_controller.py"
