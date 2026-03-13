#!/bin/bash
echo "Starting ArduHumanoid SITL..."

# Start ArduPilot with MAVProxy output
cd ~/ardupilot && Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --console --out 127.0.0.1:14551 &
ARDUPILOT_PID=$!

echo "Waiting for ArduPilot to start..."
sleep 8

# Start Gazebo
gz sim ~/humanoid-ardupilot-sitl/worlds/ardupilot_humanoid.sdf &

echo "Waiting for Gazebo..."
sleep 5

# Start balance controller
cd ~/humanoid-ardupilot-sitl && python3 scripts/balance_controller.py

