#!/bin/bash
echo "Holding pose - keep this running while starting ArduPilot and gait controller"
while true; do
  gz topic -t /l_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2" 2>/dev/null
  gz topic -t /r_hip_pitch/cmd -m gz.msgs.Double -p "data: -0.2" 2>/dev/null
  gz topic -t /l_knee/cmd -m gz.msgs.Double -p "data: 0.4" 2>/dev/null
  gz topic -t /r_knee/cmd -m gz.msgs.Double -p "data: 0.4" 2>/dev/null
  sleep 0.1
done
