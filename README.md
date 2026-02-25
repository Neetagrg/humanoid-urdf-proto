[![URDF Validation](https://github.com/Neetagrg/humanoid-urdf-proto/actions/workflows/validate_urdf.yml/badge.svg)](https://github.com/Neetagrg/humanoid-urdf-proto/actions)

## Architecture & Kinematics

The humanoid follows a modular branching structure. Each limb is defined by a chain of revolute joints, allowing for future expansion into full 6-DOF (Degrees of Freedom) legs.

### Joint Mapping
| Joint Name | Type | Axis | Physical Hardware |
| :--- | :--- | :--- | :--- |
| `base_to_torso` | Fixed | N/A | Chassis Frame |
| `l_hip_pitch` | Revolute | Y (Lateral) | MG996R Servo |
| `l_knee` | Revolute | Y (Lateral) | MG996R Servo |
| `r_hip_pitch` | Revolute | Y (Lateral) | MG996R Servo (Planned) |
| `r_knee` | Revolute | Y (Lateral) | MG996R Servo (Planned) |

### Coordinate Frames
All joints follow the **Right-Hand Rule**. The Z-axis points upward (Normal to ground), and the X-axis points forward (Direction of travel). This alignment is critical for ArduPilot's EKF3 navigation filters to interpret IMU data correctly.