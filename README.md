# Humanoid URDF Proto - ArduPilot SITL Foundation

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

---

## Physics & Simulation Stability

To support high-frequency control loops (400Hz+), this model is optimized for the **ODE/Bullet** solvers used in ArduPilot SITL. Generic URDFs often fail during high-torque transients because they lack physical consistency in their inertial definitions.

### Inertial Symmetry Validation
Every link in this repository has been verified against the **Triangle Inequality for Inertia Tensors**. If a link's mass distribution is mathematically impossible, physics engines produce "NaN" (Not a Number) errors, causing the simulation to "explode."

The validation script ensures:
* $I_{xx} + I_{yy} \ge I_{zz}$
* $I_{xx} + I_{zz} \ge I_{yy}$
* $I_{yy} + I_{zz} \ge I_{xx}$

> **Engineering Note:** By enforcing these constraints, the model remains stable even during rapid balance recovery maneuvers where instantaneous joint torques can spike. This prevents the EKF3 navigation filters from resetting due to sensor "jitter" in the simulated IMU.

---

## Control System Design Notes

###  Why 3-decimal Precision?
All radian limits and inertial origins are specified to $10^{-3}$ precision. This matches ArduPilot’s internal floating-point handling for `MAV_CMD_DO_SET_SERVO`, ensuring that simulation commands don't experience "rounding jitter" which can destabilize high-frequency (400Hz) balance loops.

###  Inertial Symmetry & EKF3
The `base_link` origin is strictly aligned with the geometric center of the torso. This allows the ArduPilot EKF3 (Extended Kalman Filter) to have a zero-offset primary IMU position, reducing the computational overhead of "lever-arm" compensations during rapid pitch/roll transients.

---

## Roadmap
* [x] **Symmetry Implementation:** Mirroring the left leg chain to the right side (`r_hip`, `r_knee`).
* [ ] **Foot Contact Points:** Defining collision primitives for ground-plane interaction.
* [ ] **ArduPilot Plugin:** Mapping URDF joint names to ArduPilot `SERVOx_FUNCTION` outputs.
* [ ] **Physics Engine Integration:** Direct support for Gazebo/Webots for dynamic gait analysis.