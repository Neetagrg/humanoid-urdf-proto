"""
calculate_com.py  —  Joint-transform-aware CoM validator for ArduPilot SITL URDF
Fixes vs original:
  - Builds a joint transform tree and accumulates each link's inertial origin
    in world frame before summing, so the reported CoM is physically correct.
  - Validates inertia triangle inequality per link.
  - Reports total mass, world-frame CoM, and lateral symmetry check.
  - Warns if any inertia diagonal entry is below the ODE stability floor (1e-4).
"""

import xml.etree.ElementTree as ET
import os
import numpy as np


# ─────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────

def parse_xyz(element, attr="xyz"):
    """Return np.array([x, y, z]) from an XML element attribute, defaulting to zeros."""
    if element is None:
        return np.zeros(3)
    raw = element.get(attr, "0 0 0").split()
    return np.array([float(v) for v in raw])


def parse_rpy(element):
    """Return (roll, pitch, yaw) from an XML element's 'rpy' attribute."""
    if element is None:
        return 0.0, 0.0, 0.0
    raw = element.get("rpy", "0 0 0").split()
    return tuple(float(v) for v in raw)


def rpy_to_matrix(roll, pitch, yaw):
    """
    Build a 3×3 rotation matrix from roll-pitch-yaw (extrinsic XYZ convention,
    same as URDF/ROS).
    """
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)

    Rx = np.array([[1,  0,   0 ],
                   [0,  cr, -sr],
                   [0,  sr,  cr]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


def make_transform(origin_element):
    """
    Return (R, t): 3×3 rotation matrix and 3-vector translation
    from a URDF <origin xyz="..." rpy="..."/> element (or None → identity).
    """
    t = parse_xyz(origin_element)
    r, p, y = parse_rpy(origin_element)
    R = rpy_to_matrix(r, p, y)
    return R, t


# ─────────────────────────────────────────────
# Build kinematic tree from URDF
# ─────────────────────────────────────────────

def build_tree(root):
    """
    Returns:
        link_inertials : dict  link_name → (mass, local_com_xyz, inertia_dict)
        joint_map      : dict  child_link_name → (parent_link_name, R_joint, t_joint)
    """
    link_inertials = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        if inertial is None:
            link_inertials[name] = (0.0, np.zeros(3), None)
            continue

        mass_elem = inertial.find("mass")
        mass = float(mass_elem.get("value", 0)) if mass_elem is not None else 0.0

        origin = inertial.find("origin")
        local_com = parse_xyz(origin)

        inertia_elem = inertial.find("inertia")
        inertia = {}
        if inertia_elem is not None:
            for attr in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia[attr] = float(inertia_elem.get(attr, 0))

        link_inertials[name] = (mass, local_com, inertia)

    joint_map = {}   # child → (parent, R, t)
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link")
        child  = joint.find("child").get("link")
        origin = joint.find("origin")
        R, t   = make_transform(origin)
        joint_map[child] = (parent, R, t)

    return link_inertials, joint_map


def world_transform(link_name, joint_map):
    """
    Walk up the kinematic chain to compute the world-frame transform
    (R_world, t_world) for link_name's origin.
    Returns (R, t) where t is the position of the link's frame origin in world.
    """
    R = np.eye(3)
    t = np.zeros(3)
    current = link_name

    chain = []
    while current in joint_map:
        parent, R_j, t_j = joint_map[current]
        chain.append((R_j, t_j))
        current = parent

    # Apply transforms from root → leaf
    for R_j, t_j in reversed(chain):
        t = t + R @ t_j
        R = R @ R_j

    return R, t


# ─────────────────────────────────────────────
# Inertia validation
# ─────────────────────────────────────────────

ODE_INERTIA_FLOOR = 1e-4   # kg·m² — below this ODE becomes numerically rank-deficient

def check_inertia(link_name, inertia):
    """Print triangle-inequality and ODE floor warnings for a single link."""
    if not inertia:
        return
    ixx = inertia.get("ixx", 0)
    iyy = inertia.get("iyy", 0)
    izz = inertia.get("izz", 0)

    issues = []
    if ixx + iyy < izz - 1e-9: issues.append("ixx+iyy < izz  (triangle inequality violated)")
    if ixx + izz < iyy - 1e-9: issues.append("ixx+izz < iyy  (triangle inequality violated)")
    if iyy + izz < ixx - 1e-9: issues.append("iyy+izz < ixx  (triangle inequality violated)")

    for v, label in ((ixx, "ixx"), (iyy, "iyy"), (izz, "izz")):
        if 0 < v < ODE_INERTIA_FLOOR:
            issues.append(f"{label}={v:.2e} is below ODE stability floor {ODE_INERTIA_FLOOR:.0e}")

    for issue in issues:
        print(f"  ⚠️  {link_name}: {issue}")


# ─────────────────────────────────────────────
# Main analysis
# ─────────────────────────────────────────────

def calculate_com(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()

    link_inertials, joint_map = build_tree(root)

    total_mass = 0.0
    world_com  = np.zeros(3)

    print(f"\n{'─'*62}")
    print(f"  CoM Analysis (world frame): {os.path.basename(urdf_file)}")
    print(f"{'─'*62}")
    print(f"  {'Link':<22} {'Mass':>6}  {'World CoM (x, y, z)'}")
    print(f"{'─'*62}")

    for link_name, (mass, local_com, inertia) in link_inertials.items():
        if mass <= 0:
            continue

        # Get world-frame transform of this link's origin
        R_w, t_w = world_transform(link_name, joint_map)

        # CoM in world frame = link_origin_world + R_world · local_com
        com_world = t_w + R_w @ local_com

        print(f"  {link_name:<22} {mass:>5.3f}kg  "
              f"({com_world[0]:+.4f}, {com_world[1]:+.4f}, {com_world[2]:+.4f})")

        check_inertia(link_name, inertia)

        total_mass   += mass
        world_com    += mass * com_world

    if total_mass > 0:
        world_com /= total_mass

    print(f"{'─'*62}")
    print(f"  TOTAL MASS : {total_mass:.4f} kg")
    print(f"  WORLD CoM  : X={world_com[0]:+.4f}  Y={world_com[1]:+.4f}  Z={world_com[2]:+.4f}")
    print(f"{'─'*62}")

    # Symmetry checks
    if abs(world_com[1]) > 0.001:
        print(f"  ⚠️  LATERAL CoM (Y) = {world_com[1]:.4f} m — robot is NOT laterally centred!")
    else:
        print(f"  ✅ Lateral symmetry OK  (|Y| = {abs(world_com[1]):.5f} m < 0.001 m)")

    if abs(world_com[0]) > 0.005:
        print(f"  ⚠️  FORE-AFT CoM (X) = {world_com[0]:.4f} m — consider shifting mass.")
    else:
        print(f"  ✅ Fore-aft balance OK  (|X| = {abs(world_com[0]):.5f} m < 0.005 m)")

    # Sanity-check CoM height: for a bipedal robot the CoM should be above mid-stance
    print(f"\n  CoM height (Z) = {world_com[2]:.4f} m above base_link origin.")
    print(f"{'─'*62}\n")

    return total_mass, world_com


if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path  = os.path.join(script_dir, "..", "humanoid_proto.urdf")
    calculate_com(urdf_path)