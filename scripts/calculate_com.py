import xml.etree.ElementTree as ET
import os

def calculate_com(urdf_file):
    tree = ET.parse(urdf_file)
    root = tree.getroot()
    
    total_mass = 0.0
    weighted_x, weighted_y, weighted_z = 0.0, 0.0, 0.0

    print(f"--- Center of Mass Analysis: {os.path.basename(urdf_file)} ---")

    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is not None:
            mass_elem = inertial.find('mass')
            if mass_elem is None: continue
            
            mass = float(mass_elem.get('value', 0))
            origin = inertial.find('origin')
            
            # If origin is missing, default to 0 0 0
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
            else:
                xyz = ['0', '0', '0']
                
            x, y, z = map(float, xyz)

            total_mass += mass
            weighted_x += mass * x
            weighted_y += mass * y
            weighted_z += mass * z
            print(f"Link: {link.get('name'):<20} | Mass: {mass:<5}kg")

    if total_mass > 0:
        com_x = weighted_x / total_mass
        com_y = weighted_y / total_mass
        com_z = weighted_z / total_mass

        print("-" * 50)
        print(f"TOTAL ROBOT MASS: {total_mass:.3f} kg")
        print(f"AGGREGATE CoM:    X: {com_x:.3f}, Y: {com_y:.3f}, Z: {com_z:.3f}")
        print("-" * 50)
        
        if abs(com_y) > 0.01:
            print("⚠️ WARNING: Lateral CoM (Y) is not centered!")
        else:
            print("✅ Lateral Symmetry Verified (Y-axis is centered).")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, '..', 'humanoid_proto.urdf')
    calculate_com(urdf_path)
