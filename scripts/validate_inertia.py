import xml.etree.ElementTree as ET
import sys
import os

def validate_urdf_physics(file_path):
    if not os.path.exists(file_path):
        print(f"❌ Error: File not found at {file_path}")
        sys.exit(1)
        
    tree = ET.parse(file_path)
    root = tree.getroot()
    links_passed = 0
    
    print(f"--- Validating Physics for {os.path.basename(file_path)} ---")
    
    for link in root.findall('link'):
        inertial = link.find('inertial/inertia')
        if inertial is not None:
            ixx = float(inertial.get('ixx'))
            iyy = float(inertial.get('iyy'))
            izz = float(inertial.get('izz'))
            
            # Triangle Inequality Check
            c1 = (ixx + iyy) >= izz
            c2 = (ixx + izz) >= iyy
            c3 = (iyy + izz) >= ixx
            
            if all([c1, c2, c3]):
                print(f"✅ {link.get('name')}: Physical Consistency Verified")
                links_passed += 1
            else:
                print(f"❌ {link.get('name')}: INVALID INERTIA TENSOR")
                print(f"   Values: ixx={ixx}, iyy={iyy}, izz={izz}")
                sys.exit(1)

    print(f"--- Validation Complete: {links_passed} links verified ---")

if __name__ == "__main__":
    # This logic finds the file in the same folder as the /scripts directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(script_dir, '..', 'humanoid_proto.urdf')
    validate_urdf_physics(urdf_path)