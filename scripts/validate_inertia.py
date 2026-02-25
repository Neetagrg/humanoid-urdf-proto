import xml.etree.ElementTree as ET
import sys

def validate_urdf_physics(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()
    links_passed = 0
    
    print(f"--- Validating Physics for {file_path} ---")
    
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
                sys.exit(1) # Fail the CI/CD build

    print(f"--- Validation Complete: {links_passed} links verified ---")

if __name__ == "__main__":
    validate_urdf_physics('humanoid_proto.urdf')