import xml.etree.ElementTree as ET
import os

def print_urdf_tree(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    joints = root.findall('joint')

    print("\n--- Kinematic Tree: " + os.path.basename(file_path) + " ---")

    # adj maps parent_link_name -> list of (child_link_name, joint_name, joint_type)
    adj = {}
    for joint in joints:
        parent = joint.find('parent').get('link')
        child  = joint.find('child').get('link')
        name   = joint.get('name')
        jtype  = joint.get('type')
        adj.setdefault(parent, []).append((child, name, jtype))

    def walk(link_name, prefix="", is_last=True):
        connector = "+-- " if is_last else "+-- "
        print(prefix + connector + link_name)

        children = adj.get(link_name, [])
        child_prefix = prefix + ("    " if is_last else "|   ")

        for i, (child_link, joint_name, joint_type) in enumerate(children):
            last = (i == len(children) - 1)
            joint_connector = "+-- " if last else "+-- "
            print(child_prefix + joint_connector + "[" + joint_name + "] (" + joint_type + ")")
            walk(child_link, child_prefix + ("    " if last else "|   "), is_last=last)

    walk("base_link")

if __name__ == "__main__":
    script_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path  = os.path.join(script_dir, '..', 'humanoid_proto.urdf')
    print_urdf_tree(urdf_path)