from lxml import etree
from collections import defaultdict
def divide_xyz(xyz_str, n):
    xyz = list(map(float, xyz_str.strip().split()))
    step = [v / (n + 1) for v in xyz]
    return step

def multiply_xyz(xyz_step, factor):
    return " ".join(str(v * factor) for v in xyz_step)

def get_origin_xyz_rpy(elem):
    origin = elem.find("origin")
    xyz = origin.get("xyz", "0 0 0") if origin is not None else "0 0 0"
    rpy = origin.get("rpy", "0 0 0") if origin is not None else "0 0 0"
    return xyz, rpy

def create_link(name):
    return etree.Element("link", name=name)

def create_joint(name, joint_type, parent, child, xyz, rpy="0 0 0"):
    joint = etree.Element("joint", name=name, type=joint_type)
    etree.SubElement(joint, "parent", link=parent)
    etree.SubElement(joint, "child", link=child)
    etree.SubElement(joint, "origin", xyz=xyz, rpy=rpy)
    return joint

def main():
    import sys

    input_file = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10.urdf'
    output_file = '/home/galileo/projects/thor_ws/src/thor_core/thor_math/ur10/ur10_with_intermediates.urdf'

    tree = etree.parse(input_file)
    root = tree.getroot()

    try:
        n_frames = int(input("Enter the number of intermediate frames per joint: "))
        assert n_frames >= 1
    except Exception:
        print("Invalid input. Please enter an integer >= 1.")
        return

    for joint in list(root.findall("joint")):
        joint_type = joint.get("type")
        if joint_type == "fixed":
            continue  # Skip fixed joints

        name = joint.get("name")
        parent = joint.find("parent").get("link")
        child = joint.find("child").get("link")
        xyz_str, rpy_str = get_origin_xyz_rpy(joint)

        step = divide_xyz(xyz_str, n_frames)
        current_parent = parent

        # Insert N intermediate frames
        for i in range(1, n_frames + 1):
            intermediate_name = f"{name}_intermediate_{i}"
            intermediate_link = create_link(intermediate_name)
            root.append(intermediate_link)

            step_xyz = multiply_xyz(step, 1)

            # Create fixed joint to the intermediate frame
            fixed_joint = create_joint(
                f"{name}_mid_{i}",
                "fixed",
                current_parent,
                intermediate_name,
                xyz=multiply_xyz(step, 1),
                rpy="0 0 0"
            )
            root.append(fixed_joint)

            current_parent = intermediate_name

        # Update original joint: parent is now the last intermediate link, and xyz is final step
        joint.find("parent").set("link", current_parent)
        if joint.find("origin") is not None:
            joint.find("origin").set("xyz", multiply_xyz(step, 1))
        else:
            etree.SubElement(joint, "origin", xyz=multiply_xyz(step, 1), rpy=rpy_str)

    # Save output
    with open(output_file, "wb") as f:
        f.write(etree.tostring(root, pretty_print=True, xml_declaration=True, encoding="UTF-8"))

    print(f"✅ Modified URDF with {n_frames} intermediate frame(s) per joint saved to {output_file}")

if __name__ == "__main__":
    main()
