import os


def parse_joint_file(file):
    split = file.split("\n")
    parent_link_name = split[0]
    child_link_name = split[1]
    return (parent_link_name, child_link_name)


def parse_link_file(file):
    values = file.split(" ")
    values = list(map(lambda x: float(x), values))
    return values


def read_format(format_path):
    if not os.path.isdir(format_path):
        print(f"format not found. {format_path}")
        return
    # read links
    links = {}
    links_path = os.path.join(format_path, "links")
    for item in os.listdir(links_path):
        with open(os.path.join(links_path, item), "r") as file:
            f = file.read()
            values = parse_link_file(f)
            link_name = item.strip(".link")
            links[link_name] = values
    # read joints
    joints = {}
    joints_path = os.path.join(format_path, "joints")
    for item in os.listdir(os.path.join(format_path, "joints")):
        with open(os.path.join(joints_path, item), "r") as file:
            f = file.read()
            joint_name = item.strip(".joints")
            joints[joint_name] = parse_joint_file(f)
    return links, joints
