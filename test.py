from read_format import read_format
from joints3d import JointChain

links, joints = read_format("ameca.robot")

joint_chain = JointChain(joints=joints, links=links)
