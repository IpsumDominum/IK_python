from pyik.ik_3d.viz import Visualization
from pyik.ik_3d.joints_3d import JointChain, Joint
import math

joint_chain = JointChain.from_dh("test.dh")

viz = Visualization(joint_chain, 800, 800)
while True:
    viz.main_loop()
