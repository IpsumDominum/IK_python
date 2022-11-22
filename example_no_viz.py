from pyik.ik_3d.viz import Visualization
from pyik.ik_3d.joints_3d import JointChain,Joint
from pyik.ik_3d.utils import Vec3
import math

joint_chain = JointChain.from_dh("test.dh")
angles = joint_chain.ik_damped_least_squares(Vec3(0,0,0))
