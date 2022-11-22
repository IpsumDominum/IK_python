from pyik.ik_3d.viz import Visualization
from pyik.ik_3d.joints_3d import JointChain,Joint
import math

joint_chain = JointChain(
    joints = [
        Joint(0,0,0,0,lower=-math.pi,upper=math.pi),
        Joint(0,math.pi/2,0.2,0,lower=-math.pi,upper=math.pi),
        Joint(3,-math.pi/2,0.2,0,lower=-math.pi,upper=math.pi),
        Joint(3,0.2,3,0,lower=-math.pi,upper=math.pi)
    ]
)

viz = Visualization(joint_chain,800,800)        
while True:
    viz.main_loop()