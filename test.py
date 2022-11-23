from pyik.ik_3d.viz import Visualization
from pyik.ik_3d.joints_3d import JointChain, Joint
import math

joint_chain = JointChain(
    joints=[
        Joint(r=0, alpha=0, d=0, theta=0, lower=0, upper=0),
        Joint(
            r=0,
            alpha=math.pi / 2,
            d=1.85 * 3,
            theta=0,
            lower=-0.3,
            upper=0.3,
        ),
        Joint(
            r=0.582 * 3,
            alpha=math.pi / 2,
            d=0,
            theta=0,
            lower=-0.1,
            upper=0,
        ),
        Joint(
            # Intermediate Joint (Not used)
            r=1,
            alpha=0,
            d=1.72 * 3,
            theta=0,
            lower=0,
            upper=0,
        ),
        Joint(
            # Clavicle Yaw
            r=0,
            alpha=math.pi / 2,
            d=0.5,
            theta=math.pi,
            lower=-0.6,
            upper=0.8,
        ),
        Joint(
            # Clavicle roll
            r=0,
            alpha=math.pi / 2,
            d=0.5,
            theta=0,
            lower=-math.pi,
            upper=-math.pi + 1.2,
        ),
        Joint(
            # Humerous Yaw
            r=4,
            alpha=0,
            d=0,
            theta=0,
            lower=0,
            upper=math.pi / 2,
        ),
        Joint(
            # Elbow Yaw
            r=3,
            alpha=math.pi / 2,
            d=0,
            theta=0,
            lower=0,
            upper=math.pi / 2,
        ),
        Joint(
            # Wrist Yaw
            r=0.1,
            alpha=math.pi / 2,
            d=0.1,
            theta=0,
            lower=0,
            upper=math.pi / 2,
        ),
    ]
)
viz = Visualization(joint_chain, 800, 800)
while True:
    viz.main_loop()
