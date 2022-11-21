from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from joints_3d import JointChain, Joint
from utils import Vec3
from scene import Scene
import pygame
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class JointTest(Scene):
    def __init__(self, *args, **kwargs):
        super(JointTest, self).__init__(*args, **kwargs)

        robot_a = [
            Joint(r=0, alpha=0, d=0, theta=0),
            Joint(r=0, alpha=math.pi / 2, d=2, theta=0),
            Joint(r=0, alpha=math.pi / 2, d=3, theta=0),
            Joint(r=3, alpha=0, d=0, theta=0),
        ]
        robot_b = [
            Joint(r=0, alpha=0, d=0, theta=0),
            Joint(r=0, alpha=-math.pi / 2, d=3, theta=0),
            Joint(r=0, alpha=-math.pi / 2, d=0.1, theta=0),
            Joint(r=0, alpha=-math.pi / 2, d=3, theta=0),
            Joint(r=0, alpha=-math.pi / 2, d=0.1, theta=0),
            Joint(r=0, alpha=-math.pi / 2, d=2.9, theta=0),
            Joint(r=1.5, alpha=math.pi / 2, d=0.1, theta=0),
            Joint(r=1.5, alpha=0, d=2.9, theta=0),
        ]
        self.joint_chain = JointChain(
            joints=robot_a,
        )
        self.target = Vec3(5, 10, 5)
        self.cur_j = 0

    def handle_keypress(self, keypress):
        if keypress[pygame.K_UP]:
            self.target.y += 0.1
        if keypress[pygame.K_DOWN]:
            self.target.y -= 0.1
        if keypress[pygame.K_LEFT]:
            self.target.x += 0.1
        if keypress[pygame.K_RIGHT]:
            self.target.x -= 0.1
        if keypress[pygame.K_n]:
            self.target.z += 0.1
        if keypress[pygame.K_m]:
            self.target.z -= 0.1
        if keypress[pygame.K_j]:
            self.joint_chain.get_joints()[self.cur_j].theta -= 0.1
        if keypress[pygame.K_k]:
            self.joint_chain.get_joints()[self.cur_j].theta += 0.1
        if keypress[pygame.K_u]:
            self.cur_j = (self.cur_j + 1) % len(self.joint_chain._joints)
        if keypress[pygame.K_0]:
            self.cur_j = 0
        if keypress[pygame.K_1]:
            self.cur_j = 1
        if keypress[pygame.K_2]:
            self.cur_j = 2
        if keypress[pygame.K_3]:
            self.cur_j = 3
        if keypress[pygame.K_4]:
            self.cur_j = 4
        if keypress[pygame.K_5]:
            self.cur_j = 5
        if keypress[pygame.K_6]:
            self.cur_j = 6
        if keypress[pygame.K_7]:
            self.cur_j = 7
        if keypress[pygame.K_8]:
            self.cur_j = 8
        if keypress[pygame.K_9]:
            self.cur_j = 9
        self.cur_j = max(0, min(self.cur_j, len(self.joint_chain._joints) - 1))

    def render(self):
        DEBUG = False
        for i, j in enumerate(self.joint_chain.get_joints()):
            if i == 1:
                pass
                # j.rotate(self.t*3)
        (
            back_poses_p_prev,
            back_vec_p_prev,
            back_vecs,
            back_poses,
            forward_vecs,
            forward_poses,
        ) = self.joint_chain.perform_fabrik(self.target)
        self.draw_sphere(self.target, color=(0.2, 0.5, 0.1, 0.5))
        # Visualize joints
        for i, j in enumerate(self.joint_chain.get_joints()):
            if i + 1 == self.cur_j:
                self.draw_joint(
                    j.position, j.pose, length=1, color="orange", radius=0.5
                )
                continue
            if i == 0:
                self.draw_joint(
                    j.position, j.pose, length=1, color="yellow", radius=0.5
                )
            elif i == len(self.joint_chain.get_joints()) - 1:
                self.draw_sphere(j.position, color="light_blue", radius=0.6)
            else:
                self.draw_joint(j.position, j.pose, length=1, color="black", radius=0.5)

        for i, j in enumerate(self.joint_chain.get_joints()):
            self.draw_axis(j.position, j.pose, length=2, radius=0.1)
            if j.parent:
                self.draw_link(
                    j.parent.position, j.orientation, j.length, color="gray", radius=0.3
                )
            if i != len(self.joint_chain.get_joints()) - 1:
                self.draw_plane(j.position, j.x_axis, j.y_axis, 1, color="gray")

        
        for i in range(len(back_poses)):
            pos = back_poses[i]
            vec = back_vecs[i]
            joint = self.joint_chain.get_joints()[
                len(self.joint_chain.get_joints()) - 1 - i
            ]
            self.draw_sphere(pos, radius=0.8, color="purple")
            self.draw_link(pos, vec, joint.length, color="purple", radius=0.3)
        
        for i in range(len(back_poses_p_prev)):
            pos = back_poses_p_prev[i]
            vec = back_vec_p_prev[i]
            joint = self.joint_chain.get_joints()[
                len(self.joint_chain.get_joints()) - 1 - i
            ]
            self.draw_sphere(pos, radius=0.8, color="green")
            self.draw_link(pos, vec, joint.length, color="green", radius=0.3)

        for i in range(len(forward_vecs)):
            pos = forward_poses[i]
            vec = forward_vecs[i]
            self.draw_sphere(pos, color="green", radius=0.3)
            joint = self.joint_chain.get_joints()[i + 1]
            self.draw_link(pos, vec, joint.length, color="green", radius=0.3)


j = JointTest(width=500, height=500)


def main():
    while True:
        j.main_loop()


# Call the main function
if __name__ == "__main__":
    main()
