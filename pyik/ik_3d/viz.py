from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from .joints_3d import JointChain, Joint
from .utils import Vec3
from .scene import Scene
import pygame
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class Visualization(Scene):
    def __init__(self, joint_chain, width, height, show_axis=False):
        super(Visualization, self).__init__(width, height)
        self.joint_chain = joint_chain
        self.target = Vec3(0, 0, 0)
        self.cur_j = 0
        self.doing_ik = True
        self.show_axis = show_axis

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
            j = self.joint_chain.get_joints()[self.cur_j]
            j.rotate(j.rot_angle - 0.1)
        if keypress[pygame.K_k]:
            j = self.joint_chain.get_joints()[self.cur_j]
            j.rotate(j.rot_angle + 0.1)
        if keypress[pygame.K_i]:
            self.doing_ik = True
        if keypress[pygame.K_u]:
            self.doing_ik = False

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
        if self.doing_ik:
            self.joint_chain.ik_fabrik(self.target)
        else:
            self.joint_chain.forward_kinematics()
        self.draw_sphere(self.target, color=(0.2, 0.5, 0.1, 0.5))
        # Visualize joints
        for i, j in enumerate(self.joint_chain.get_joints()):
            if i + 1 == self.cur_j:
                self.draw_joint(
                    j.position, j.pose, length=1, color="orange", radius=0.5
                )
            elif i == 0:
                self.draw_joint(
                    j.position, j.pose, length=1, color="yellow", radius=0.5
                )
            elif i == len(self.joint_chain.get_joints()) - 1:
                self.draw_sphere(j.position, color="light_blue", radius=0.6)
            else:
                self.draw_joint(j.position, j.pose, length=1, color="black", radius=0.5)

            if self.show_axis:
                self.draw_axis(j.position, j.pose, length=2, radius=0.1)
            if j.parent:
                self.draw_link(
                    j.parent.position, j.orientation, j.length, color="gray", radius=0.3
                )
