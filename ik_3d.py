from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from joints_3d import JointChain, Vec3, Joint
from scene import Scene
import pygame
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import math


class JointTest(Scene):
    def __init__(self, *args, **kwargs):
        super(JointTest, self).__init__(*args, **kwargs)

        self.joint_chain = JointChain(
            root_pos=Vec3(0, 0, 0),
            joints=[
                Joint(3, 0.5, 5, 0.5),
                Joint(4, 0.5, 2, 0.5),
            ],
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

    def render(self):
        self.draw_sphere(self.joint_chain._root.position, color=(0.5, 0.5, 0.1, 0.5))
        self.draw_sphere(self.target, color=(0.2, 0.5, 0.1, 0.5))

        DEBUG = False
        (
            back_vecs,
            back_poses,
            forward_vecs,
            forward_poses,
        ) = self.joint_chain.perform_fabrik(self.target)
        # Visualize joints
        for i, j in enumerate(self.joint_chain.get_joints()):
            self.draw_sphere(j.position, color="black")
            self.draw_joint(
                "z", j.parent.position, j.pose, j.length, color="blue", radius=0.2
            )
            self.draw_joint(
                "y", j.parent.position, j.pose, j.length, color="green", radius=0.2
            )
            self.draw_joint(
                "x", j.parent.position, j.pose, j.length, color="red", radius=0.2
            )


j = JointTest(width=500, height=500)


def main():
    while True:
        j.main_loop()


# Call the main function
if __name__ == "__main__":
    main()
