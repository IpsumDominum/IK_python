from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from joints_3d import JointChain, Vec3, Joint
from scene import Scene
import pygame
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


class JointTest(Scene):
    def __init__(self, *args, **kwargs):
        super(JointTest, self).__init__(*args, **kwargs)

        self.joint_chain = JointChain(
            joints=[
                Joint(r=0, alpha=0, d=0, theta=0),
                Joint(r=0, alpha=math.pi/2, d=5, theta=0),
                Joint(r=5, alpha=0, d=5, theta=0),
                Joint(r=2, alpha=0, d=2, theta=0),
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
        DEBUG = False
        for i, j in enumerate(self.joint_chain.get_joints()):
            if(i==1):
                pass
                #j.rotate(self.t*3)
        (
            back_vecs,
            back_poses,
            forward_vecs,
            forward_poses,
        ) = self.joint_chain.perform_fabrik(self.target)
        self.draw_sphere(self.target, color=(0.2, 0.5, 0.1, 0.5))
        # Visualize joints        
        for i, j in enumerate(self.joint_chain.get_joints()):
            if(i==0):                
                self.draw_joint(j.position, j.pose,length=1,color="yellow",radius=0.5)
                length = 2
            elif(i==len(self.joint_chain.get_joints())-1):
                self.draw_sphere(j.position, color="light_blue",radius=0.6)
            else:
                self.draw_joint(j.position, j.pose,length=1,color="black",radius=0.5)
        
        for i, j in enumerate(self.joint_chain.get_joints()):
            self.draw_axis(
                j.position, j.pose, length=2, radius=0.1
            )
            if(j.parent):
                self.draw_link(j.parent.position, j.orientation, j.length, color="gray", radius=0.3)
            if(i!=len(self.joint_chain.get_joints())-1):
                self.draw_plane(j.position,j.x_axis,j.y_axis,1, color="gray")
        
        
        for i in range(len(back_poses)):
            pos = back_poses[i]
            vec = back_vecs[i]
            joint = self.joint_chain.get_joints()[len(self.joint_chain.get_joints())-1-i]
            self.draw_sphere(pos)
            self.draw_link(pos, vec,joint.length, color="gray", radius=0.3)            
        
        
        for i in range(len(forward_vecs)):
            pos = forward_poses[i]
            vec = forward_vecs[i]            
            self.draw_sphere(pos,color="green")
            joint = self.joint_chain.get_joints()[i+1]
            self.draw_link(pos, vec,joint.length, color="green", radius=0.3)


j = JointTest(width=500, height=500)


def main():
    while True:
        j.main_loop()


# Call the main function
if __name__ == "__main__":
    main()
