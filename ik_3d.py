from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from joints_3d import JointChain, Vec3, Joint
from scene import Scene
import pygame


class JointTest(Scene):
    def __init__(self, *args, **kwargs):
        super(JointTest, self).__init__(*args, **kwargs)
        self.joint_chain = JointChain(
            root=Vec3(0, 0, 0),
            joints=[
                Joint(5, Vec3(1, 0, 0), Vec3(0, 1, 0)),
                Joint(8, Vec3(0, 1, 0), Vec3(1, 0, 0)),
                Joint(8, Vec3(0, 0, 1), Vec3(0, 1, 0)),
            ],
        )
        self.target = Vec3(5, 10, 5)

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

    def render(self):
        self.draw_sphere(self.joint_chain._root, color=(0.5, 0.5, 0.1, 0.5))
        self.draw_sphere(self.target, color=(0.2, 0.5, 0.1, 0.5))

        (
            back_vecs,
            back_poses,
            forward_vecs,
            forward_poses,
        ) = self.joint_chain.perform_fabrik(self.target)
        DEBUG = True
        # Visualize joints
        for i, j in enumerate(self.joint_chain.get_joints()):

            if DEBUG:
                # Debug backwards
                self.draw_cylinder(
                    back_poses[len(back_vecs) - 1 - i],
                    back_vecs[len(back_vecs) - 1 - i],
                    j.length,
                    color="yellow",
                    radius=0.1,
                )
                self.draw_sphere(back_poses[len(back_vecs) - 1 - i], color="light_blue")

                if i < len(forward_vecs):
                    # Debug forwards
                    self.draw_cylinder(
                        forward_poses[i],
                        forward_vecs[i],
                        j.length,
                        color=(0.8, 0.5, 0.1, 1),
                        radius=0.1,
                    )
                    self.draw_sphere(forward_poses[i], color=(0.8, 0.8, 0.1, 1))

            self.draw_sphere(j.position, color="black")
            self.draw_cylinder(
                j.parent_position, j.rot_axis, 2, color="green", radius=0.1
            )
            cross = (j.orientation.cross(j.rot_axis)).abs()
            self.draw_cylinder(j.parent_position, cross, 2, color="red", radius=0.1)
            self.draw_cylinder(
                j.parent_position, j.orientation, j.length, color="gray", radius=0.2
            )
            self.draw_plane(
                j.parent_position, cross, j.orientation, 2, color="light_blue"
            )


j = JointTest(width=500, height=500)


def main():
    while True:
        j.main_loop()


# Call the main function
if __name__ == "__main__":
    main()
