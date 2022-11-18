from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import pygame
from pygame.locals import *
import numpy as np
from joints_3d import Vec3
from scipy.spatial.transform import Rotation as R
import math


class Scene:
    def __init__(self, width, height):
        self.p = None
        self.width = width
        self.height = height
        self.display = (width, height)
        self.zoom = -8
        self.t = 0
        self.init()

    def init(self):
        pygame.init()
        self.screen = pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("FABRIK 3D Test")
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glShadeModel(GL_SMOOTH)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glClearColor(1.0, 1.0, 1.0, 0.0)

        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.5, 0.5, 0.5, 1])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [1.0, 1.0, 1.0, 1])

        self.p = gluNewQuadric()

        glMatrixMode(GL_PROJECTION)
        gluPerspective(45, (self.display[0] / self.display[1]), 0.1, 50.0)

        glMatrixMode(GL_MODELVIEW)
        gluLookAt(0, -8, 0, 0, 0, 0, 0, 0, 1)
        self.viewMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)
        glLoadIdentity()

    def main_loop(self):
        self.paused = False
        self.running = True
        # init mouse movement and center mouse on screen
        displayCenter = [self.screen.get_size()[i] // 2 for i in range(2)]
        mouseMove = [0, 0]
        pygame.mouse.set_pos(displayCenter)
        pygame.mouse.set_visible(False)
        up_down_angle = -990
        while self.running:
            self.t += 0.001
            wheelMove = 0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE or event.key == pygame.K_RETURN:
                        self.running = False
                    if event.key == pygame.K_PAUSE or event.key == pygame.K_p:
                        self.paused = not self.paused
                        pygame.mouse.set_pos(displayCenter)
                if not self.paused:
                    if event.type == pygame.MOUSEWHEEL:
                        wheelMove = event.y
                    if event.type == pygame.MOUSEMOTION:
                        mouseMove = [event.pos[i] - displayCenter[i] for i in range(2)]
                    pygame.mouse.set_pos(displayCenter)
            if not self.paused:
                keypress = pygame.key.get_pressed()

                glMatrixMode(GL_MODELVIEW)
                glLoadIdentity()
                self.zoom += wheelMove
                gluLookAt(0, self.zoom, 0, 0, 0, 0, 0, 0, 1)
                up_down_angle += mouseMove[1] * 0.1
                glRotatef(up_down_angle, 1.0, 0.0, 0.0)

                # Handle keyboard stuff
                glPushMatrix()
                glLoadIdentity()
                if keypress[pygame.K_w]:
                    glTranslatef(0, 0, 0.1)
                if keypress[pygame.K_s]:
                    glTranslatef(0, 0, -0.1)
                if keypress[pygame.K_d]:
                    glTranslatef(-0.1, 0, 0)
                if keypress[pygame.K_a]:
                    glTranslatef(0.1, 0, 0)

                self.handle_keypress(keypress)

                glRotatef(mouseMove[0] * 0.1, 0.0, 1.0, 0.0)
                glMultMatrixf(self.viewMatrix)
                self.viewMatrix = glGetFloatv(GL_MODELVIEW_MATRIX)
                glPopMatrix()
                glMultMatrixf(self.viewMatrix)
                glLightfv(GL_LIGHT0, GL_POSITION, [1, -1, 1, 0])
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

                # Draw Floor
                glPushMatrix()
                glColor4f(0.5, 0.5, 0.5, 1)
                glBegin(GL_QUADS)
                glVertex3f(-15, -15, -2)
                glVertex3f(15, -15, -2)
                glVertex3f(15, 15, -2)
                glVertex3f(-15, 15, -2)
                glEnd()
                glPopMatrix()
                # glLoadIdentity()

                self.render()

                pygame.display.flip()
                pygame.time.wait(1)
        pygame.quit()

    def render(self):
        # To be inherited
        pass

    def handle_keypress(self, keypress):
        # To be inherited
        pass

    def get_color(self, color):
        if color == "red":
            color = (0.5, 0.2, 0.2, 1)
        elif color == "blue":
            color = (0.2, 0.2, 0.5, 1)
        elif color == "green":
            color = (0.2, 0.5, 0.2, 1)
        elif color == "light_blue":
            color = (0.2, 0.5, 0.5, 1)
        elif color == "black":
            color = (0.1, 0.1, 0.1, 1)
        elif color == "yellow":
            color = (0.5, 0.5, 0.1, 1)
        elif color == "gray":
            color = (0.3, 0.3, 0.3, 1)
        else:
            if type(color) == str:
                raise Exception(f"invalid color: {color}")
            color = color
        return color

    def draw_sphere(self, p, color="red"):
        color = self.get_color(color)

        glPushMatrix()
        glTranslatef(p.x, p.y, p.z)
        glColor4f(*color)
        gluSphere(self.p, 1.0, 32, 16)
        glPopMatrix()

    def draw_plane(self, position, up, about, plane_length, color="gray"):
        up *= plane_length
        about *= plane_length
        color = self.get_color(color)
        p1 = up + about + position
        p2 = up - about + position
        p3 = position - up - about
        p4 = about + position - up
        glPushMatrix()
        glColor4f(*color)
        glBegin(GL_QUADS)
        glVertex3f(p1.x, p1.y, p1.z)
        glVertex3f(p2.x, p2.y, p2.z)
        glVertex3f(p3.x, p3.y, p3.z)
        glVertex3f(p4.x, p4.y, p4.z)
        glEnd()
        glPopMatrix()

    def get_rotation_to_orientation(self, orientation):
        up = Vec3(0, 0, 1)
        q = [0, 0, 0, 0]
        a = orientation.cross(up)
        q[0] = a.x
        q[1] = a.y
        q[2] = a.z
        q[3] = (
            math.sqrt((up.magnitude() ** 2) * (orientation.magnitude() ** 2))
            + up * orientation
        )
        r = R.from_quat(q).as_matrix()
        r = [
            [r[0][0], r[0][1], r[0][2], 0],
            [r[1][0], r[1][1], r[1][2], 0],
            [r[2][0], r[2][1], r[2][2], 0],
            [0, 0, 0, 1],
        ]
        return r

    def draw_cylinder(self, position, orientation, length, color="blue", radius=0.2):
        color = self.get_color(color)

        glColor3f(*color)
        glPushMatrix()

        glTranslatef(position.x, position.y, position.z)
        """
        rots = self.get_rotation_to_orientation(orientation)
        glRotatef(rots.x, 1, 0, 0)
        glRotatef(rots.y, 0, 1, 0)
        glRotatef(rots.z, 0, 0, 1)
        """
        glMultMatrixf(self.get_rotation_to_orientation(orientation))
        glTranslatef(0, 0, 0)
        gluCylinder(self.p, radius, radius, length, 200, 200)
        glPopMatrix()
