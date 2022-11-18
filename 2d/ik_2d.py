import cv2
import numpy as np
import math

from joints_2d import Joints2D


# mouse callback function
def set_target(event, x, y, flags, param):
    # if event == cv2.EVENT_LBUTTONDBLCLK:
    global mouseX, mouseY, canvas, WIDTH, HEIGHT
    canvas = np.zeros((HEIGHT, WIDTH, 3))
    mouseX = x
    mouseY = y
    canvas = cv2.circle(canvas, (mouseX, mouseY), 10, (0, 1, 1), 8)


cv2.namedWindow("canvas")
cv2.setMouseCallback("canvas", set_target)
WIDTH = 1200
HEIGHT = 800
canvas = np.zeros((HEIGHT, WIDTH, 3))
mouseX = 0
mouseY = 0


def get_dist_2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def get_dist(a, b):
    return abs(a - b)


def get_dist_np(a, b):
    mean_dist = 0
    for i in range(len(a)):
        mean_dist += get_dist(a[i], b[i])
    return mean_dist / len(a)


def random_ik(joints, target):
    best_dist = 100000
    best_angle_dist = 100000
    best_angles = []
    cur_angles = joints.get_current_angles()
    for j in range(50000):
        angles = []
        for i in range(len(joints)):
            import random

            angles.append(random.random() * math.pi * 2)
        angle_dist = get_dist_np(cur_angles, angles)
        joints.set_pose(angles)
        x, y = joints.get_end_effector_pos()
        dist = get_dist_2d([x, y], target)
        if dist < 10 and angle_dist < 0.3:
            return angles
        if dist < best_dist and angle_dist < best_angle_dist:
            best_angles = angles
            best_angle_dist = angle_dist
            best_dist = dist
    return best_angles


joints = Joints2D(
    [WIDTH // 2, HEIGHT // 2],
    [
        [100, 0.5, 0, math.pi],
        [60, 0, 0, 30],
        [60, 0, 0, 60],
        [60, 0, 0, math.pi],
    ],
)
while True:
    canvas = joints.draw(canvas)
    joints.forwards_backwards_ik(mouseX, mouseY, canvas, debug=True)

    cv2.imshow("canvas", 1 - canvas)

    k = cv2.waitKey(1)
    if k == ord("q"):
        break
