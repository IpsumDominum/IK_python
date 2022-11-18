import math
import cv2
import numpy as np


class Joint2D:
    def __init__(self, length, initial_angle, lower_limit=0, upper_limit=math.pi):
        self.length = length
        self.angle = initial_angle
        self.lower_limit = lower_limit
        self.upper_limit = upper_limit

    def clamp_angle(self, angle):
        return max(self.lower_limit, min(self.upper_limit, angle))

    def set_angle(self, angle):
        self.angle = angle


class Joints2D:
    def __init__(self, joint_root_pos, joints_list):
        self._joint_root_pos = joint_root_pos
        self._joints = [Joint2D(j[0], j[1], j[2], j[3]) for j in joints_list]

    def set_pose(self, angles):
        assert len(angles) == len(self._joints)
        for idx, angle in enumerate(angles):
            self._joints[idx].set_angle(angle)

    def __len__(self):
        return len(self._joints)

    def set_joint_pose(self, joint_idx, angle):
        self._joints[joint_idx].set_angle(angle)

    def normalize(self, v):
        norm = math.sqrt(v[0] ** 2 + v[1] ** 2)
        return [v[0] / (norm + 0.0001), v[1] / (norm + 0.0001)]

    def get_direction_vec(self, p1, p2):
        return self.normalize([p1[0] - p2[0], p1[1] - p2[1]])

    def forwards_backwards_ik(self, goal_x, goal_y, canvas=None, debug=False):

        joint_pos = self.get_joint_pos()
        forward_points = []
        target_x = goal_x
        target_y = goal_y
        # forwards (backwards from end effector)
        for i, pos in enumerate(reversed(joint_pos)):
            direction_vector = self.get_direction_vec([target_x, target_y], pos)
            joint = self._joints[
                len(self._joints) - 1 - i
            ]  # start from second to last joint
            next_point = [
                int(target_x - direction_vector[0] * joint.length),
                int(target_y - direction_vector[1] * joint.length),
            ]
            if i < len(joint_pos) - 1:
                forward_points.append(next_point)
                if debug:
                    canvas = cv2.circle(
                        canvas, (next_point[0], next_point[1]), 7, (0, 0, 1), 3
                    )

            target_x = next_point[0]
            target_y = next_point[1]

        backwards_points = []
        # backwards (forwards from root)
        angles = []
        next_pos = joint_pos[0]
        forward_points = list(reversed(forward_points))

        for i in range(len(forward_points) + 1):
            if i < len(forward_points) - 1:
                target = forward_points[i]
            else:
                target = [goal_x, goal_y]

            joint = self._joints[i]
            direction_vector = self.get_direction_vec(target, next_pos)
            new_next_pos = [
                int(next_pos[0] + direction_vector[0] * joint.length),
                int(next_pos[1] + direction_vector[1] * joint.length),
            ]

            # get actual rotation
            direction_angle = math.atan2(
                new_next_pos[1] - next_pos[1], new_next_pos[0] - next_pos[0]
            )
            """
            new_next_pos = [
                int(next_pos[0] + math.cos(direction_angle) * joint.length),
                int(next_pos[1] + math.sin(direction_angle) * joint.length),
            ]
            """
            angles.append(direction_angle)
            backwards_points.append(new_next_pos)

            next_pos = new_next_pos

            if debug:
                canvas = cv2.circle(
                    canvas, (next_pos[0], next_pos[1]), 5, (1, 0.5, 1), -1
                )

        for i, a in enumerate(angles):
            self._joints[i].set_angle(a)

    def get_joint_pos(self):
        joint_pos = []
        parent_x = self._joint_root_pos[0]
        parent_y = self._joint_root_pos[1]
        for j in self._joints:
            joint_pos.append([parent_x, parent_y])
            j_x = int(parent_x + j.length * math.cos(j.angle))
            j_y = int(parent_y + j.length * math.sin(j.angle))
            parent_x = j_x
            parent_y = j_y
        return joint_pos

    def get_end_effector_pos(self):
        parent_x = self._joint_root_pos[0]
        parent_y = self._joint_root_pos[1]
        for j in self._joints:
            j_x = int(parent_x + j.length * math.cos(j.angle))
            j_y = int(parent_y + j.length * math.sin(j.angle))
            parent_x = j_x
            parent_y = j_y
        return parent_x, parent_y

    def get_current_angles(self):
        return [j.angle for j in self._joints]

    def draw_line_between(self, canvas, j_x, j_y, parent_x, parent_y):
        angle = math.atan((j_x - parent_x) / (j_y - parent_y + 0.000001))
        l = 5
        p1 = [
            int(parent_x + l * math.cos(angle)),
            int(parent_y + l * math.sin(angle)),
        ]
        p2 = [
            int(parent_x - l * math.cos(angle)),
            int(parent_y - l * math.sin(angle)),
        ]
        p3 = [int(j_x - l * math.cos(angle)), int(j_y - l * math.sin(angle))]
        p4 = [int(j_x + l * math.cos(angle)), int(j_y + l * math.sin(angle))]

        cv2.polylines(
            canvas, np.array([[p1, p2, p3, p4]], dtype=np.int32), True, (1, 1, 1)
        )
        return canvas

    def draw(self, canvas):
        parent_x = self._joint_root_pos[0]
        parent_y = self._joint_root_pos[1]
        for j in self._joints:
            j_x = int(parent_x + j.length * math.cos(j.angle))
            j_y = int(parent_y + j.length * math.sin(j.angle))

            canvas = cv2.circle(canvas, (j_x, j_y), 10, (1, 1, 1), -1)
            canvas = self.draw_line_between(canvas, j_x, j_y, parent_x, parent_y)

            parent_x = j_x
            parent_y = j_y
        canvas = cv2.circle(
            canvas,
            (self._joint_root_pos[0], self._joint_root_pos[1]),
            20,
            (1, 1, 0),
            -1,
        )
        return canvas
