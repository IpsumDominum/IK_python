import numpy as np
import math
from utils import Vec3
import random

# Reference:
# Inverse Kinematics Survey http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf?fbclid=IwAR0KLAgGkpl3S9o4gEnWrMch9OrDAPZNgHrum5vr4OeQXy8kFHl7bhJho-c
# DH parameters https://www.youtube.com/watch?v=nuB_7BkYNMk&t=50s


def get_rot_matrix(mat):
    return np.array(
        [
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ]
    )


class Joint:
    def __init__(
        self,
        parent_frame,
        child_frame,
        rot_axis,
        matrix,
        name="",
        lower=-math.pi * 2,  # lower range for angle rotation
        upper=math.pi * 2,  # upper range for angle rotation
        demand_lower=0,  # lower range for actual demand
        demand_upper=1,  # upper range for actual demand
    ):
        self.rot_angle = 0

        self.parent = None
        self.name = name

        self.child_frame = child_frame
        self.parent_frame = parent_frame
        self.matrix = matrix
        self.rot_axis = rot_axis
        self.pose = self.parent_frame

        self.lower = lower
        self.upper = upper
        self.demand_lower = demand_lower
        self.demand_upper = demand_upper

        self.rot_angle = min(self.upper, max(self.lower, self.rot_angle))

    def map_rotation_angle_to_demand(self, angle):
        angle = min(self.upper, max(self.lower, self.rot_angle))
        # From 0-1
        if self.upper - self.lower == 0:
            normalized_value = 0
        else:
            normalized_value = (angle - self.lower) / (self.upper - self.lower)
        demand_value = self.demand_lower + self.demand_upper * normalized_value
        return demand_value

    @property
    def length(self):
        return Vec3(self.frame[0][3], self.frame[1][3], self.frame[2][3]).magnitude()

    @property
    def position(self):
        v = Vec3(self.pose[0][3], self.pose[1][3], self.pose[2][3])
        return v

    def rotate(self, angle):
        self.rot_angle = min(self.upper, max(self.lower, angle))

    def set_parent(self, j):
        self.parent = j

    @property
    def x_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([1, 0, 0]))).norm()

    @property
    def y_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([0, 1, 0]))).norm()

    @property
    def z_axis(self):
        return Vec3.from_numpy((get_rot_matrix(self.pose) @ np.array([0, 0, 1]))).norm()

    @property
    def orientation(self):
        if self.parent:
            return (self.position - self.parent.position).norm()
        else:
            return Vec3(0, 0, 1)

    def project_vector_onto_self_rotational_plane(self, vec):
        """https://www.youtube.com/watch?v=NA0lC3wucG0"""
        u1 = self.x_axis
        u2 = self.y_axis
        a = (vec * u1) / (u1.magnitude() ** 2)
        b = (vec * u2) / (u2.magnitude() ** 2)
        return a * u1 + b * u2, math.atan2(b, a)

    def get_rot_angle_to(self, vec):
        """https://www.youtube.com/watch?v=NA0lC3wucG0"""
        u1 = self.x_axis
        u2 = self.y_axis
        a = (vec * u1) / (u1.magnitude() ** 2)
        b = (vec * u2) / (u2.magnitude() ** 2)
        return math.atan2(b, a)

    def project_vector_onto_self_transformation(self, vec):
        """https://reader.elsevier.com/reader/sd/pii/S1000936121002910?token=3CC7C9EAB41E474AFB63C60EC4AEEDE6C27EC025F52A5D6CAEC0E21C58CE019BA8127930DF9C9E4A420EC5B3566A909B&originRegion=eu-west-1&originCreation=20221120160212"""
        P = np.array(
            [
                [1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 0, 0],
                [0, 0, 0, 1],
            ]
        )
        return Vec3.from_numpy(
            np.linalg.inv(self.pose)
            @ P
            @ self.pose
            @ np.array([vec.x, vec.y, vec.z, 1])
        )

    def get_frame_from_dh_params(self, r, alpha, d, theta):
        # Frame from DH params is constructed from multiplying
        # 4 translation/rotation matrices
        # From the previous joint, rotate theta degrees about the old z axis with radius r
        # Translate joint up by d along the old z axis
        # Rotate z by alpha to find new z axis of current joint
        c = math.cos
        s = math.sin
        T = np.array(
            [
                [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), r * c(theta)],
                [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), r * s(theta)],
                [0, s(alpha), c(alpha), d],
                [0, 0, 0, 1],
            ]
        )
        return T

    def forward_kinematics_from_parent(self):
        self.frame = self.get_frame_from_dh_params(
            self.r, self.alpha, self.d, self.theta + self.rot_angle
        )
        if self.parent:
            self.pose = self.parent.pose @ self.frame
        else:
            self.pose = self.frame


class JointChain:
    def __init__(self, joints=[], links=[]):
        self._joints = [
            Joint(parent_frame=j[0], child_frame=j[1], name=j_name)
            for j_name, j in joints.items()
        ]

    def rotate_joint(self, joint_name, angle):
        for j in self._joints:
            if j.name != "" and j.name == joint_name:
                j.rotate(angle)

    def export_to_dh_params(self, output_path):
        to_write = ""
        for j in self._joints:
            to_write += f"{j.r},{j.alpha},{j.d},{j.theta},{j.lower},{j.upper}\n"
        with open(output_path, "w") as file:
            file.write(to_write.strip("\n"))

    def register_parents(self):
        for j in self._joints:
            print(j)
        exit()

    def forward_kinematics(self):
        for idx, j in enumerate(self._joints):
            j.forward_kinematics_from_parent()

    def get_joints(self):
        return self._joints

    def get_mapped_rotation_angles_to_demand(self, angles):
        assert len(angles) + 1 == len(self._joints)
        mapped_demands = {}
        # Skip first joint. Which is root.
        for angle, joint in zip(angles, self._joints[1:]):
            mapped_demands[joint.name] = joint.map_rotation_angle_to_demand(angle)
        return mapped_demands

    def get_jacobian(self, target=None):
        # http://graphics.cs.cmu.edu/nsp/course/15-464/Spring11/handouts/iksurvey.pdf?fbclid=IwAR0KLAgGkpl3S9o4gEnWrMch9OrDAPZNgHrum5vr4OeQXy8kFHl7bhJho-c
        jacobian = np.zeros((3, len(self._joints) - 1))
        if target:
            # Use alternative jacobian
            s_i = target
        else:
            s_i = self._joints[-1].position
        for j in range(1, len(self._joints)):
            if j < len(self._joints):
                v_j = self._joints[j - 1].z_axis.norm()
                ds_dtheta = v_j.cross(s_i - self._joints[j - 1].position)
                jacobian[0][j - 1] = ds_dtheta.x
                jacobian[1][j - 1] = ds_dtheta.y
                jacobian[2][j - 1] = ds_dtheta.z
            else:
                ds_dtheta = 0
                for k in range(3):
                    jacobian[k][j - 1] = ds_dtheta
        return jacobian

    def ik_jacobian_transpose(self, target):
        def clamp_error(e, max_d):
            if e.magnitude() <= max_d:
                return e
            else:
                return e.norm() * max_d

        def get_alpha(e, jacobian):
            update = jacobian @ jacobian.T @ e.numpy()
            return np.dot(e.numpy(), update) / np.dot(update, update)

        self.forward_kinematics()
        jacobian = self.get_jacobian(target)
        e = target - self._joints[-1].position
        e = clamp_error(e, 5)
        alpha = get_alpha(e, jacobian)

        dtheta = alpha * jacobian.T @ e.numpy()
        max_joint_length = 10

        joint_angles = []
        for i, j in enumerate(self._joints[1:]):
            j.rot_angle = max(j.lower, min(j.upper, j.rot_angle + dtheta[i]))
            joint_angles.append(j.rot_angle)
        return joint_angles

    def ik_damped_least_squares(self, target):
        # Start with random pose

        def clamp_error(e, max_d):
            if e.magnitude() <= max_d:
                return e
            else:
                return e.norm() * max_d

        def get_alpha(e, jacobian):
            update = jacobian @ jacobian.T @ e.numpy()
            return np.dot(e.numpy(), update) / np.dot(update, update)

        self.forward_kinematics()
        jacobian = self.get_jacobian(target)
        e = target - self._joints[-1].position
        # e = clamp_error(e, 2)

        Lambda = 1
        # a = jacobian @ jacobian.T + Lambda**2 * np.eye(3)
        # dtheta = jacobian.T @ np.linalg.inv(a) @ e.numpy()
        # SVD
        u, s, vh = np.linalg.svd(jacobian, full_matrices=True)
        # D.T @ (DD.T +  lambda^2 @ I) ^-1  (See reference: Singular Value Decomposition)
        E = np.zeros((8, 3))
        for i in range(3):
            E[i][i] = s[i] / (s[i] ** 2 + Lambda**2)
        dtheta = (vh.T @ E @ u.T) @ e.numpy()

        joint_angles = []
        for i, j in enumerate(self._joints[1:]):
            if i == 2:
                dtheta[i] = min(0.01, dtheta[i] * 0.1)
            elif i > (len(self._joints[1:]) - 3):
                dtheta[i] = min(1, dtheta[i])
            else:
                dtheta[i] = min(0.1, dtheta[i])
            j.rot_angle = max(j.lower, min(j.upper, j.rot_angle + dtheta[i]))
            joint_angles.append(j.rot_angle)
        return joint_angles

    def ik_pseudoinverse(self, target):
        def clamp_error(e, max_d):
            if e.magnitude() <= max_d:
                return e
            else:
                return e.norm() * max_d

        def get_alpha(e, jacobian):
            update = jacobian @ jacobian.T @ e.numpy()
            return np.dot(e.numpy(), update) / np.dot(update, update)

        self.forward_kinematics()
        jacobian = self.get_jacobian(target)
        e = target - self._joints[-1].position
        e = clamp_error(e, 5)

        dtheta = jacobian.T @ np.linalg.pinv(jacobian @ jacobian.T) @ e.numpy()
        joint_angles = []
        for i, j in enumerate(self._joints[1:]):
            j.rot_angle = max(j.lower, min(j.upper, j.rot_angle + dtheta[i]))
            joint_angles.append(j.rot_angle)
        return joint_angles


if __name__ == "__main__":
    min_max = {
        "Clavicle Yaw Right": (-3.0, 29.0),
        "Clavicle Roll Right": (-3.5901036262512207, 29.742788314819336),
        "Shoulder Yaw Right": (-59.0, 49.0),
        "Shoulder Roll Right": (-6.9463348388671875, 83.38965606689453),
        "Shoulder Pitch Right": (0.0, 114.0),
        "Elbow Pitch Right": (-0.07368606328964233, 106.12593078613281),
        "Wrist Yaw Right": (5.0, 175.0),
    }
    joint_chain = JointChain(
        joints=[
            Joint(
                # Clavicle Yaw
                r=0,
                alpha=math.pi / 2,
                d=1.85 * 3,
                theta=0,
                name="Clavicle Yaw Right",
                lower=-0.3,
                upper=0.3,
                demand_lower=min_max["Clavicle Yaw Right"][0],
                demand_upper=min_max["Clavicle Yaw Right"][1],
            ),
            Joint(
                # Clavicle Roll
                r=0.582 * 3,
                alpha=math.pi / 2,
                d=0,
                theta=0,
                name="Clavicle Roll Right",
                lower=-0.1,
                upper=0,
                demand_lower=min_max["Clavicle Roll Right"][0],
                demand_upper=min_max["Clavicle Roll Right"][1],
            ),
            Joint(
                # Intermediate Joint (Not used)
                r=1,
                alpha=0,
                d=1.72 * 3,
                theta=0,
                name="",
                lower=0,
                upper=0,
            ),
            Joint(
                # Shoulder Yaw
                r=0,
                alpha=math.pi / 2,
                d=0.5,
                theta=math.pi,
                name="Shoulder Yaw Right",
                lower=-0.6,
                upper=0.8,
                demand_lower=min_max["Shoulder Yaw Right"][0],
                demand_upper=min_max["Shoulder Yaw Right"][1],
            ),
            Joint(
                # Shoulder roll
                r=0,
                alpha=math.pi / 2,
                d=0.5,
                theta=0,
                name="Shoulder Roll Right",
                lower=-math.pi,
                upper=-math.pi + 1.2,
                demand_lower=min_max["Shoulder Roll Right"][0],
                demand_upper=min_max["Shoulder Roll Right"][1],
            ),
            Joint(
                # Shoulder Pitch
                r=4,
                alpha=0,
                d=0,
                theta=0,
                name="Shoulder Pitch Right",
                lower=0,
                upper=math.pi / 2,
                demand_lower=min_max["Shoulder Pitch Right"][0],
                demand_upper=min_max["Shoulder Pitch Right"][1],
            ),
            Joint(
                # Elbow Pitch
                r=3,
                alpha=math.pi / 2,
                d=0,
                theta=0,
                name="Elbow Pitch Right",
                lower=0,
                upper=math.pi / 2,
                demand_lower=min_max["Elbow Pitch Right"][0],
                demand_upper=min_max["Elbow Pitch Right"][1],
            ),
            Joint(
                # Wrist Yaw
                r=0.1,
                alpha=math.pi / 2,
                d=0.1,
                theta=0,
                name="Wrist Yaw Right",
                lower=0,
                upper=math.pi / 2,
                demand_lower=min_max["Wrist Yaw Right"][0],
                demand_upper=min_max["Wrist Yaw Right"][1],
            ),
        ]
    )
    from viz import Visualization

    viz = Visualization(joint_chain, 800, 800)
    viz.main_loop()
