import numpy as np
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R
from utils import mat3d_to_homogeneous, get_translation_matrix


class Vec3:
    def from_numpy(np_array):
        return Vec3(
            np_array[0],
            np_array[1],
            np_array[2],
        )

    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    @property
    def elements(self):
        return [self.x, self.y, self.z]

    def cross(self, b):
        return Vec3(
            self.y * b.z - b.y * self.z,
            self.z * b.x - b.z * self.x,
            self.x * b.y - b.x * self.y,
        )

    def __add__(self, b):
        return Vec3(self.x + b.x, self.y + b.y, self.z + b.z)

    def __sub__(self, b):
        return Vec3(self.x - b.x, self.y - b.y, self.z - b.z)

    def __neg__(self):
        return Vec3(-self.x, -self.y, -self.z)

    def __rmul__(self, b):
        return Vec3(self.x * b, self.y * b, self.z * b)

    def __mul__(self, b):  # dot product
        if isinstance(b, Vec3):
            return self.x * b.x + self.y * b.y + self.z * b.z
        elif isinstance(b, float) or isinstance(b, int):
            return Vec3(self.x * b, self.y * b, self.z * b)
        else:
            raise Exception(f"operation not supported between type Vec3 and {type(b)}")

    def __eq__(self, b):
        return self.x == b.x and self.y == b.y and self.z == b.z

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def norm(self):
        d = math.sqrt(self.x**2 + self.y**2 + self.z**2) + 0.0001
        return Vec3(self.x / d, self.y / d, self.z / d)

    def numpy(self):
        return np.array([self.x, self.y, self.z])

    def __str__(self):
        return f"Vec3({self.x},{self.y},{self.z})"

    def abs(self):
        return Vec3(abs(self.x), abs(self.y), abs(self.z))


class Joint:
    def __init__(
        self,
        r,
        alpha,
        d,
        theta,
        color="red",
        joint_color="blue",
    ):
        self.rot_angle = 0

        self.r = r
        self.alpha = alpha
        self.d = d
        self.theta = theta

        self.frame = self.get_frame_from_dh_params(
            self.r, self.alpha, self.d, self.theta
        )
        self.pose = np.eye((4))

        self.color = color
        self.joint_color = joint_color
        self.parent = None

    @property
    def z_axis(self):
        pass

    @property
    def y_axis(self):
        pass

    @property
    def z_axis(self):
        pass

    @property
    def length(self):
        return Vec3(self.frame[0][3], self.frame[1][3], self.frame[2][3]).magnitude()

    @property
    def position(self):
        return Vec3(self.pose[0][3], self.pose[1][3], self.pose[2][3])

    def set_parent(self, j):
        self.parent = j

    def get_frame_from_dh_params(self, r, alpha, d, theta):
        c = math.cos
        s = math.sin
        return np.array(
            [
                [c(theta), -s(theta) * c(alpha), s(theta) * s(alpha), r * c(theta)],
                [s(theta), c(theta) * c(alpha), -c(theta) * s(alpha), r * s(theta)],
                [0, s(alpha), c(alpha), d],
                [0, 0, 0, 1],
            ]
        )

    def forward_kinematics_from_parent(self):
        self.pose = self.parent.pose @ self.frame


class JointChain:
    def __init__(self, root_pos=Vec3(0, 0, 0), joints=[]):
        self._root = Joint(0, 0, 0, 0)

        self._joints = joints
        self.register_parents()

    def register_parents(self):
        base = self._root
        for idx, j in enumerate(self._joints):
            j.set_parent(base)
            base = j

    def propagate_joints(self):
        for idx, j in enumerate(self._joints):
            j.forward_kinematics_from_parent()

    def get_joints(self):
        return self._joints

    def perform_fabrik(self, target):
        # First propagate joints to get current positions
        self.propagate_joints()

        return 0, 0, 0, 0
        # Inverse propagate
        p_target = target
        back_vecs = []
        back_poses = []
        for joint in reversed(self._joints):
            back_vec = p_target - joint.parent.position
            back_vec = joint.project_vector_onto_self_rotational_plane(back_vec)
            back_pos = p_target - (back_vec).norm() * joint.length
            p_target = back_pos

            # DEBUG
            back_vecs.append(back_vec)
            back_poses.append(back_pos)

        # Forwards propagate
        next_root = self._joints[0].parent.position
        forward_vecs = []
        forward_poses = [next_root]
        for i, p_target_forward in enumerate(reversed([target] + back_poses[:-1])):
            joint = self._joints[i]
            forward_vec = p_target_forward - next_root
            forward_vec = joint.project_vector_onto_self_rotational_plane(forward_vec)

            forward_pos = next_root + (forward_vec).norm() * joint.length
            next_root = forward_pos
            forward_vecs.append(forward_vec)
            forward_poses.append(forward_pos)

        for i, vec in enumerate(forward_vecs):
            joint = self._joints[i]
            # joint.set_target_orientation(vec.norm())

        return back_vecs, back_poses, forward_vecs, forward_poses
