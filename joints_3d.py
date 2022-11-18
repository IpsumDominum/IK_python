import numpy as np
import math
from pyquaternion import Quaternion
from scipy.spatial.transform import Rotation as R


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
        length,
        orientation=Vec3(0, 0, 0),
        rot_axis=Vec3(0, 0, 0),
        color="red",
        joint_color="blue",
    ):
        self.position = Vec3(0, 0, 0)  # position is inferred from parents
        self.parent_position = self.position
        self.parent_orientation = orientation.norm()
        self.base_orientation = orientation.norm()
        self.base_rot_axis = rot_axis.norm()
        self.parent_rot_axis = rot_axis.norm()
        self.length = length
        self.theta = 0
        self.color = color
        self.joint_color = joint_color

    @property
    def orientation(self):
        return self.get_cur_orientation()

    @property
    def rot_axis(self):
        return self.get_cur_rot_axis()

    def project_vector_onto_self_rotational_plane(self, vec):
        """https://www.youtube.com/watch?v=NA0lC3wucG0"""
        u1 = (self.orientation.cross(self.rot_axis)).abs()
        u2 = self.orientation
        a = (vec * u1) / (u1.magnitude() ** 2)
        b = (vec * u2) / (u2.magnitude() ** 2)
        return a * u1 + b * u2

    def get_cur_rot_axis(self):
        return (self.parent_orientation + self.base_rot_axis).norm()

    def get_cur_orientation(self):
        quat = Quaternion(
            axis=[self.rot_axis.x, self.rot_axis.y, self.rot_axis.z], angle=self.theta
        )
        o = quat.rotate(self.base_orientation.numpy())
        return Vec3(o[0], o[1], o[2])

    def get_position_from_parent(self):
        cur_orientation = self.get_cur_orientation()
        self.position = Vec3(
            self.parent_position.x + cur_orientation.x * self.length,
            self.parent_position.y + cur_orientation.y * self.length,
            self.parent_position.z + cur_orientation.z * self.length,
        )


class JointChain:
    def __init__(self, root=Vec3(0, 0, 0), joints=[]):
        self._root = root
        self._joints = joints
        self.propagate_joints()

    def propagate_joints(self):
        base_position = self._root
        base_orientation = Vec3(0, 0, 0)
        base_rot_axis = Vec3(0, 0, 0)
        for idx, j in enumerate(self._joints):
            j.parent_position = base_position
            j.parent_orientation = base_orientation
            j.parent_rot_axis = base_rot_axis
            j.get_position_from_parent()
            base_position = j.position
            base_orientation = j.orientation
            base_rot_axis = j.rot_axis

    def get_joints(self):
        return self._joints

    def perform_fabrik(self, target):
        # First propagate joints to get current positions
        self.propagate_joints()

        # Inverse propagate
        p_target = target
        back_vecs = []
        back_poses = []
        for joint in reversed(self._joints):
            back_vec = p_target - joint.parent_position
            back_vec = joint.project_vector_onto_self_rotational_plane(back_vec)
            back_pos = p_target - (back_vec).norm() * joint.length
            p_target = back_pos

            # DEBUG
            back_vecs.append(back_vec)
            back_poses.append(back_pos)

        # Forwards propagate
        next_root = self._joints[0].parent_position
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
