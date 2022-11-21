import numpy as np
import math


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


def mat3d_to_homogeneous(mat):
    return np.array(
        [
            [mat[0][0], mat[0][1], mat[0][2], 0],
            [mat[1][0], mat[1][1], mat[1][2], 0],
            [mat[2][0], mat[2][1], mat[2][2], 0],
            [0, 0, 0, 1],
        ]
    )


def get_rot_matrix(mat):
    return np.array(
        [
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ]
    )


def project_vector_onto_plane(vec, u1, u2):
    """https://www.youtube.com/watch?v=NA0lC3wucG0"""
    a = (vec * u1) / (u1.magnitude() ** 2)
    b = (vec * u2) / (u2.magnitude() ** 2)
    return a * u1 + b * u2


def get_translation_matrix(x=0, y=0, z=0):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])


def get_translation_from_matrix(mat):
    return Vec3(mat[0][3], mat[1][3], mat[2][3])


def get_x_axis(pose):
    return Vec3.from_numpy((get_rot_matrix(pose) @ np.array([1, 0, 0]))).norm()


def get_y_axis(pose):
    return Vec3.from_numpy((get_rot_matrix(pose) @ np.array([0, 1, 0]))).norm()


def get_z_axis(pose):
    return Vec3.from_numpy((get_rot_matrix(pose) @ np.array([0, 0, 1]))).norm()


def project_vector_onto_rotational_plane(vec, pose):
    """https://www.youtube.com/watch?v=NA0lC3wucG0"""
    u1 = get_x_axis(pose)
    u2 = get_y_axis(pose)
    a = (vec * u1) / (u1.magnitude() ** 2)
    b = (vec * u2) / (u2.magnitude() ** 2)
    return a * u1 + b * u2, math.atan2(b, a)
