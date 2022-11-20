import numpy as np


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
        return np.array([
            [mat[0][0], mat[0][1], mat[0][2]],
            [mat[1][0], mat[1][1], mat[1][2]],
            [mat[2][0], mat[2][1], mat[2][2]],
        ])

def project_vector_onto_plane(vec, u1, u2):
    """https://www.youtube.com/watch?v=NA0lC3wucG0"""
    a = (vec * u1) / (u1.magnitude() ** 2)
    b = (vec * u2) / (u2.magnitude() ** 2)
    return a * u1 + b * u2


def get_translation_matrix(x=0, y=0, z=0):
    return np.array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
