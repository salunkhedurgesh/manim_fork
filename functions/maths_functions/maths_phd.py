import numpy as np
import numpy.linalg as la

PI = np.pi


def get_axis_angle(v1, v2):
    if type(v1) is list or type(v2) is list:
        v1 = np.array(v1)
        v2 = np.array(v2)

    return np.cross(v1, v2), np.arccos(la.multi_dot([v1, v2]) / (la.norm(v1) * la.norm(v2)))


def rad2deg(val):
    return val * 180 / PI


def deg2rad(val):
    return val * PI / 180


def z_translation_matrix(val):
    k = np.eye(4, 4)
    k[2, 3] = val
    return k


def x_translation_matrix(val):
    k = np.eye(4, 4)
    k[0, 3] = val
    return k


def z_rotation_matrix(in_theta):
    mat_id = np.eye(4, 4)
    mat_id[0, 0] = np.cos(in_theta)
    mat_id[1, 0] = np.sin(in_theta)
    mat_id[0, 1] = - np.sin(in_theta)
    mat_id[1, 1] = np.cos(in_theta)

    return mat_id


def x_rotation_matrix(in_theta):
    mat_id = np.eye(4, 4)
    mat_id[1, 1] = np.cos(in_theta)
    mat_id[2, 1] = np.sin(in_theta)
    mat_id[1, 2] = - np.sin(in_theta)
    mat_id[2, 2] = np.cos(in_theta)

    return mat_id
