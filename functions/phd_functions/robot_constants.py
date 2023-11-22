# file to save the robot constants
from numpy import cos, pi, deg2rad

PI = pi


def get_dh_parameters(robot: str):
    if robot == "jaco":
        # DH parameters of Jaco Gen2 (non-spherical wrist) robot are:
        alpha_list = [PI / 2, PI, PI / 2, deg2rad(55), deg2rad(55), PI]
        d_list = [212 / 100, 50 / 100, -1 * cos(alpha_list[1]) * 50 / 100 - 12 / 100, -249.3 / 100, -84.6 / 100,
                  -222.73 / 100]
        a_list = [0, 410 / 100, 0, 0, 0, 0]

    elif robot == "philippe":
        d_list = [0, 1, 0]
        a_list = [1, 2, 1.5]
        alpha_list = [-PI / 2, PI / 2, 0]

    elif robot == "orthogonal3r":
        d_list = [1, 1, 0]
        a_list = [1, 2, 1.5]
        alpha_list = [-PI / 2, PI / 2, 0]

    else:
        raise FileNotFoundError("The robot type is recorded in functions/phd_functions/robot_constants.py")

    return d_list, a_list, alpha_list
