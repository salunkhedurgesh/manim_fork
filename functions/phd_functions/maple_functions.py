import numpy as np
import sympy as sym
from numpy import cos, sin
from numpy import pi as Pi
from numpy import pi as PI
from sympy import solve


def return_intermediate(point1, point2, c_iter, total_iter):
    return point1 + (point2 - point1) * c_iter / total_iter


def min_det_traj(d_list, a_list, alpha_list, point1, point2, total_iter=100, print_val=False, isjaco=False):
    det_val_vec = []
    if len(point1) == 6:
        point1f = point1[1:5]
        point2f = point2[1:5]
    else:
        point1f = point1
        point2f = point2

    for lv in range(total_iter):
        intermediate_point = [a + (b - a) * lv / total_iter for a, b in zip(point1f, point2f)]
        det_val = static_pref_jacdet(intermediate_point, d_list, a_list, alpha_list, isjaco=isjaco)
        det_val_vec.append(det_val)

    if print_val:
        print(f"The minimum value for the determinant during this trajectory is {min(det_val_vec)}")
        print(f"The maximum value for the determinant during this trajectory is {max(det_val_vec)}")
        if np.sign(min(det_val_vec)) == np.sign(max(det_val_vec)):
            print("The robot is cuspidal and given trajectory is a non-singular posture change")
        else:
            print("The given trajectory is not a non-singular posture change")

    return [min(det_val_vec), max(det_val_vec)]


def static_pref_jacdet(theta_list, d_list, a_list, alpha_list, isjaco=False):
    a1, a2, a3, a4, a5, a6 = a_list
    d1, d2, d3, d4, d5, d6 = d_list
    alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = alpha_list
    if len(theta_list) == 6:
        theta_list = theta_list[1:5]
    theta2, theta3, theta4, theta5 = theta_list

    if isjaco:
        return jaco_det(theta_list)

    jac_mat = np.array([[(-cos(alpha2) * sin(theta3) * a3 + a1 * sin(theta2) + sin(alpha2) * d3) * cos(alpha1) +
                         cos(theta2) * sin(alpha1) * (sin(alpha2) * sin(theta3) * a3 + cos(alpha2) * d3 + d2),
                         -cos(alpha2) * sin(theta3) * a3 + sin(alpha2) * d3, -sin(theta3) * a3, 0,
                         ((-sin(theta4) * d4 * cos(alpha3) - a4 * sin(alpha3)) * sin(theta3) + cos(theta4) * d4 *
                          cos(theta3)) * sin(alpha4) + a4 * cos(alpha4) * (cos(theta4) * sin(theta3) * cos(alpha3) +
                                                                           sin(theta4) * cos(theta3)),
                         ((((-cos(theta4) * sin(theta5) * d5 - cos(theta5) * sin(theta4) * d4) *
                            cos(alpha4) + (-a4 * cos(theta5) * sin(alpha4) - sin(alpha4) * a5 - sin(theta5) * d4) * cos(
                                     theta4) - cos(theta5) * sin(theta4) * d5) * cos(alpha3) - sin(alpha3) *
                           ((cos(theta5) * a4 + a5) * cos(alpha4) - sin(alpha4) * sin(theta5) * d5)) * sin(theta3) -
                          cos(theta3) * ((-cos(theta4) * cos(theta5) * d4 + sin(theta4) * sin(theta5) * d5) *
                                         cos(alpha4) - cos(theta5) * d5 * cos(theta4) + sin(theta4) * (a4 * cos(theta5)
                                                                                                       * sin(
                                                     alpha4) + sin(alpha4) * a5 + sin(theta5) * d4))) * sin(alpha5) + (
                                 ((cos(theta4) * (
                                         cos(theta5) * a5 + a4) * cos(
                                     alpha4) - sin(
                                     theta4) * (
                                           sin(alpha4) * d4 + sin(theta5) * a5)) * cos(
                                     alpha3) - sin(alpha3) * sin(alpha4) * (cos(theta5) * a5 + a4)) * sin(theta3) + (
                                         sin(theta4) * (cos(theta5) * a5 + a4) * cos(alpha4) + cos(theta4) * (
                                         sin(alpha4) * d4 + sin(theta5) * a5)) * cos(theta3)) * cos(alpha5)],
                        [(-sin(theta2) * d2 * cos(alpha2) - sin(alpha2) * (cos(theta3) * a3 + a2) * cos(
                            theta2) - a1 * sin(alpha2) - sin(theta2) * d3) * sin(alpha1) + cos(alpha1) * cos(
                            alpha2) * (
                                 cos(theta2) * a1 + cos(theta3) * a3 + a2),
                         cos(alpha2) * (cos(theta3) * a3 + a2), cos(theta3) * a3, 0, (
                                 (sin(theta4) * d4 * cos(alpha3) + a4 * sin(alpha3)) * sin(alpha4) - a4 *
                                 cos(theta4) * cos(alpha3) * cos(alpha4)) * cos(
                            theta3) + sin(theta3) * (
                                 a4 * sin(theta4) * cos(alpha4) + cos(theta4) * d4 * sin(alpha4)),
                         ((((cos(theta4) * sin(theta5) * d5 + cos(theta5) * sin(theta4) * d4) * cos(alpha4) +
                            (a4 * cos(theta5) * sin(alpha4) + sin(alpha4) * a5 + sin(theta5) * d4) * cos(theta4) +
                            cos(theta5) * sin(theta4) * d5) * cos(alpha3) + sin(alpha3) * ((cos(theta5) * a4 + a5)
                                                                                           * cos(alpha4) -
                                                                                           sin(alpha4) * sin(theta5)
                                                                                           * d5)) * sin(alpha5) -
                          cos(alpha5) * ((cos(theta4) * (cos(theta5) * a5 + a4) * cos(alpha4) - sin(theta4) *
                                          (sin(alpha4) * d4 + sin(theta5) * a5)) * cos(alpha3) - sin(alpha3) *
                                         sin(alpha4) * (cos(theta5) * a5 + a4))) * cos(theta3) + sin(theta3) *
                         (((cos(theta4) * cos(theta5) * d4 - sin(theta4) * sin(theta5) * d5) * cos(
                             alpha4) + cos(theta5) * d5 * cos(theta4) - sin(theta4) * (
                                   a4 * cos(theta5) * sin(alpha4) + sin(alpha4) * a5 + sin(theta5) * d4)) *
                          sin(alpha5) + (sin(theta4) * (cos(theta5) * a5 + a4) * cos(alpha4) + cos(theta4) *
                                         (sin(alpha4) * d4 + sin(theta5) * a5)) * cos(alpha5))],
                        [(sin(theta2) * d2 * sin(alpha2) + ((-cos(theta3) * a3 - a2) * cos(theta2) - a1) * cos(
                            alpha2) + sin(
                            theta2) * sin(
                            theta3) * a3) *
                         sin(alpha1) - sin(alpha2) * cos(alpha1) * (cos(theta2) * a1 + cos(theta3) * a3 + a2),
                         -sin(alpha2) * (cos(theta3) * a3 + a2), 0, 0,
                         (sin(theta4) * d4 * sin(alpha3) - a4 * cos(alpha3)) * sin(alpha4) - sin(alpha3) * cos(theta4)
                         * cos(alpha4) * a4, (((cos(theta4) * sin(theta5) * d5 + cos(theta5) * sin(theta4) * d4) *
                                               cos(alpha4) + (a4 * cos(theta5) * sin(alpha4) + sin(alpha4) * a5 +
                                                              sin(theta5) * d4) * cos(theta4) + cos(theta5) *
                                               sin(theta4) * d5) * sin(alpha5) - cos(alpha5) * (cos(theta4) * (
                                cos(theta5) * a5 + a4) * cos(
                            alpha4) - sin(theta4) * (sin(alpha4) *
                                                     d4 + sin(
                                    theta5) * a5))) * sin(alpha3) -
                         cos(alpha3) * (((cos(theta5) * a4 + a5) * cos(alpha4) - sin(alpha4) * sin(theta5) * d5) *
                                        sin(alpha5) + cos(alpha5) * sin(alpha4) * (cos(theta5) * a5 + a4))],
                        [sin(alpha1) * sin(theta2), 0, 0, sin(theta3) * sin(alpha3),
                         (sin(alpha4) * cos(alpha3) * cos(theta4) + cos(alpha4) * sin(alpha3)) * sin(theta3) +
                         cos(theta3) * sin(theta4) * sin(alpha4), (((cos(alpha4) * cos(theta4) * cos(theta5) -
                                                                     sin(theta4) * sin(theta5)) * cos(alpha3) -
                                                                    cos(theta5) * sin(alpha3) * sin(alpha4)) * sin(
                            theta3) + cos(theta3) * (cos(theta5) * sin(theta4) * cos(alpha4) + sin(theta5) *
                                                     cos(theta4))) * sin(alpha5) + (
                                 (sin(alpha4) * cos(alpha3) * cos(theta4) + cos(alpha4) * sin(alpha3)) *
                                 sin(theta3) + cos(theta3) * sin(theta4) * sin(alpha4)) * cos(alpha5)],
                        [sin(alpha1) * cos(theta2) * cos(alpha2) + cos(alpha1) * sin(alpha2), sin(alpha2), 0,
                         -cos(theta3) * sin(alpha3),
                         (-cos(alpha3) * cos(theta3) * cos(theta4) + sin(theta4) * sin(theta3)) * sin(
                             alpha4) - cos(theta3) * sin(alpha3) * cos(alpha4), (((-cos(alpha4) *
                                                                                   cos(theta4) * cos(theta5) +
                                                                                   sin(theta4) * sin(theta5)) *
                                                                                  cos(alpha3) + cos(theta5) *
                                                                                  sin(alpha3) * sin(alpha4)) *
                                                                                 sin(alpha5) - cos(alpha5) *
                                                                                 (sin(alpha4) * cos(alpha3) *
                                                                                  cos(theta4) + cos(alpha4) *
                                                                                  sin(alpha3))) * cos(theta3) +
                         sin(theta3) * ((cos(theta5) * sin(theta4) * cos(alpha4) + sin(theta5) * cos(theta4)) * sin(
                            alpha5) + sin(alpha4) * sin(theta4) * cos(alpha5))],
                        [-sin(alpha1) * cos(theta2) * sin(alpha2) + cos(alpha1) * cos(alpha2), cos(alpha2), 1,
                         cos(alpha3), -cos(theta4) * sin(alpha3) * sin(alpha4) + cos(alpha3) * cos(alpha4), (
                                 (-cos(alpha4) * cos(theta4) * cos(theta5) + sin(theta4) * sin(theta5)) *
                                 sin(alpha5) - sin(alpha4) * cos(alpha5) * cos(theta4)) * sin(alpha3) - cos(alpha3) * (
                                 cos(theta5) * sin(alpha4) * sin(alpha5) - cos(alpha4) * cos(alpha5))]])

    return np.linalg.det(jac_mat)


def jaco_det(theta_list):
    if len(theta_list) == 6:
        theta_list = theta_list[1:5]

    theta2, theta3, theta4, theta5 = theta_list
    return (((4.753024540 * sin(theta4) * np.square(cos(theta3)) +
              (-33.59365598 + 5.473415483 * np.square(cos(theta4)) + 20.42658155 * sin(theta3)) * cos(theta3) + (
                      7.816847417 * sin(theta3) - 4.753024540 - 0.1601975263 * cos(theta4)) * sin(theta4)) * sin(
        theta5) + 0.2792958640 * np.square(cos(theta4)) * cos(theta5) - 0.2792958640 * cos(theta5) + 9.542608689 * sin(
        theta4) * cos(theta4) * cos(theta5) * cos(theta3)) * cos(theta2) + (4.753024540 * sin(theta2) * cos(theta3) *
                                                                            sin(theta4) * sin(
                theta3) - 20.42658155 * sin(theta2) * np.square(cos(theta3))) * sin(theta5)) * 100000


def mid_val(point1, point2, sd, i1, i2):
    x_m = (point1[i1] + point2[i1]) / 2
    y_m = (point1[i2] + point2[i2]) / 2
    slope_m = (point2[i2] - point1[i2]) / (point2[i1] - point1[i1])

    # perpendicular line
    x = sym.Symbol('x')
    c2 = y_m + x_m / slope_m
    y = -x / slope_m + c2

    x_sols = solve((x - x_m) ** 2 + (y - y_m) ** 2 - sd ** 2, x)
    y_sols = []
    for i in range(len(x_sols)):
        y_sols.append(y.subs(x, x_sols[i]))

    return x_sols, y_sols


def back_propogation(theta1):
    pass


def adapt_2Pi(in_val):
    if isinstance(in_val, int) or isinstance(in_val, float):
        if in_val < -Pi:
            return in_val + 2 * Pi
        elif in_val > Pi:
            return in_val - 2 * Pi
        else:
            return in_val

    else:
        ret_val = []
        for i in range(len(in_val)):
            if in_val[i] < -Pi:
                ret_val.append(in_val[i] + 2 * Pi)
            elif in_val[i] > Pi:
                ret_val.append(in_val[i] - 2 * Pi)
            else:
                ret_val.append(in_val[i])

        return ret_val


def jac3R(theta_list, d_list, a_list, alpha_list):
    a1, a2, a3 = a_list
    d1, d2, d3 = d_list
    alpha1, alpha2, alpha3 = alpha_list
    if len(theta_list) > 2:
        theta_list = theta_list[1:3]
    theta2, theta3 = theta_list

    s2 = np.sin(theta2)
    c2 = np.cos(theta2)
    s3 = np.sin(theta3)
    c3 = np.cos(theta3)

    det = a3 * (c3 * s3 ** 2 * ((np.power(cos(alpha1), 3) * cos(alpha2) ** 2 * sin(alpha2) * a3 ** 2 + cos(
        alpha1) * cos(alpha2) ** 2 * sin(alpha1) ** 2 * sin(alpha2) * a3 ** 2 + np.power(cos(alpha1), 3) * sin(
        alpha2) * a3 ** 2 - cos(alpha1) * cos(alpha2) ** 2 * sin(alpha2) * a3 ** 2 + cos(alpha1) * sin(
        alpha1) ** 2 * sin(alpha2) * a3 ** 2 - cos(alpha1) * sin(alpha2) * a3 ** 2) * s2 * c2 + (
                                            -cos(alpha1) ** 2 * cos(alpha2) * sin(alpha1) * sin(
                                        alpha2) ** 2 * a3 ** 2 - cos(alpha2) * np.power(sin(alpha1), 3) * sin(
                                        alpha2) ** 2 * a3 ** 2 - np.power(cos(alpha2), 3) * sin(alpha1) * a3 ** 2 + cos(
                                        alpha2) * sin(alpha1) * a3 ** 2) * s2) + c3 * s3 * ((-2 * np.power(cos(alpha1),
                                                                                                           3) * cos(
        alpha2) * sin(alpha2) ** 2 * a3 * d3 - 2 * cos(alpha1) * cos(alpha2) * sin(alpha1) ** 2 * sin(
        alpha2) ** 2 * a3 * d3 + 2 * cos(alpha1) * cos(alpha2) * sin(alpha2) ** 2 * a3 * d3) * c2 * s2 + (
                                                                                                        cos(alpha1) ** 2 * sin(
                                                                                                    alpha1) * sin(
                                                                                                    alpha2) ** 2 * a2 * a3 + np.power(
                                                                                                    sin(alpha1),
                                                                                                    3) * sin(
                                                                                                    alpha2) ** 2 * a2 * a3 + cos(
                                                                                                    alpha1) * cos(
                                                                                                    alpha2) * sin(
                                                                                                    alpha2) * a1 * a3 + cos(
                                                                                                    alpha2) ** 2 * sin(
                                                                                                    alpha1) * a2 * a3 - 2 * sin(
                                                                                                    alpha1) * a2 * a3) * c2 + (
                                                                                                        2 * np.power(
                                                                                                    cos(alpha1),
                                                                                                    3) * cos(
                                                                                                    alpha2) * sin(
                                                                                                    alpha2) * a2 * a3 + 2 * cos(
                                                                                                    alpha1) * cos(
                                                                                                    alpha2) * sin(
                                                                                                    alpha1) ** 2 * sin(
                                                                                                    alpha2) * a2 * a3 - 2 * cos(
                                                                                                    alpha1) * cos(
                                                                                                    alpha2) * sin(
                                                                                                    alpha2) * a2 * a3) * s2 ** 2 + (
                                                                                                        -cos(
                                                                                                            alpha1) ** 2 * cos(
                                                                                                    alpha2) ** 2 * sin(
                                                                                                    alpha1) * sin(
                                                                                                    alpha2) * a3 * d3 + cos(
                                                                                                    alpha1) ** 2 * sin(
                                                                                                    alpha1) * np.power(
                                                                                                    sin(alpha2),
                                                                                                    3) * a3 * d3 - cos(
                                                                                                    alpha2) ** 2 * np.power(
                                                                                                    sin(alpha1),
                                                                                                    3) * sin(
                                                                                                    alpha2) * a3 * d3 + np.power(
                                                                                                    sin(alpha1),
                                                                                                    3) * np.power(
                                                                                                    sin(alpha2),
                                                                                                    3) * a3 * d3 - cos(
                                                                                                    alpha1) ** 2 * cos(
                                                                                                    alpha2) * sin(
                                                                                                    alpha1) * sin(
                                                                                                    alpha2) * a3 * d2 - cos(
                                                                                                    alpha2) * np.power(
                                                                                                    sin(alpha1),
                                                                                                    3) * sin(
                                                                                                    alpha2) * a3 * d2 + 2 * cos(
                                                                                                    alpha2) ** 2 * sin(
                                                                                                    alpha1) * sin(
                                                                                                    alpha2) * a3 * d3 - sin(
                                                                                                    alpha1) * sin(
                                                                                                    alpha2) * a3 * d3) * s2 - np.power(
        cos(alpha1), 3) * cos(alpha2) * sin(alpha2) * a2 * a3 - cos(alpha1) * cos(alpha2) * sin(alpha1) ** 2 * sin(
        alpha2) * a2 * a3 + cos(alpha1) * cos(alpha2) * sin(alpha2) * a2 * a3 + cos(alpha2) ** 2 * sin(
        alpha1) * a1 * a3 - sin(alpha1) * a1 * a3) + c3 * ((np.power(cos(alpha1), 3) * np.power(sin(alpha2),
                                                                                                3) * d3 ** 2 + cos(
        alpha1) * sin(alpha1) ** 2 * np.power(sin(alpha2), 3) * d3 ** 2 - np.power(cos(alpha1), 3) * sin(
        alpha2) * a2 ** 2 - np.power(cos(alpha1), 3) * sin(alpha2) * a3 ** 2 - cos(alpha1) * sin(alpha1) ** 2 * sin(
        alpha2) * a2 ** 2 - cos(alpha1) * sin(alpha1) ** 2 * sin(alpha2) * a3 ** 2 - cos(alpha1) * np.power(sin(alpha2),
                                                                                                            3) * d3 ** 2 + cos(
        alpha1) * sin(alpha2) * a2 ** 2 + cos(alpha1) * sin(alpha2) * a3 ** 2) * c2 * s2 + (
                                                                       cos(alpha1) ** 2 * cos(alpha2) * sin(
                                                                   alpha1) * sin(alpha2) * a2 * d3 + cos(
                                                                   alpha2) * np.power(sin(alpha1), 3) * sin(
                                                                   alpha2) * a2 * d3 + cos(alpha1) ** 2 * sin(
                                                                   alpha1) * sin(alpha2) * a2 * d2 + np.power(
                                                                   sin(alpha1), 3) * sin(alpha2) * a2 * d2 - cos(
                                                                   alpha1) * sin(alpha2) ** 2 * a1 * d3 - cos(
                                                                   alpha2) * sin(alpha1) * sin(
                                                                   alpha2) * a2 * d3) * c2 + (
                                                                       -2 * np.power(cos(alpha1), 3) * sin(
                                                                   alpha2) ** 2 * a2 * d3 - 2 * cos(alpha1) * sin(
                                                                   alpha1) ** 2 * sin(alpha2) ** 2 * a2 * d3 + 2 * cos(
                                                                   alpha1) * sin(alpha2) ** 2 * a2 * d3) * s2 ** 2 + (
                                                                       cos(alpha1) ** 2 * cos(alpha2) * sin(
                                                                   alpha1) * sin(alpha2) ** 2 * d3 ** 2 + cos(
                                                                   alpha2) * np.power(sin(alpha1), 3) * sin(
                                                                   alpha2) ** 2 * d3 ** 2 + cos(alpha1) ** 2 * sin(
                                                                   alpha1) * sin(alpha2) ** 2 * d2 * d3 + np.power(
                                                                   sin(alpha1), 3) * sin(alpha2) ** 2 * d2 * d3 - cos(
                                                                   alpha2) * sin(alpha1) * sin(
                                                                   alpha2) ** 2 * d3 ** 2 + cos(alpha1) * sin(
                                                                   alpha2) * a1 * a2) * s2 + np.power(cos(alpha1),
                                                                                                      3) * sin(
        alpha2) ** 2 * a2 * d3 + cos(alpha1) * sin(alpha1) ** 2 * sin(alpha2) ** 2 * a2 * d3 - cos(alpha1) * sin(
        alpha2) ** 2 * a2 * d3 - cos(alpha2) * sin(alpha1) * sin(alpha2) * a1 * d3) + np.power(s3, 3) * ((-cos(
        alpha1) ** 2 * sin(alpha1) * sin(alpha2) ** 2 * a3 ** 2 - np.power(sin(alpha1), 3) * sin(
        alpha2) ** 2 * a3 ** 2 - cos(alpha2) ** 2 * sin(alpha1) * a3 ** 2 + sin(alpha1) * a3 ** 2) * c2 + (
                                                                                                                     -2 * np.power(
                                                                                                                 cos(alpha1),
                                                                                                                 3) * cos(
                                                                                                                 alpha2) * sin(
                                                                                                                 alpha2) * a3 ** 2 - 2 * cos(
                                                                                                                 alpha1) * cos(
                                                                                                                 alpha2) * sin(
                                                                                                                 alpha1) ** 2 * sin(
                                                                                                                 alpha2) * a3 ** 2 + 2 * cos(
                                                                                                                 alpha1) * cos(
                                                                                                                 alpha2) * sin(
                                                                                                                 alpha2) * a3 ** 2) * s2 ** 2 + np.power(
        cos(alpha1), 3) * cos(alpha2) * sin(alpha2) * a3 ** 2 + cos(alpha1) * cos(alpha2) * sin(alpha1) ** 2 * sin(
        alpha2) * a3 ** 2 - cos(alpha1) * cos(alpha2) * sin(alpha2) * a3 ** 2) + s3 ** 2 * ((2 * np.power(cos(alpha1),
                                                                                                          3) * sin(
        alpha2) * a2 * a3 + 2 * cos(alpha1) * sin(alpha1) ** 2 * sin(alpha2) * a2 * a3 - 2 * cos(alpha1) * sin(
        alpha2) * a2 * a3) * c2 * s2 + (-cos(alpha1) ** 2 * cos(alpha2) * sin(alpha1) * sin(alpha2) * a3 * d3 - cos(
        alpha2) * np.power(sin(alpha1), 3) * sin(alpha2) * a3 * d3 - cos(alpha1) ** 2 * sin(alpha1) * sin(
        alpha2) * a3 * d2 - np.power(sin(alpha1), 3) * sin(alpha2) * a3 * d2 + cos(alpha2) * sin(alpha1) * sin(
        alpha2) * a3 * d3) * c2 + (2 * np.power(cos(alpha1), 3) * sin(alpha2) ** 2 * a3 * d3 + 2 * cos(alpha1) * sin(
        alpha1) ** 2 * sin(alpha2) ** 2 * a3 * d3 - 2 * cos(alpha1) * sin(alpha2) ** 2 * a3 * d3) * s2 ** 2 + (-cos(
        alpha1) * sin(alpha2) * a1 * a3 + cos(alpha2) * sin(alpha1) * a2 * a3) * s2 - np.power(cos(alpha1), 3) * sin(
        alpha2) ** 2 * a3 * d3 - cos(alpha1) * sin(alpha1) ** 2 * sin(alpha2) ** 2 * a3 * d3 + cos(alpha1) * sin(
        alpha2) ** 2 * a3 * d3) + s3 * ((cos(alpha1) ** 2 * sin(alpha1) * sin(alpha2) ** 2 * a3 ** 2 + np.power(
        sin(alpha1), 3) * sin(alpha2) ** 2 * a3 ** 2 + cos(alpha2) ** 2 * sin(alpha1) * a3 ** 2 - sin(
        alpha1) * a2 ** 2 - sin(alpha1) * a3 ** 2) * c2 + (2 * np.power(cos(alpha1), 3) * cos(alpha2) * sin(
        alpha2) * a3 ** 2 + 2 * cos(alpha1) * cos(alpha2) * sin(alpha1) ** 2 * sin(alpha2) * a3 ** 2 - 2 * cos(
        alpha1) * cos(alpha2) * sin(alpha2) * a3 ** 2) * s2 ** 2 - sin(alpha1) * sin(alpha2) * a2 * d3 * s2 - np.power(
        cos(alpha1), 3) * cos(alpha2) * sin(alpha2) * a3 ** 2 - cos(alpha1) * cos(alpha2) * sin(alpha1) ** 2 * sin(
        alpha2) * a3 ** 2 + cos(alpha1) * cos(alpha2) * sin(alpha2) * a3 ** 2 - sin(alpha1) * a1 * a2) + (
                            -2 * np.power(cos(alpha1), 3) * sin(alpha2) * a2 * a3 - 2 * cos(alpha1) * sin(
                        alpha1) ** 2 * sin(alpha2) * a2 * a3 + 2 * cos(alpha1) * sin(alpha2) * a2 * a3) * c2 * s2 + (
                            cos(alpha1) ** 2 * cos(alpha2) * sin(alpha1) * sin(alpha2) * a3 * d3 + cos(
                        alpha2) * np.power(sin(alpha1), 3) * sin(alpha2) * a3 * d3 + cos(alpha1) ** 2 * sin(
                        alpha1) * sin(alpha2) * a3 * d2 + np.power(sin(alpha1), 3) * sin(alpha2) * a3 * d2 - cos(
                        alpha2) * sin(alpha1) * sin(alpha2) * a3 * d3) * c2 + (
                            -2 * np.power(cos(alpha1), 3) * sin(alpha2) ** 2 * a3 * d3 - 2 * cos(alpha1) * sin(
                        alpha1) ** 2 * sin(alpha2) ** 2 * a3 * d3 + 2 * cos(alpha1) * sin(
                        alpha2) ** 2 * a3 * d3) * s2 ** 2 + cos(alpha1) * sin(alpha2) * a1 * a3 * s2 + np.power(
        cos(alpha1), 3) * sin(alpha2) ** 2 * a3 * d3 + cos(alpha1) * sin(alpha1) ** 2 * sin(
        alpha2) ** 2 * a3 * d3 - cos(alpha1) * sin(alpha2) ** 2 * a3 * d3)

    return det


def my_det(t2, t3):
    inter_val = [0, t2, t3]
    # return -3 * (3 * cos(inter_val[2]) + 4) * (
    #             sin(inter_val[2]) * (2 + 4 * cos(inter_val[1])) - 2 * cos(inter_val[1]) * cos(inter_val[2]))

    return jac3R(inter_val, [0, 1, 0], [1, 2, 3 / 2], [np.pi / 2, np.pi / 2, 0]) * 8


def my_det6R(s, e, cur_iter=0, total_iter=50):
    point1_comp = [-3.1201, 0.7082, 1.4904, 2.62, -1.9637, -1.8817]
    point2_comp = [3.0675, 1.0545, 1.3090, 2.4283, -1.2305, -2.3002]
    point3_comp = [2.9132, 0.2824, 1.9297, -2.1648, -2.9685, -2.7165]
    point4_comp = [2.5338, 1.2022, 1.1149, 1.5839, 0.6030, 2.6322]
    point5_comp = [2.4730, 0.0943, 2.0281, -1.4916, -2.4244, 2.4362]
    point6_comp = [2.4335, 0.0936, 1.5741, 1.4311, 2.3452, 0.5391]
    point7_comp = [-0.8579, 3.0408, 1.5721, -1.5912, 2.1625, 0.5390]
    point8_comp = [-0.8046, 3.0466, 1.1135, 1.5103, -2.2697, 2.4394]
    point9_comp = [-0.7501, 1.9399, 2.0268, -1.4270, 0.6212, 2.6291]
    point10_comp = [-0.2812, 2.0346, 1.8631, -0.5833, -1.0917, -2.4130]
    point11_comp = [-0.2456, 2.8156, 1.3090, 0.4882, -2.8301, -2.3003]
    point12_comp = [-0.1583, 2.7025, 1.4699, -0.0656, -2.5402, -1.9078]

    point_record = [point1_comp, point2_comp, point3_comp, point4_comp, point5_comp, point6_comp, point7_comp,
                    point8_comp, point9_comp, point10_comp, point11_comp, point12_comp]
    # JACO
    d1s = 212
    d3s = -12
    d4s = -249.3
    d5s = -84.6
    d6s = -222.73
    a2s = 410

    # DH parameters of CRX-10ia/L robot are:
    d_list = [d1s, 0, d3s, d4s, d5s, d6s]
    a_list = [0, a2s, 0, 0, 0, 0]
    alpha_list = [PI / 2, PI, PI / 2, np.deg2rad(55), np.deg2rad(55), PI]
    d_list = [number / 100 for number in d_list]
    a_list = [number / 100 for number in a_list]

    theta_list = return_intermediate(np.array(point_record[s]), np.array(point_record[e]), cur_iter, total_iter)

    return static_pref_jacdet(theta_list, d_list, a_list, alpha_list)
