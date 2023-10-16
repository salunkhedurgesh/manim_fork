import numpy as np
import sympy as sym
from numpy import cos, sin
from numpy import pi as Pi
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

    det = a3*(c3*s3**2*((np.power(cos(alpha1),  3)*cos(alpha2)**2*sin(alpha2)*a3**2 + cos(alpha1)*cos(alpha2)**2*sin(alpha1)**2*sin(alpha2)*a3**2 + np.power(cos(alpha1),  3)*sin(alpha2)*a3**2 - cos(alpha1)*cos(alpha2)**2*sin(alpha2)*a3**2 + cos(alpha1)*sin(alpha1)**2*sin(alpha2)*a3**2 - cos(alpha1)*sin(alpha2)*a3**2)*s2*c2 + (-cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)**2*a3**2 - cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)**2*a3**2 - np.power(cos(alpha2),  3)*sin(alpha1)*a3**2 + cos(alpha2)*sin(alpha1)*a3**2)*s2) + c3*s3*((-2*np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)**2*a3*d3 - 2*cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)**2*a3*d3 + 2*cos(alpha1)*cos(alpha2)*sin(alpha2)**2*a3*d3)*c2*s2 + (cos(alpha1)**2*sin(alpha1)*sin(alpha2)**2*a2*a3 + np.power(sin(alpha1),  3)*sin(alpha2)**2*a2*a3 + cos(alpha1)*cos(alpha2)*sin(alpha2)*a1*a3 + cos(alpha2)**2*sin(alpha1)*a2*a3 - 2*sin(alpha1)*a2*a3)*c2 + (2*np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a2*a3 + 2*cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a2*a3 - 2*cos(alpha1)*cos(alpha2)*sin(alpha2)*a2*a3)*s2**2 + (-cos(alpha1)**2*cos(alpha2)**2*sin(alpha1)*sin(alpha2)*a3*d3 + cos(alpha1)**2*sin(alpha1)*np.power(sin(alpha2),  3)*a3*d3 - cos(alpha2)**2*np.power(sin(alpha1),  3)*sin(alpha2)*a3*d3 + np.power(sin(alpha1),  3)*np.power(sin(alpha2),  3)*a3*d3 - cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)*a3*d2 - cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)*a3*d2 + 2*cos(alpha2)**2*sin(alpha1)*sin(alpha2)*a3*d3 - sin(alpha1)*sin(alpha2)*a3*d3)*s2 - np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a2*a3 - cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a2*a3 + cos(alpha1)*cos(alpha2)*sin(alpha2)*a2*a3 + cos(alpha2)**2*sin(alpha1)*a1*a3 - sin(alpha1)*a1*a3) + c3*((np.power(cos(alpha1),  3)*np.power(sin(alpha2),  3)*d3**2 + cos(alpha1)*sin(alpha1)**2*np.power(sin(alpha2),  3)*d3**2 - np.power(cos(alpha1),  3)*sin(alpha2)*a2**2 - np.power(cos(alpha1),  3)*sin(alpha2)*a3**2 - cos(alpha1)*sin(alpha1)**2*sin(alpha2)*a2**2 - cos(alpha1)*sin(alpha1)**2*sin(alpha2)*a3**2 - cos(alpha1)*np.power(sin(alpha2),  3)*d3**2 + cos(alpha1)*sin(alpha2)*a2**2 + cos(alpha1)*sin(alpha2)*a3**2)*c2*s2 + (cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)*a2*d3 + cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)*a2*d3 + cos(alpha1)**2*sin(alpha1)*sin(alpha2)*a2*d2 + np.power(sin(alpha1),  3)*sin(alpha2)*a2*d2 - cos(alpha1)*sin(alpha2)**2*a1*d3 - cos(alpha2)*sin(alpha1)*sin(alpha2)*a2*d3)*c2 + (-2*np.power(cos(alpha1),  3)*sin(alpha2)**2*a2*d3 - 2*cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a2*d3 + 2*cos(alpha1)*sin(alpha2)**2*a2*d3)*s2**2 + (cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)**2*d3**2 + cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)**2*d3**2 + cos(alpha1)**2*sin(alpha1)*sin(alpha2)**2*d2*d3 + np.power(sin(alpha1),  3)*sin(alpha2)**2*d2*d3 - cos(alpha2)*sin(alpha1)*sin(alpha2)**2*d3**2 + cos(alpha1)*sin(alpha2)*a1*a2)*s2 + np.power(cos(alpha1),  3)*sin(alpha2)**2*a2*d3 + cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a2*d3 - cos(alpha1)*sin(alpha2)**2*a2*d3 - cos(alpha2)*sin(alpha1)*sin(alpha2)*a1*d3) + np.power(s3,3)*((-cos(alpha1)**2*sin(alpha1)*sin(alpha2)**2*a3**2 - np.power(sin(alpha1),  3)*sin(alpha2)**2*a3**2 - cos(alpha2)**2*sin(alpha1)*a3**2 + sin(alpha1)*a3**2)*c2 + (-2*np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a3**2 - 2*cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a3**2 + 2*cos(alpha1)*cos(alpha2)*sin(alpha2)*a3**2)*s2**2 + np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a3**2 + cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a3**2 - cos(alpha1)*cos(alpha2)*sin(alpha2)*a3**2) + s3**2*((2*np.power(cos(alpha1),  3)*sin(alpha2)*a2*a3 + 2*cos(alpha1)*sin(alpha1)**2*sin(alpha2)*a2*a3 - 2*cos(alpha1)*sin(alpha2)*a2*a3)*c2*s2 + (-cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)*a3*d3 - cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)*a3*d3 - cos(alpha1)**2*sin(alpha1)*sin(alpha2)*a3*d2 - np.power(sin(alpha1),  3)*sin(alpha2)*a3*d2 + cos(alpha2)*sin(alpha1)*sin(alpha2)*a3*d3)*c2 + (2*np.power(cos(alpha1),  3)*sin(alpha2)**2*a3*d3 + 2*cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a3*d3 - 2*cos(alpha1)*sin(alpha2)**2*a3*d3)*s2**2 + (-cos(alpha1)*sin(alpha2)*a1*a3 + cos(alpha2)*sin(alpha1)*a2*a3)*s2 - np.power(cos(alpha1),  3)*sin(alpha2)**2*a3*d3 - cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a3*d3 + cos(alpha1)*sin(alpha2)**2*a3*d3) + s3*((cos(alpha1)**2*sin(alpha1)*sin(alpha2)**2*a3**2 + np.power(sin(alpha1),  3)*sin(alpha2)**2*a3**2 + cos(alpha2)**2*sin(alpha1)*a3**2 - sin(alpha1)*a2**2 - sin(alpha1)*a3**2)*c2 + (2*np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a3**2 + 2*cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a3**2 - 2*cos(alpha1)*cos(alpha2)*sin(alpha2)*a3**2)*s2**2 - sin(alpha1)*sin(alpha2)*a2*d3*s2 - np.power(cos(alpha1),  3)*cos(alpha2)*sin(alpha2)*a3**2 - cos(alpha1)*cos(alpha2)*sin(alpha1)**2*sin(alpha2)*a3**2 + cos(alpha1)*cos(alpha2)*sin(alpha2)*a3**2 - sin(alpha1)*a1*a2) + (-2*np.power(cos(alpha1),  3)*sin(alpha2)*a2*a3 - 2*cos(alpha1)*sin(alpha1)**2*sin(alpha2)*a2*a3 + 2*cos(alpha1)*sin(alpha2)*a2*a3)*c2*s2 + (cos(alpha1)**2*cos(alpha2)*sin(alpha1)*sin(alpha2)*a3*d3 + cos(alpha2)*np.power(sin(alpha1),  3)*sin(alpha2)*a3*d3 + cos(alpha1)**2*sin(alpha1)*sin(alpha2)*a3*d2 + np.power(sin(alpha1),  3)*sin(alpha2)*a3*d2 - cos(alpha2)*sin(alpha1)*sin(alpha2)*a3*d3)*c2 + (-2*np.power(cos(alpha1),  3)*sin(alpha2)**2*a3*d3 - 2*cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a3*d3 + 2*cos(alpha1)*sin(alpha2)**2*a3*d3)*s2**2 + cos(alpha1)*sin(alpha2)*a1*a3*s2 + np.power(cos(alpha1),  3)*sin(alpha2)**2*a3*d3 + cos(alpha1)*sin(alpha1)**2*sin(alpha2)**2*a3*d3 - cos(alpha1)*sin(alpha2)**2*a3*d3)

    return det
