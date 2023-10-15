import numpy as np
import numpy.linalg as la


def get_axis_angle(v1, v2):
    if isinstance(v1, list) or isinstance(v2, list):
        v1 = np.array(v1)
        v2 = np.array(v2)

    # print(f"Cross Value and corresponding vectors : {np.cross(v1, v2), v1, v2}")
    # print(f"The angle value is : {np.arccos(la.multi_dot([v1, v2]) / (la.norm(v1) * la.norm(v2)))}")
    return np.cross(v1, v2), np.arccos(la.multi_dot([v1, v2]) / (la.norm(v1) * la.norm(v2)))


def z_translation_matrix(val):
    k = np.eye(4, 4)
    k[2, 3] = val
    return k


def x_translation_matrix(val):
    k = np.eye(4, 4)
    k[0, 3] = val
    return k


def get_coordinates(d_list, theta_list, a_list, alpha_list, coord_num, intermediate=False, offset=0.0, for_joint=False):
    if coord_num == 0:
        return np.array([0, 0, offset])

    if for_joint:
        d_list = [de + 0.00000001 if de == 0 else de for de in d_list]
        a_list = [de + 0.00000001 if de == 0 else de for de in a_list]

    M1 = np.eye(4, 4)
    for index_i in range(coord_num):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        if not intermediate or index_i != coord_num - 1:
            M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return np.matmul(M1, np.array([0, 0, 0, 1]))[0:3] + np.array([0, 0, offset])


def get_FrameMatrix(d_list, theta_list, a_list, alpha_list, coord_num=1, intermediate=False, offset=0.0):
    M1 = np.eye(4, 4)
    for index_i in range(coord_num):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        if not intermediate or index_i != coord_num - 1:
            M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return np.matmul(M1, np.array([0, 0, 0, 1]))[0:3] + np.array([0, 0, offset]), M1[0:3, 0:3]


def get_RobotLink(p1, p2, link_color=YELLOW_D, radius=0.075, opacity=1.0):
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link_length = la.norm(p2 - p1)
    if link_length < 1e-5:
        return 0

    link = OpenGLSurface(
        lambda u, v: (
            radius * np.cos(u),
            radius * np.sin(u),
            v),
        v_range=[-link_length / 2 - radius * 0.5, link_length / 2 + radius * 0.5], u_range=[0, TAU], color=link_color,
        resolution=(101, 101), opacity=opacity
    )
    covering1 = OpenGLSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            link_length / 2),
        u_range=[0, TAU], v_range=[0, radius], color=link_color, opacity=opacity
    )
    covering2 = OpenGLSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            -link_length / 2),
        u_range=[0, TAU], v_range=[0, radius], color=link_color, opacity=opacity
    )

    link = Group(link, covering1, covering2)
    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)
    return link.rotate(angle=angle, axis=axis, about_point=link.get_center()).move_to((p1 + p2) / 2)


def get_EE(point, radius=0.2, opacity=1.0):
    ee_sphere = OpenGLSurface(
        lambda u, v: (
            radius * np.cos(u) * np.cos(v),
            radius * np.sin(u) * np.cos(v),
            radius * np.sin(v)),
        v_range=[0, TAU], u_range=[0, TAU], color=RED_D, resolution=(101, 101), opacity=opacity
    )

    return ee_sphere.move_to(point)


def get_RobotJoint(p1, p2, joint_color=RED_D, radius=0.1, opacity=1.0):
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link = OpenGLSurface(
        lambda u, v: (
            radius * np.cos(u),
            radius * np.sin(u),
            v),
        v_range=[-radius * 2, radius * 2], u_range=[0, TAU], color=joint_color, resolution=(101, 101), opacity=opacity
    )
    covering1 = OpenGLSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            radius * 2),
        u_range=[0, TAU], v_range=[0, radius], color=joint_color, opacity=opacity
    )
    covering2 = OpenGLSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            -radius * 2),
        u_range=[0, TAU], v_range=[0, radius], color=joint_color, opacity=opacity
    )

    link = Group(link, covering1, covering2)

    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)

    return link.rotate(angle=angle, axis=axis, about_point=link.get_center()).move_to(p1)


def get_Frame(origin, R, scale=1.0, thickness=0.02, opacity=1.0):
    if isinstance(origin, list):
        origin = np.array(origin)
    if R.shape[0] == 4:
        R = R[0:3, 0:3]

    x_axis = np.matmul(R, np.array([1 * scale, 0, 0])) + origin
    y_axis = np.matmul(R, np.array([0, 1 * scale, 0])) + origin
    z_axis = np.matmul(R, np.array([0, 0, 1 * scale])) + origin

    x_arrow = Line3D(start=origin, end=x_axis, thickness=thickness, opacity=opacity).set_color(RED)
    y_arrow = Line3D(start=origin, end=y_axis, thickness=thickness, opacity=opacity).set_color(GREEN)
    z_arrow = Line3D(start=origin, end=z_axis, thickness=thickness, opacity=opacity).set_color(BLUE)

    return Group(x_arrow, y_arrow, z_arrow)


def get_RobotInstance(d_list, theta_list, a_list, alpha_list, offset=-5.0, link_radius=0.175, joint_radius=0.24,
                      link_color=YELLOW_D, joint_color=BLUE_D, opacity=1.0, show_frame=True):
    coord_vec = []
    coord_vecjoint = []
    joint_collection = []
    link_collection = []

    for ri in range(len(d_list) + 1):
        coord_vec.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, True, offset=offset))
        coord_vecjoint.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, True, offset=offset,
                                              for_joint=True))
        if ri > 0:
            jc = RED_D if ri == 1 else joint_color
            joint_collection.append(get_RobotJoint(coord_vecjoint[-2], coord_vecjoint[-1], joint_color=jc,
                                                   radius=joint_radius, opacity=opacity))

            link = get_RobotLink(coord_vec[-2], coord_vec[-1], link_color=link_color, radius=link_radius,
                                 opacity=opacity)
            if not isinstance(link, int):
                link_collection.append(link)

            coord_vec.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, False, offset=offset))
            coord_vecjoint.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, False, offset=offset,
                                                  for_joint=True))
            link = get_RobotLink(coord_vec[-2], coord_vec[-1], link_color=link_color, radius=link_radius,
                                 opacity=opacity)
            if not isinstance(link, int):
                link_collection.append(link)

    link_group = Group()
    joint_group = Group()
    link_group.add(link_collection[0])
    joint_group.add(joint_collection[0])
    ee = get_EE(coord_vec[-1], radius=0.3, opacity=opacity)
    ee_point, ee_R = get_FrameMatrix(d_list, theta_list, a_list, alpha_list, 6, offset=offset)
    ee_frame = get_Frame(ee_point, ee_R, thickness=0.03, opacity=opacity)

    for link_i in range(1, len(link_collection)):
        link_group.add(link_collection[link_i])
    for joint_i in range(1, len(joint_collection)):
        joint_group.add(joint_collection[joint_i])

    if show_frame:
        return Group(joint_group, link_group, ee_frame, ee)
    else:
        return Group(joint_group, link_group, ee)


def get_interpolation(point1, point2, c_iter, total_iter):
    if isinstance(point1, list) or isinstance(point2, list):
        point1 = np.array(point1)
        point2 = np.array(point2)

    return point1 + (point2 - point1) * c_iter / total_iter


def get_interpolation_vector(start, end, via_points=None, total_iterations=100, each_iteration=None):
    total_paths = 2 + len(via_points) if via_points is not None else 2

    if via_points is None or len(via_points) == 0:
        return np.linspace(start, end, total_iterations)
    else:
        each_path_iterations = int(total_iterations / (total_paths - 1)) if each_iteration is None else each_iteration
        via_points.append(end)
        interpolation_path = np.linspace(start, via_points[0], each_path_iterations)
        for tp in range(len(via_points) - 1):
            interpolation_path = np.append(interpolation_path, np.linspace(via_points[tp], via_points[tp + 1],
                                                                           each_path_iterations), axis=0)
        return interpolation_path


def adapt_2Pi(in_val):
    if isinstance(in_val, int) or isinstance(in_val, float):
        if in_val < -PI:
            return in_val + 2 * PI
        elif in_val > PI:
            return in_val - 2 * PI
        else:
            return in_val

    else:
        ret_val = []
        for i in range(len(in_val)):
            if in_val[i] < -PI:
                ret_val.append(in_val[i] + 2 * PI)
            elif in_val[i] > PI:
                ret_val.append(in_val[i] - 2 * PI)
            else:
                ret_val.append(in_val[i])

        return ret_val
