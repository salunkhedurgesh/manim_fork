import pandas as pd
from functions.maths_functions.maths_phd import *
from functions.phd_functions.robot_constants import *
from manimlib import *


def get_coordinates(d_list, theta_list, a_list, alpha_list, coord_num, intermediate=False, offset=0.0):
    if coord_num == 0:
        return np.array([0, 0, offset])

    M1 = np.eye(4, 4)
    for index_i in range(coord_num):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        if not intermediate or index_i != coord_num - 1:
            M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return np.matmul(M1, np.array([0, 0, 0, 1]))[0:3] + np.array([0, 0, offset])


def get_joint_coordinates(d_list, theta_list, a_list, alpha_list, coord_num, offset=0.0):
    if coord_num == 0:
        return np.array([0, 0, offset])
    elif coord_num == 1:
        return [np.array([0, 0, offset]), np.array([0, 0, offset + 1])]

    M1 = np.eye(4, 4)
    for index_i in range(coord_num - 1):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    origin_joint = np.matmul(M1, np.array([0, 0, 0, 1]))[0:3] + np.array([0, 0, offset])
    end_joint = np.matmul(M1, np.array([0, 0, 1, 1]))[0:3] + np.array([0, 0, offset])
    return [origin_joint, end_joint]




def get_frame_matrix(theta_list, d_list=None, a_list=None, alpha_list=None, coord_num=1, intermediate=False, offset=0, robot_type=None):
    if d_list is None and robot_type is None:
        robot_type = "jaco"
    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    M1 = np.eye(4, 4)
    for index_i in range(coord_num):
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        if not intermediate or index_i != coord_num - 1:
            M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return np.matmul(M1, np.array([0, 0, 0, 1]))[0:3] + np.array([0, 0, offset]), M1[0:3, 0:3]


def get_robot_link(p1, p2, link_color=YELLOW_D, radius=0.075, opacity=1.0):
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link_length = la.norm(p2 - p1)
    if link_length < 1e-5:
        return 0

    link = ParametricSurface(
        lambda u, v: (
            radius * np.cos(u),
            radius * np.sin(u),
            v),
        v_range=(-link_length / 2 - radius * 0.5, link_length / 2 + radius * 0.5), u_range=(0, TAU), color=link_color,
        resolution=(101, 101), opacity=opacity
    )
    covering1 = ParametricSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            link_length / 2),
        u_range=(0, TAU), v_range=(0, radius), color=link_color, opacity=opacity
    )
    covering2 = ParametricSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            -link_length / 2),
        u_range=(0, TAU), v_range=(0, radius), color=link_color, opacity=opacity
    )

    link = Group(link, covering1, covering2)
    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)
    return link.rotate(angle=angle, axis=axis, about_point=link.get_center()).move_to((p1 + p2) / 2)


def get_ee(point, radius=0.2, opacity=1.0):
    return Sphere(radius=radius, opacity=opacity, color=RED_D).move_to(point)


def get_robot_joint(p1, p2, joint_color=RED_D, radius=0.1, opacity=1.0):
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link = ParametricSurface(
        lambda u, v: (
            radius * np.cos(u),
            radius * np.sin(u),
            v),
        v_range=(-radius * 2, radius * 2), u_range=(0, TAU), color=joint_color, resolution=(101, 101), opacity=opacity
    )
    covering1 = ParametricSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            radius * 2),
        u_range=(0, TAU), v_range=(0, radius), color=joint_color, opacity=opacity
    )
    covering2 = ParametricSurface(
        lambda u, v: (
            v * np.cos(u),
            v * np.sin(u),
            -radius * 2),
        u_range=(0, TAU), v_range=(0, radius), color=joint_color, opacity=opacity
    )

    link = Group(link, covering1, covering2)

    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)

    return link.rotate(angle=angle, axis=axis, about_point=link.get_center()).move_to(p1)


def get_frame(origin, R, scale=1.0, thickness=0.02, opacity=1.0):
    if isinstance(origin, list):
        origin = np.array(origin)
    if R.shape[0] == 4:
        R = R[0:3, 0:3]

    x_axis = np.matmul(R, np.array([1 * scale, 0, 0])) + origin
    y_axis = np.matmul(R, np.array([0, 1 * scale, 0])) + origin
    z_axis = np.matmul(R, np.array([0, 0, 1 * scale])) + origin
    x_arrow = Line3D(start=origin, end=x_axis, width=thickness, opacity=opacity).set_color(RED)
    y_arrow = Line3D(start=origin, end=y_axis, width=thickness, opacity=opacity).set_color(GREEN)
    z_arrow = Line3D(start=origin, end=z_axis, width=thickness, opacity=opacity).set_color(BLUE)

    return Group(x_arrow, y_arrow, z_arrow)


def get_robot_instance(theta_list, d_list=None, a_list=None, alpha_list=None, offset=-5.0, link_radius=0.175, joint_radius=0.24,
                       link_color=YELLOW_D, joint_color=BLUE_D, opacity=1.0, show_frame=True, adjust_frame=False, robot_type=None, hide_ee=False,
                       add_plane=False):
    coord_vec, joint_collection, link_collection = list(), list(), list()
    offset = 2 * joint_radius if offset == 0 else offset

    if d_list is None and robot_type is None:
        robot_type = "jaco"
    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    for ri in range(len(d_list) + 1):
        coord_vec.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, True, offset=offset))
        # print(f"ri : {ri}, coord_vec before {coord_vec[-1]}")
        if ri == 0:
            continue
        jc = RED_D if ri == 1 else joint_color
        joint_points = get_joint_coordinates(d_list, theta_list, a_list, alpha_list, ri, offset=offset)
        joint_collection.append(get_robot_joint(joint_points[0], joint_points[1], joint_color=jc,
                                                radius=joint_radius, opacity=opacity))
        link = get_robot_link(coord_vec[-2], coord_vec[-1], link_color=link_color, radius=link_radius,
                                  opacity=opacity)
        coord_vec.append(get_coordinates(d_list, theta_list, a_list, alpha_list, ri, False, offset=offset))
        # print(f"coord_vec after {coord_vec[-1]}")
        link_after = get_robot_link(coord_vec[-2], coord_vec[-1], link_color=link_color, radius=link_radius,
                              opacity=opacity)
        if not isinstance(link, int):
            link_collection.append(link)
        if not isinstance(link_after, int):
            link_collection.append(link_after)

    link_group, joint_group = Group(), Group()
    if hide_ee:
        ee = get_ee(coord_vec[-1], radius=0.003, opacity=opacity)
    else:
        ee = get_ee(coord_vec[-1], radius=0.3, opacity=opacity)

    for link_i in link_collection:
        link_group.add(link_i)
    for joint_i in joint_collection:
        joint_group.add(joint_i)

    if add_plane:
        plane = ParametricSurface(lambda u, v: np.array([u, v, 0]), u_range=(-1, 1), v_range=(-1, 1), color=GREY_E).move_to(coord_vec - np.array([0, 0, 0.]))

    if show_frame:
        ee_point, ee_R = get_frame_matrix(theta_list, d_list, a_list, alpha_list, len(d_list), offset=offset)
        if adjust_frame:
            ee_R = np.matmul(ee_R, x_rotation_matrix(PI)[0:3, 0:3])
        ee_frame = get_frame(ee_point, ee_R, thickness=0.03, opacity=opacity)
        return Group(joint_group, link_group, ee_frame, ee)
    else:
        return Group(joint_group, link_group, ee)


def get_interpolation(point1, point2, c_iter, total_iter):
    return [ii + (jj - ii) * c_iter / total_iter for ii, jj in zip(point1, point2)]


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


def adapt_2_pi(in_val):
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


def make_paths(df, thresh=0.1):
    path_record = []

    for i2 in range(len(df)):
        # print(f"Index {i2} of the dataframe is being processed")
        elements = 0
        for j in range(len(df.columns) - 2):
            if pd.isna(df.iloc[i2, j + 2]):
                break
            else:
                elements += 1
            if i2 == 0: path_record.append([0, df.iloc[0, j + 2]])

        # if i2 > 50:
        #     print(f"The number of elements are {elements}")
        if i2 != 0:
            current_path_index = []
            match_found = []

            for j2 in range(len(path_record)):
                if path_record[j2][0] == i2 - 1:
                    current_path_index.append(j2)

            # print(path_record)
            # print(f"The number of elements are {elements}")

            for k in current_path_index:
                matching_list = None
                threshold = thresh
                for k2 in range(elements):
                    d1 = abs(path_record[k][-1] - df.iloc[i2, k2 + 2])
                    if d1 < threshold:
                        threshold = d1
                        matching_list = [k, k2 + 2]
                        match_found.append(k2)
                if matching_list is not None:
                    path_record[matching_list[0]].append(df.iloc[i2, matching_list[1]])
                    path_record[matching_list[0]][0] = i2

            if len(match_found) != elements:
                for m in range(elements):
                    if m not in match_found:
                        path_record.append([i2, df.iloc[i2, m + 2]])

    return path_record


def get_saved_mat(df, to_save=None, return_data=False):
    df_n = pd.DataFrame([{'index': []}])
    index_search = True
    column_close = False
    first_instance = 1
    column_number = 1
    for i in range(len(df.columns)):
        # Debug started
        # print(df_n)
        # print(f"Index: {i}, started")
        # print(f"The length of df is : {len(df_n)}")
        # Debug ended

        if len(df.columns[i].split('.')) == 3:
            m = df.columns[i].split('.')
            m = m[0] + '.' + m[1]
        else:
            m = df.columns[i]
        first_split = m.split('[')
        if len(first_split) == 1 and index_search is True:
            df_n.loc[len(df_n) - first_instance, 'index'] = int(first_split[0])
            first_instance = 0
            index_search = False

        elif len(first_split) == 2:
            df_n.loc[len(df_n) - 1, column_number] = float(first_split[1])
            column_number += 1
        else:
            second_split = first_split[0].split(']')
            df_n.loc[len(df_n) - 1, column_number] = float(second_split[0])
            if len(second_split) == 2:
                column_close = True
                column_number = 1
            else:
                column_number += 1

        if column_close:
            column_close = False
            index_search = True

    if return_data:
        return df_n
    else:
        df_n.to_csv(to_save)


def make_total_paths(path=""):
    path_record = []
    df1 = get_saved_mat(pd.read_csv(path + "resources/data/icra_vectors/21_9_pos_theta1_8648.csv"),
        return_data=True)
    df2 = get_saved_mat(pd.read_csv(path + "resources/data/icra_vectors/21_9_pos_theta2_8648.csv"),
        return_data=True)
    df3 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_pos_theta3_8648.csv"),
        return_data=True)
    df4 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_pos_theta4_8648.csv"),
        return_data=True)
    df5 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_pos_theta5_8648.csv"),
        return_data=True)
    df6 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_pos_theta6_8648.csv"),
        return_data=True)

    df7 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta1_8648.csv"),
        return_data=True)
    df8 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta2_8648.csv"),
        return_data=True)
    df9 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta3_8648.csv"),
        return_data=True)
    df10 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta4_8648.csv"),
        return_data=True)
    df11 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta5_8648.csv"),
        return_data=True)
    df12 = get_saved_mat(pd.read_csv(
        "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/icra_vectors/21_9_neg_theta6_8648.csv"),
        return_data=True)

    child_df = pd.DataFrame(columns=['instance', 'IKS', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6'])
    child_df2 = pd.DataFrame(columns=['instance', 'IKS', 'theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6'])
    for i2 in range(len(df1)):
        for i3 in np.arange(1, 5):
            child_df.loc[len(child_df)] = [i2 + 1, i3, df1.iloc[i2, i3], df2.iloc[i2, i3], df3.iloc[i2, i3],
                                           df4.iloc[i2, i3], df5.iloc[i2, i3], df6.iloc[i2, i3]]
            child_df2.loc[len(child_df2)] = [i2 + 1, i3, df7.iloc[i2, i3], df8.iloc[i2, i3], df9.iloc[i2, i3],
                                             df10.iloc[i2, i3], df11.iloc[i2, i3], df12.iloc[i2, i3]]

    child_df.to_csv("D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/data_jaco/positive_child.csv")
    child_df2.to_csv("D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/data_jaco/negative_child.csv")


def get_solution(instance, t1_val, det_sign, path=None):
    if path is None: path = ""
    if det_sign == 1:
        df = pd.read_csv(path + "resources/data/positive_child.csv")
    else:
        df = pd.read_csv(path + "resources/data/negative_child.csv")

    df2 = df.loc[df['instance'] == instance]
    cur_diff = 0.5
    m_i = 0
    for k in range(len(df2)):
        if (pd.isna(df2.iloc[k]['theta1']) is False) and abs(t1_val - df2.iloc[k]['theta1']) < cur_diff:
            m_i = k
            cur_diff = abs(t1_val - df2.iloc[k]['theta1'])

    return list(df2[df2['IKS'] == m_i + 1].iloc[0][['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6']])

def get_theta_instance(instance, path=None):
    if path is None: path = ""
    pos_df = pd.read_csv(path + "resources/data/positive_child.csv")
    neg_df = pd.read_csv(path + "resources/data/negative_child.csv")

    return [list(pos_df[pos_df['instance'] == instance]['theta1']), list(neg_df[neg_df['instance'] == instance]['theta1'])]




