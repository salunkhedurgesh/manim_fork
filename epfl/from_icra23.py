from functions.phd_functions.robot_functions import *
from manimlib import *

class AnimateRegular3(ThreeDScene):
    def construct(self):

        df_n = pd.read_csv("resources/data/saved_data_theta1.csv")
        paths = make_paths(df_n, thresh=0.1)

        offset = -2
        ee_point_vec = []
        ee_trace = Group()

        self.camera.frame.scale(1.5)
        self.camera.frame.set_euler_angles(theta=120 * DEGREES, phi=90 * DEGREES)

        first_set = np.array(get_solution(1, paths[1][1], 1))
        rob_ins = get_robot_instance(theta_list=first_set, offset=offset, robot_type="jaco")
        ee_point, ee_R = get_frame_matrix(theta_list=first_set, coord_num=6, offset=offset, robot_type="jaco")
        ee_point_vec.append(ee_point)

        self.add(rob_ins)
        self.add(get_robot_instance(theta_list=first_set, offset=offset,
                                    opacity=0.25, show_frame=True, robot_type="jaco"))
        for anim_iter in range(1, len(paths[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths[1][anim_iter], 1))
            ee_point, ee_R = get_frame_matrix(theta_list=inter_set, coord_num=6, offset=offset, robot_type="jaco")
            ee_point_vec.append(ee_point)
            ee_trace.add(Dot().move_to(ee_point_vec[-1]))
            # ee_trace.add(
            #     Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=WHITE, thickness=0.05, fill_opacity=1))

        self.add(ee_trace)

        for anim_iter in range(1, len(paths[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths[1][anim_iter], 1))
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set, offset=offset,
                                                            show_frame=True, robot_type="jaco")), run_time=0.05)


class AnimateError(ThreeDScene):
    def construct(self):

        # JACO
        d1s = 212
        d3s = -12
        d4s = -249.3
        d5s = -84.6
        d6s = -222.73
        a2s = 410

        # DH parameters of CRX-10ia/L robot are:
        d_list = [d1s, 50, 50 + d3s, d4s, d5s, d6s]
        a_list = [0, a2s, 0, 0, 0, 0]
        alpha_list = [PI / 2, PI, PI / 2, np.deg2rad(55), np.deg2rad(55), PI]
        d_list = [number / 100 for number in d_list]
        a_list = [number / 100 for number in a_list]

        df_n = pd.read_csv(
            "D:/Work/Projects/PhD/manim_durgesh/PhD_thesis/sixR/resources/data/data_jaco/saved_data_theta1.csv")
        paths = make_paths(df_n, thresh=0.1)
        point1_comp = np.array(get_solution(len(paths[2]) - 1, paths[2][len(paths[2]) - 1], 1))
        point5_comp = np.array(get_solution(len(paths[2]) - 1, paths[3][len(paths[2]) - 1], 1))
        start_point = np.array(get_solution(1, paths[2][1], 1))
        theta_list = adapt_2_pi(point1_comp)
        theta_list2 = adapt_2_pi(point5_comp)

        each_iteration = 50
        total_iterations = 50
        offset = -2
        # key_points = [theta_list, theta_list_inter1, theta_list_inter2, theta_list2]
        key_points = [theta_list, theta_list2]

        # theta_list = [0, PI / 2, -PI / 3, PI, 0, 0]
        if key_points is None or len(key_points) < 2:
            raise ValueError("The key points are either empty or only 1 configuration is mentioned\n"
                             "Please mention least 2 arrays of length 6 that will act as the start and end point"
                             "of the trajectory to plot")

        if len(key_points) == 2:
            intermediate_theta = get_interpolation_vector(key_points[0], key_points[1],
                                                          total_iterations=total_iterations,
                                                          each_iteration=each_iteration)
        else:
            via_points = []
            for vi in range(1, len(key_points) - 1):
                via_points.append(key_points[vi])
            intermediate_theta = get_interpolation_vector(key_points[0], key_points[-1],
                                                          via_points=via_points, total_iterations=total_iterations)

        ee_point_vec = []
        ee_trace = Group()

        self.camera.frame.scale(1.5)
        self.camera.frame.set_euler_angles(theta=120 * DEGREES, phi=90 * DEGREES)

        rob_ins = get_robot_instance(d_list, key_points[0], a_list, alpha_list, offset=offset, link_color=RED,
                                     joint_color=RED)
        ee_point, ee_R = get_frame_matrix(d_list, key_points[0], a_list, alpha_list, 6, offset=offset)
        ee_point_vec.append(ee_point)

        self.add(rob_ins)
        self.add(
            get_robot_instance(d_list, start_point, a_list, alpha_list, offset=offset, opacity=0.25, show_frame=True))

        fee_point_vec = []
        full_ee_trace = Group()
        for anim_iter in range(1, len(paths[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths[1][anim_iter], 1))
            fee_point, fee_R = get_frame_matrix(d_list, inter_set, a_list, alpha_list, 6, offset=offset)
            fee_point_vec.append(fee_point)
            full_ee_trace.add(Dot().move_to(fee_point_vec[-1]))
            # ee_trace.add(
            #     Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=WHITE, thickness=0.05, fill_opacity=1))

        self.add(full_ee_trace)

        for anim_iter in range(1, len(intermediate_theta)):
            ee_point, ee_R = get_frame_matrix(d_list, intermediate_theta[anim_iter], a_list, alpha_list,
                                              6, offset=offset)
            ee_point_vec.append(ee_point)
            ee_trace.add(
                Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=WHITE, thickness=0.05, fill_opacity=1))

            self.play(Transform(rob_ins, get_robot_instance(d_list, intermediate_theta[anim_iter], a_list,
                                                            alpha_list, offset=offset, show_frame=True, link_color=RED,
                                                            joint_color=RED)), run_time=0.05)
            self.add(ee_trace)
        self.wait(2)


class AnimateNSCS(ThreeDScene):
    def construct(self):

        # JACO
        d1s = 212
        d3s = -12
        d4s = -249.3
        d5s = -84.6
        d6s = -222.73
        a2s = 410

        # DH parameters of CRX-10ia/L robot are:
        d_list = [d1s, 50, 50 + d3s, d4s, d5s, d6s]
        a_list = [0, a2s, 0, 0, 0, 0]
        alpha_list = [PI / 2, PI, PI / 2, np.deg2rad(55), np.deg2rad(55), PI]
        d_list = [number / 100 for number in d_list]
        a_list = [number / 100 for number in a_list]

        df_n = pd.read_csv("D:/manim_durgesh/PhD_thesis/sixR/resources/data/data_jaco/saved_data_neg_theta1.csv")
        paths2 = make_paths(df_n, thresh=0.1)

        offset = -2
        ee_point_vec = []
        ee_trace = Group()

        self.camera.frame.scale(1.5)
        self.camera.frame.set_euler_angles(theta=120 * DEGREES, phi=90 * DEGREES)

        first_set = np.array(get_solution(1, paths2[1][1], -1))
        rob_ins = get_robot_instance(d_list, first_set, a_list, alpha_list, offset=offset)
        ee_point, ee_R = get_frame_matrix(d_list, first_set, a_list, alpha_list, 6, offset=offset)
        ee_point_vec.append(ee_point)

        self.add(rob_ins)
        self.add(get_robot_instance(d_list, first_set, a_list, alpha_list, offset=offset,
                                    opacity=0.25, show_frame=True))
        for anim_iter in range(1, len(paths2[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths2[1][anim_iter], -1))
            ee_point, ee_R = get_frame_matrix(d_list, inter_set, a_list, alpha_list, 6, offset=offset)
            ee_point_vec.append(ee_point)
            ee_trace.add(Dot().move_to(ee_point_vec[-1]))
            # ee_trace.add(
            #     Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=WHITE, thickness=0.05, fill_opacity=1))

        self.add(ee_trace)

        for anim_iter in range(1, len(paths2[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths2[1][anim_iter], -1))
            self.play(Transform(rob_ins, get_robot_instance(d_list, inter_set, a_list,
                                                            alpha_list, offset=offset, show_frame=True)), run_time=0.05)


class AnimateErrorB(ThreeDScene):
    def construct(self):

        # JACO
        d1s = 212
        d3s = -12
        d4s = -249.3
        d5s = -84.6
        d6s = -222.73
        a2s = 410

        # DH parameters of CRX-10ia/L robot are:
        d_list = [d1s, 50, 50 + d3s, d4s, d5s, d6s]
        a_list = [0, a2s, 0, 0, 0, 0]
        alpha_list = [PI / 2, PI, PI / 2, np.deg2rad(55), np.deg2rad(55), PI]
        d_list = [number / 100 for number in d_list]
        a_list = [number / 100 for number in a_list]

        df_n = pd.read_csv("D:/manim_durgesh/PhD_thesis/sixR/resources/data/data_jaco/saved_data_neg_theta1.csv")
        paths2 = make_paths(df_n, thresh=0.1)

        offset = -2
        ee_point_vec = []
        ee_trace = Group()

        self.camera.frame.scale(1.5)
        self.camera.frame.set_euler_angles(theta=120 * DEGREES, phi=90 * DEGREES)

        first_set = np.array(get_solution(1, paths2[2][1], 1))
        rob_ins = get_robot_instance(d_list, first_set, a_list, alpha_list, offset=offset)
        ee_point, ee_R = get_frame_matrix(d_list, first_set, a_list, alpha_list, 6, offset=offset)
        ee_point_vec.append(ee_point)

        self.add(rob_ins)
        self.add(get_robot_instance(d_list, first_set, a_list, alpha_list, offset=offset,
                                    opacity=0.25, show_frame=True))
        for anim_iter in range(1, len(paths2[1]), 8):
            inter_set = np.array(get_solution(anim_iter, paths2[1][anim_iter], 1))
            ee_point, ee_R = get_frame_matrix(d_list, inter_set, a_list, alpha_list, 6, offset=offset)
            ee_point_vec.append(ee_point)
            ee_trace.add(Dot().move_to(ee_point_vec[-1]))
            # ee_trace.add(
            #     Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=WHITE, thickness=0.05, fill_opacity=1))

        self.add(ee_trace)

        for anim_iter in range(1, len(paths2[2]), 2):
            inter_set = np.array(get_solution(anim_iter, paths2[2][anim_iter], 1))
            self.play(Transform(rob_ins, get_robot_instance(d_list, inter_set, a_list,
                                                            alpha_list, offset=offset, show_frame=True)), run_time=0.05)
