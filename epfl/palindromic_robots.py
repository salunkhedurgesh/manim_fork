from functions.phd_functions.robots_3r import *
from manimlib import *
from manim_slides.slide import Slide, ThreeDSlide

class PalindromicRobot(ThreeDScene):
    """
    Scene to create the animations for square loop for Jaco robot presented in ICRA 2023
    """
    make_slides = True

    def construct(self) -> None:
        # setting frame for slice presentation
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        previous_scale = 1.4
        previous_shift = LEFT * UP
        frame.scale(previous_scale)
        frame.shift(previous_shift)

        # fixed_background = get_background().fix_in_frame()
        circle = Circle().fix_in_frame()  # testing fixed_in_frame object to render the plot by side
        # plotting the slice
        back_plane = NumberPlane(x_range=(-3.4, 0), y_range=(-1.85, 2.50)).set_width(3.4)
        back_plane.shift([-1.7, 0.325, 5.6 + 0.48])
        group_dot = VGroup()

        # animating the robot
        point1 = np.array([20, 40, 10, -70, 30, 0]) * DEGREES
        point2 = np.array([20, 36, 26, -72, 36, 0]) * DEGREES
        point3 = np.array([20, 72, 36, -126, 72, 0]) * DEGREES
        point4 = np.array([20, 72, 36, -170, 170, 0]) * DEGREES
        point5 = np.array([20, 72, 36, 18, 126, 0]) * DEGREES
        point6 = np.array([20, 72, 36, 18, 162, 0]) * DEGREES
        point7 = np.array([-160, 140, -10, 70, 150, 180]) * DEGREES

        matrix_paths = np.array([point1, point2, point3, point4, point5, point6, point7])

        plane_paths = get_path_axis().fix_in_frame()
        line_y_offset = Line(plane_paths.c2p(0, -3), plane_paths.c2p(0, -3.15)).fix_in_frame()
        full_path = Group()
        full_path2 = Group()

        path_interpolation = get_interpolation_vector(point1, point7, via_points=[point2, point3, point4, point5, point6], each_iteration=10)
        path_length = 10
        traj_length = len(path_interpolation) - 1
        for i3 in range(traj_length-1):
            full_path.add(Line(plane_paths.c2p(i3 * path_length / traj_length, path_interpolation[i3][1]),
                                plane_paths.c2p((i3 + 1) * path_length / traj_length, path_interpolation[i3 + 1][1]),
                                stroke_color=RED_D, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

            full_path2.add(Line(plane_paths.c2p(i3 * path_length / traj_length, PI - path_interpolation[i3][1]),
                               plane_paths.c2p((i3 + 1) * path_length / traj_length, PI - path_interpolation[i3 + 1][1]),
                               stroke_color=RED_D, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

        # Points to follow
        point_plot = Dot().move_to(plane_paths.c2p(0, point1[1])).fix_in_frame()
        # point_plot2 = Dot().move_to(plane_paths.c2p(0, point7[1])).fix_in_frame()

        # texts
        repeatable_nscs = TexText(
            r"""\begin{minipage}{5 cm}\centering This is a nonsingular change of solution leading to repeatable path\end{minipage}""",
            font_size=36).to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

        self.add(plane_paths, line_y_offset)
        self.add(full_path)
        self.add(full_path2)

        print("Animating frame orientation, scale and position...\n")
        self.play(frame.animate.set_euler_angles(8.18e-01, 1.15, 0).scale(1.3 / previous_scale).shift(
            np.array([-2, 0.2, 2.3]) - previous_shift), run_time=2)
        rob_ins = get_robot_instance(theta_list=matrix_paths[0], d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0], show_frame=True, offset=0)
        rob_ins_faded = get_robot_instance(theta_list=matrix_paths[0], d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0], show_frame=True, offset=0, opacity=0.25)
        self.add(rob_ins_faded)

        ee_trace_2d = Group()
        ee_point, ee_R = get_frame_matrix(theta_list=matrix_paths[0], coord_num=6, offset=0.48, d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0])
        ee_point_vec = [ee_point]

        self.add(TracedPath(point_plot.get_center, stroke_width=5, stroke_color=GOLD_A).fix_in_frame())
        # self.add(TracedPath(point_plot2.get_center, stroke_width=5, stroke_color=GOLD_A).fix_in_frame())
        jj = 0
        for ii in path_interpolation:
            self.play(Transform(rob_ins, get_robot_instance(theta_list=ii, d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0], show_frame=True, offset=0)), run_time=0.05)

            ee_point, ee_R = get_frame_matrix(theta_list=ii, coord_num=6, offset=0.48, d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0])
            ee_point_vec.append(ee_point)
            # ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=GOLD_D))
            # self.add(ee_trace_2d)
            self.play(*[Transform(obj, obj2) for obj, obj2 in zip([rob_ins, point_plot, ee_trace_2d], [get_robot_instance(theta_list=ii, d_list=[0, 1, -1, 0, 0, 0], a_list=[0, 4, 1, 4/5, 0, 0], alpha_list=[PI/2, 0, 3 * PI / 2, 0, PI / 2, 0], show_frame=True, offset=0), point_plot.move_to(plane_paths.c2p((jj * path_length / traj_length), ii[1])), ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=GOLD_D))])], run_time=0.1)
            # self.play(Transform(point_plot, point_plot.move_to(plane_paths.c2p((jj * 10 / len(path_interpolation)), ii[1])), run_time=0.2))
            # self.play(*[Transform(obj, obj.move_to(plane_paths.c2p((jj * 10 / len(path_interpolation)), obj2))) for obj, obj2 in zip([point_plot, point_plot2], [ii[1], PI - ii[1]])])
            jj += 1

        self.wait()
        self.FadeInFadeOut(repeatable_nscs)
        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])