from functions.phd_functions.robot_functions import *
from functions.phd_functions.functions_epfl import *
from manimlib import *
from manim_slides.slide import Slide, ThreeDSlide
import pandas as pd


class JacDetPlot(ThreeDScene):
    CONFIG = {
        "axis_config": {
            "numbers_to_exclude": None
        }
    }

    def construct(self):
        # setting frame for slice presentation
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(0.9)

        # object definition
        # NumberPlane 2
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
        start_index = 0
        end_index = 4
        print("Started the min det traj stuff")
        y_min = min(0,
                    min_det_traj(d_list, a_list, alpha_list, point_record[start_index], point_record[end_index], 50)[0])
        if y_min == 0:
            y_max = min_det_traj(d_list, a_list, alpha_list, point_record[start_index], point_record[end_index], 50)[1]
        else:
            y_max = 0
        print(f"Ended the min det traj stuff {y_min, y_max}")

        back_plane = Axes(x_range=(0, 50, 10), y_range=(-2.7, -0.2, 0.5), width=6, height=5.5).shift(LEFT * 3)
        x_label2 = TexText("path", font_size=30, color=YELLOW_D).next_to(back_plane.x_axis, DOWN)
        y_label2 = TexText("$\det(\mathbf{J})$", font_size=40, color=YELLOW_D).next_to(back_plane.y_axis).rotate(
            np.pi / 2).scale(0.8).shift(LEFT * 1.7)
        print(back_plane.y_axis.get_stroke_width())
        line_y_offset = Line(back_plane.c2p(0, -0.2), back_plane.c2p(0, 0), stroke_width=2).fix_in_frame()
        back_plane.add(x_label2, y_label2)
        back_plane.add_coordinate_labels(
            font_size=20,
            num_decimal_places=1,
            buffers=[-0.3, 0.3],
            excluding=[-1.1],
        )
        back_plane.fix_in_frame()
        det_point = Dot(color=LIGHT_BROWN).move_to(
            back_plane.c2p(0, my_det6R(start_index, end_index, cur_iter=0, total_iter=50))).fix_in_frame()

        jac_cusp = TexText(r"""\begin{minipage}{8cm}\centering JACO robot is a cuspidal robot\end{minipage}""",
                           font_size=36).to_edge(RIGHT).shift(UP).fix_in_frame()
        jac_cusp1 = TexText(r"""\begin{minipage}{8cm}\begin{itemize} \item Maximum 12 IKS have been found 
        \item The 12 IKS lie in 2 separate aspects \end{itemize}\end{minipage}""", font_size=36).next_to(jac_cusp,
                                                                                                         DOWN).fix_in_frame()

        title = TexText(r"""\begin{minipage}{12cm}\centering Example nonsingular change of solutions \\ 
        The robot changes configuration without the change of sign for $\det(\mathbf{J})$\end{minipage}""",
                        font_size=24).to_edge(DOWN, buff=0.6).fix_in_frame()

        rob_ins = get_robot_instance(theta_list=point_record[0], offset=0, robot_type="jaco")
        rob_ins2 = get_robot_instance(theta_list=point_record[0], opacity=0.25, offset=0, robot_type="jaco")

        # animations
        self.add(get_background("JACO robot is cuspidal").fix_in_frame())
        self.add(back_plane, line_y_offset)
        self.FadeInFadeOut(jac_cusp, jac_cusp1)
        self.FadeInFadeOut(title)
        self.add(TracedPath(det_point.get_center, stroke_width=8, stroke_color=LIGHT_BROWN).fix_in_frame())
        print("Animating frame orientation, scale and position...\n")
        self.play(frame.animate.set_euler_angles(1.6, 1.15, 0).scale(1.6).shift(np.array([-3.5, -3.5, 2])),
                  run_time=2)
        self.add(rob_ins, rob_ins2)
        ee_trace_2d = Group()
        ee_point, ee_R = get_frame_matrix(theta_list=point_record[0], coord_num=6, offset=0.48, robot_type="jaco")
        ee_point_vec = [ee_point]
        path_iter = 300
        for k in range(path_iter + 2):
            k = min(k, path_iter)
            self.play(ReplacementTransform(det_point,
                                           det_point.move_to(
                                               back_plane.c2p(k * 50 / path_iter, my_det6R(start_index, end_index, cur_iter=k,
                                                                          total_iter=path_iter)))), run_time=0.02)
            inter_theta = get_interpolation(point_record[0], point_record[4], k, path_iter)
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_theta, offset=0, robot_type="jaco")), run_time=0.02)

            ee_point, ee_R = get_frame_matrix(theta_list=inter_theta, coord_num=6, offset=0.48, robot_type="jaco")
            ee_point_vec.append(ee_point)
            ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1]))
            self.add(ee_trace_2d)

        self.wait(2)
        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[ReplacementTransform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


class RobotTrial(ThreeDScene):

    def construct(self) -> None:
        point1_comp = [-3.1201, 0.7082, 1.4904, 2.62, -1.9637, -1.8817]
        rob_ins = get_robot_instance(theta_list=point1_comp, offset=0, robot_type="jaco")

        self.add(rob_ins)
        self.embed()