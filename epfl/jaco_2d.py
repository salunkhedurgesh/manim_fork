import numpy as np

from functions.phd_functions.functions_epfl import *
from functions.phd_functions.robots_3r import *

from manimlib import *
from manim_slides.slide import Slide, ThreeDSlide


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
                           font_size=36).to_edge(RIGHT, buff=LARGE_BUFF).shift(UP).fix_in_frame()
        jac_cusp1 = TexText(r"""\begin{minipage}{8cm}\begin{itemize} \item Maximum 12 IKS have been found 
        \item The 12 IKS lie in 2 separate aspects \end{itemize}\end{minipage}""", font_size=36).next_to(jac_cusp,
                                                                                                         DOWN).fix_in_frame()

        title = TexText(r"""\begin{minipage}{12cm}\centering Example nonsingular change of solutions \\ 
        The robot changes configuration without the change of sign for $\det(\mathbf{J})$\end{minipage}""",
                        font_size=24).to_edge(DOWN, buff=0.6).fix_in_frame()

        rob_ins = get_robot_instance(theta_list=point_record[0], offset=0, robot_type="jaco")
        rob_ins2 = get_robot_instance(theta_list=point_record[0], opacity=0.25, offset=0, robot_type="jaco")

        # animations
        # self.add(get_background("JACO robot is cuspidal").fix_in_frame())
        self.add(back_plane, line_y_offset)
        self.FadeInFadeOut(jac_cusp, jac_cusp1)
        self.FadeInFadeOut(title)
        self.add(TracedPath(det_point.get_center, stroke_width=6, stroke_color=LIGHT_BROWN).fix_in_frame())
        print("Animating frame orientation, scale and position...\n")
        self.play(frame.animate.set_euler_angles(1.6, 1.15, 0).scale(1.6).shift(np.array([-3.5, -3.5, 2])),
                  run_time=2)
        self.add(rob_ins, rob_ins2)
        ee_trace_2d = Group()
        ee_point, ee_R = get_frame_matrix(theta_list=point_record[0], coord_num=6, offset=0.48, robot_type="jaco")
        ee_point_vec = [ee_point]
        path_iter = 100
        for k in range(path_iter + 2):
            k = min(k, path_iter)
            self.play(ReplacementTransform(det_point,
                                           det_point.move_to(
                                               back_plane.c2p(k * 50 / path_iter,
                                                              my_det6R(start_index, end_index, cur_iter=k,
                                                                       total_iter=path_iter)))), run_time=0.05)
            inter_theta = get_interpolation(point_record[0], point_record[4], k, path_iter)
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_theta, offset=0, robot_type="jaco")),
                      run_time=0.02)

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


class Definition(Scene):
    def construct(self):
        # object definition
        toc = TexText("What is a cuspidal robot?").shift(UP * 2)
        ans = TexText(r"""\begin{minipage}{8 cm} \centering A robot that has multiple inverse kinematic solutions in an 
        \emph{aspect} is defined as a cuspidal robot \end{minipage}""", font_size=36).next_to(toc, DOWN * 1.5)

        assumption1 = TexText(
            r"""\begin{minipage}{8 cm} \centering $\rightarrow$ There are no joint limits \end{minipage}""",
            font_size=36).next_to(ans, DOWN * 3)
        assumption2 = TexText(
            r"""\begin{minipage}{8 cm} \centering $\rightarrow$ Collision constraints are 
            not considered \end{minipage}""", font_size=36).next_to(assumption1, DOWN * 1.2)

        # animations
        # self.add(get_background("Definition"))
        self.play(FadeIn(toc))
        self.wait()
        # self.next_slide()
        self.play(FadeIn(ans))
        self.wait(3)
        # self.next_slide()
        self.play(FadeIn(assumption1), FadeIn(assumption2))
        self.wait(3)
        # self.next_slide()


class Robot2R(Scene):

    def construct(self):
        link_lengths = [2, 3]
        offset = np.array([0, -1, 0])
        theta1, theta2 = 55 * DEGREES, -60 * DEGREES
        l1, l2 = link_lengths
        point1 = np.array([0, 0, 0]) + offset
        point2 = point1 + np.array([l1 * cos(theta1), l1 * sin(theta1), 0])
        ee_point = point2 + np.array([l2 * cos(theta1 + theta2), l2 * sin(theta1 + theta2), 0])
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)
        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point2)
        ee = Dot().move_to(ee_point)

        link1 = Line(point1, point2)
        link2 = Line(point2, ee_point)
        mirror_ee_point = ee_point - offset
        mirroring_line = Line(-3 * mirror_ee_point, 3 * mirror_ee_point, stroke_width=0.5, stroke_color=YELLOW).shift(offset)
        axis_config = dict(include_ticks=True, stroke_color=WHITE)
        plot_joint_space, labels = get_small_plot(edge=LEFT, label=True, xconfig=axis_config, yconfig=axis_config, only_labels=True)
        x_label2 = TexText(r"""$\theta_1$""", font_size=36).next_to(plot_joint_space.x_axis, DOWN * 0.5).shift(RIGHT * 1.5)
        y_label2 = TexText(r"""$\theta_2$""", font_size=36).next_to(plot_joint_space.y_axis).rotate(np.pi / 2).shift(UP * 1.7 + LEFT * 0.8)
        singularity_curves = Group()
        for val in [-PI, 0, PI]:
            singularity_curves.add(plot_joint_space.get_graph(lambda x: val).set_color(color=BLUE_D))

        dot_js = Dot(fill_color=PURPLE).move_to(plot_joint_space.c2p(theta1, theta2))

        # text objects
        elbow_up = TexText("Elbow Up configuration", font_size=42).move_to(np.array([2, 2, 0]))
        elbow_down = TexText("Elbow Down configuration", font_size=42).move_to(np.array([2, -2.5, 0]))
        singularity = TexText(
            r"""\begin{minipage}{5 cm}\centering A singular configuration is met while changing IKS \end{minipage}""",
            font_size=36).to_edge(RIGHT, buff=LARGE_BUFF).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
        singularity_curves_text = TexText("singularity curves").to_edge(LEFT, buff=LARGE_BUFF).shift(UP).add_background_rectangle(color=BLACK, opacity=1)

        # Animations

        # self.add(get_background())
        self.add(link1, link2, joint1, joint2, ee)
        self.add(plot_joint_space, labels, x_label2, y_label2, dot_js, dot_js.copy())
        self.wait()
        self.play(FadeIn(singularity_curves))
        self.wait()
        self.FadeInFadeOut(singularity_curves_text)

        theta1_mirror, theta2_mirror = self.draw_mirror(point1, point2, ee_point, l1, _add_obj=False)
        self.wait(2)
        self.play(ShowCreation(mirroring_line), run_time=3)
        self.wait()
        self.draw_mirror(point1, point2, ee_point, l1, _add_obj=True)
        self.wait()
        self.play(FadeIn(elbow_up))
        self.wait()
        self.play(FadeIn(elbow_down))
        self.wait()
        self.play(FadeOut(plot_joint_space.y_axis))

        pause = False
        last_sign = theta2
        self.add(TracedPath(dot_js.get_center, stroke_width=3, stroke_color=GOLD_A))
        for ii, jj in zip(np.linspace(theta1, theta1_mirror, 40), np.linspace(theta2, theta2_mirror, 40)):
            new_point2 = point1 + np.array([l1 * cos(ii), l1 * sin(ii), 0])
            new_ee_point = new_point2 + np.array([l2 * cos(ii + jj), l2 * sin(ii + jj), 0])
            if last_sign * jj < 0:
                stroke_color = RED_D
                pause = True
            else:
                stroke_color = WHITE
            last_sign = jj
            temp_link1 = Line(point1, new_point2, stroke_color=stroke_color)
            temp_link2 = Line(new_point2, new_ee_point, stroke_color=stroke_color)
            temp_joint2 = joint2.copy().move_to(new_point2)
            temp_ee = ee.copy().move_to(new_ee_point)
            temp_dotjs = dot_js.copy().move_to(plot_joint_space.c2p(ii, jj))
            if pause:
                temp_ee.set_fill(color=RED_D, opacity=1)
            self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip([link1, link2, joint2, ee, dot_js], [temp_link1, temp_link2, temp_joint2, temp_ee, temp_dotjs])], run_time=0.2)
            if pause:
                self.play(Transform(dot_js, dot_js.scale(2)))
                self.play(Transform(dot_js, dot_js.scale(0.5)))
                self.FadeInFadeOut(singularity)
                pause = False

        self.wait(2)
        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


    def draw_mirror(self, point1, point2, ee_point, l1, _add_obj=False):

        alpha = angle_between_vectors(ee_point - point1, point2 - point1)
        beta = angle_between_vectors(RIGHT, ee_point - point1)
        theta1_mirror = -(alpha - beta)
        mirror_point2 = point1 + np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        new_point1 = np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        theta2_mirror = angle_between_vectors(new_point1, ee_point - (point1 + new_point1))

        if not _add_obj:
            return [theta1_mirror, theta2_mirror]

        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(mirror_point2)
        mirror_link1 = DashedLine(point1, mirror_point2)
        mirror_link2 = DashedLine(mirror_point2, ee_point)
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)

        self.play(*[FadeIn(obj) for obj in [mirror_link1, mirror_link2, joint2, joint1]])


class ExplainingConfiguration(Scene):

    def construct(self):
        link_lengths = [2, 3]
        offset = np.array([0, -1, 0])
        theta1, theta2 = 55 * DEGREES, -60 * DEGREES
        l1, l2 = link_lengths
        point1 = np.array([0, 0, 0]) + offset
        point2 = point1 + np.array([l1 * cos(theta1), l1 * sin(theta1), 0])
        ee_point = point2 + np.array([l2 * cos(theta1 + theta2), l2 * sin(theta1 + theta2), 0])
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)
        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point2)
        ee = Dot().move_to(ee_point)

        link1 = Line(point1, point2)
        link2 = Line(point2, ee_point)
        mirror_ee_point = ee_point - offset
        mirroring_line = Line(-3 * mirror_ee_point, 3 * mirror_ee_point, stroke_width=0.5, stroke_color=YELLOW).shift(offset)
        axis_config = dict(include_ticks=True, stroke_color=WHITE)
        plot_joint_space, labels = get_small_plot(edge=LEFT, label=True, xconfig=axis_config, yconfig=axis_config, only_labels=True)
        x_label2 = TexText(r"""$\theta_1$""", font_size=36).next_to(plot_joint_space.x_axis, DOWN * 0.5).shift(RIGHT * 1.5)
        y_label2 = TexText(r"""$\theta_2$""", font_size=36).next_to(plot_joint_space.y_axis).rotate(np.pi / 2).shift(UP * 1.7 + LEFT * 0.8)
        singularity_curves = Group()
        for val in [-PI, 0, PI]:
            singularity_curves.add(plot_joint_space.get_graph(lambda x: val).set_color(color=BLUE_D))

        dot_js = Dot(fill_color=PURPLE).move_to(plot_joint_space.c2p(theta1, theta2))

        # text objects
        elbow_up = TexText("Elbow Up configuration", font_size=42).move_to(np.array([2, 2, 0]))
        elbow_down = TexText("Elbow Down configuration", font_size=42).move_to(np.array([2, -2.5, 0]))
        singularity = TexText(
            r"""\begin{minipage}{5 cm}\centering A singular configuration is met while changing IKS \end{minipage}""",
            font_size=36).to_edge(RIGHT, buff=LARGE_BUFF).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
        singularity_curves_text = TexText("singularity curves").to_edge(LEFT, buff=LARGE_BUFF).shift(UP).add_background_rectangle(color=BLACK, opacity=1)

        # Animations

        # self.add(get_background())
        self.add(dot_js.copy())
        self.add(plot_joint_space, labels, x_label2, y_label2, dot_js, singularity_curves)
        self.remove(plot_joint_space.y_axis)
        self.add(link1, link2, joint1, joint2, ee)
        theta1_mirror, theta2_mirror = self.draw_mirror(point1, point2, ee_point, l1, _add_obj=False)
        self.wait()
        mirrored_robot = self.draw_mirror(point1, point2, ee_point, l1, _add_obj=True, _give_obj=True)
        self.add(mirrored_robot)
        self.wait()
        self.play(FadeIn(elbow_up))
        self.wait()
        self.play(FadeIn(elbow_down))
        self.wait()


        pause = False
        last_sign = theta2
        self.add(TracedPath(dot_js.get_center, stroke_width=3, stroke_color=GOLD_A))
        for ii in np.linspace(theta1, 105 * DEGREES, 20):
            jj = theta2
            new_point2 = point1 + np.array([l1 * cos(ii), l1 * sin(ii), 0])
            new_ee_point = new_point2 + np.array([l2 * cos(ii + jj), l2 * sin(ii + jj), 0])
            new_mirrored_robot = self.draw_mirror(point1, new_point2, new_ee_point, l1, _add_obj=True, _give_obj=True)

            stroke_color = WHITE
            temp_link1 = Line(point1, new_point2, stroke_color=stroke_color)
            temp_link2 = Line(new_point2, new_ee_point, stroke_color=stroke_color)
            temp_joint2 = joint2.copy().move_to(new_point2)
            temp_ee = ee.copy().move_to(new_ee_point)
            temp_dotjs = dot_js.copy().move_to(plot_joint_space.c2p(ii, jj))

            self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip([link1, link2, joint2, ee, dot_js, mirrored_robot], [temp_link1, temp_link2, temp_joint2, temp_ee, temp_dotjs, new_mirrored_robot])], run_time=0.2)

        self.play(elbow_up.animate.shift(LEFT + UP * 1.1), elbow_down.animate.shift(RIGHT + UP))
        self.wait(2)
        self.play(elbow_down.copy().scale(0.8).animate.move_to(plot_joint_space.c2p(0, PI / 2)), elbow_up.copy().scale(0.8).animate.move_to(plot_joint_space.c2p(0, -PI / 2)))
        self.wait()
        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


    def draw_mirror(self, point1, point2, ee_point, l1, _add_obj=False, _give_obj=False):

        alpha = angle_between_vectors(ee_point - point1, point2 - point1)
        beta = angle_between_vectors(RIGHT, ee_point - point1)
        theta1_mirror = -(alpha - beta)
        mirror_point2 = point1 + np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        new_point1 = np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        theta2_mirror = angle_between_vectors(new_point1, ee_point - (point1 + new_point1))

        if not _add_obj and not _give_obj:
            return [theta1_mirror, theta2_mirror]

        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(mirror_point2)
        mirror_link1 = DashedLine(point1, mirror_point2)
        mirror_link2 = DashedLine(mirror_point2, ee_point)
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)

        if _give_obj:
            return Group(mirror_link1, mirror_link2, joint2, joint1)

        self.play(*[FadeIn(obj) for obj in [mirror_link1, mirror_link2, joint2, joint1]])


class ShowEightConfigurations(Scene):

    def construct(self) -> None:

        # images
        image_path = "resources/raster_images/"
        scale_image = 0.67
        image_000 = ImageMobject(image_path + "000.png").scale(scale_image).to_corner(UL)
        image_001 = ImageMobject(image_path + "001.png").scale(scale_image).next_to(image_000, RIGHT)
        image_011 = ImageMobject(image_path + "011.png").scale(scale_image).next_to(image_001, RIGHT)
        image_111 = ImageMobject(image_path + "111.png").scale(scale_image).next_to(image_011, RIGHT)
        image_100 = ImageMobject(image_path + "100.png").scale(scale_image).next_to(image_000, DOWN * 4)
        image_110 = ImageMobject(image_path + "110.png").scale(scale_image).next_to(image_100, RIGHT)
        image_010 = ImageMobject(image_path + "010.png").scale(scale_image).next_to(image_110, RIGHT)
        image_101 = ImageMobject(image_path + "101.png").scale(scale_image).next_to(image_010, RIGHT)

        # texts
        fs = 30
        text_000 = TexText(r"RIGHT-DOWN-FLIP", font_size=fs).next_to(image_000, DOWN)
        text_001 = TexText(r"RIGHT-DOWN-\underline{UNFLIP}", font_size=fs).next_to(image_001, DOWN)
        text_011 = TexText(r"RIGHT-\underline{UP}-UNFLIP", font_size=fs).next_to(image_011, DOWN)
        text_111 = TexText(r"\underline{LEFT}-UP-UNFLIP", font_size=fs).next_to(image_111, DOWN)
        text_100 = TexText(r"LEFT-DOWN-FLIP", font_size=fs).next_to(image_100, DOWN)
        text_110 = TexText(r"LEFT-\underline{UP}-FLIP", font_size=fs).next_to(image_110, DOWN)
        text_010 = TexText(r"\underline{RIGHT}-UP-FLIP", font_size=fs).next_to(image_010, DOWN)
        text_101 = TexText(r"\underline{LEFT}-\underline{DOWN}-\underline{UNFLIP}", font_size=fs).next_to(image_101, DOWN)

        # Animations
        # self.add(image_000, image_001, image_011, image_111, image_100, image_110, image_010, image_101)
        for obj, obj2 in zip([image_000, image_001, image_011, image_111, image_100, image_110, image_010, image_101], [text_000, text_001, text_011, text_111, text_100, text_110, text_010, text_101]):
            self.play(FadeIn(obj), FadeIn(obj2))
            self.wait()

        self.embed()


