import numpy as np
from functions.phd_functions.robot_functions import *
from functions.phd_functions.functions_epfl import *
from functions.phd_functions.robots_3r import *
from manimlib import *
from numpy import sin, cos, sqrt
import pandas as pd
from manim_slides.slide import Slide, ThreeDSlide

"""
File where experiments related to the plotting of 2D slice of the workspace of 6R robots will be conducted
"""

dict_color_code = {0: BLACK, 2: BLUE_D, 4: YELLOW_D, 6: PURPLE_D, 8: GREEN_D, 10: PINK,
                   12: "#40E0D0", 14: "#9932cc", 16: RED_D}

slide_dot = Dot(opacity=0, color=BLACK, fill_color=BLACK, stroke_opacity=0).to_corner(DL, buff=0)


def get_text():
    # all texts

    four_choices = TexText(
        r"""\begin{minipage}{5 cm}\centering We have 4 IKS in the same aspect and we can start the 
        trajectory from any one of them \end{minipage}""",
        font_size=36).to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    first_choice = TexText(
        r"""\begin{minipage}{5 cm} \centering Let us start with IKS corresponding to $T_3$\end{minipage}""",
        font_size=36).to_edge(RIGHT, buff=0.5).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    zeroth_problem = TexText(
        r"""\begin{minipage}{5 cm} \centering The path exits from a region with 8 IKS to enter a region with 
        6 IKS and thus lose 2 IKS (1 in each aspect) \end{minipage}""",
        font_size=36).to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    first_problem = TexText(
        r"""\begin{minipage}{5 cm}We do not have a continuous path beyond this point and a sudden jump to 
        any other paths in the same aspect will take place \end{minipage}""",
        font_size=36).to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    zeroth_problem2 = TexText(
        r"""\begin{minipage}{5 cm} \centering The path exits from a region with 6 IKS to enter a region 
        with 4 IKS and further loses 2 IKS (1 in each aspect) \end{minipage}""",
        font_size=36).to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    second_choice = TexText(
        r"""\begin{minipage}{5 cm} \centering Let us start with IKS corresponding to 
        $T_7$\end{minipage}""").to_edge(RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    second_problem = TexText(
        r"""\begin{minipage}{5 cm}Same problem is encountered at this point and a sudden jump to any other paths 
        with a solution at next instance will take place \end{minipage}""", font_size=36).to_edge(
        RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

    third_choice = TexText(
        r"""\begin{minipage}{5 cm} \centering Trajectories  $T_2$ and $T_6$ lead to a continuous path 
        that can be \emph{repeated} \end{minipage}""", font_size=36).to_edge(RIGHT).add_background_rectangle(
        color=BLACK, opacity=0.8).fix_in_frame()
    third_problem = TexText(
        r"""\begin{minipage}{5 cm} \centering This path can be \emph{repeated} because the 
        trajectory ends with the same IKS it started with it \end{minipage}""", font_size=36).to_edge(
        RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
    zeroth_problem3 = TexText(
        r"""\begin{minipage}{5 cm} \centering The path exits from a region with 4 IKS to enter a region
         with 8 IKS and thus gains 4 IKS (2 in each aspect) \end{minipage}""", font_size=36).to_edge(
        RIGHT).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

    return dict({'four_choices': four_choices, 'first_choice': first_choice, 'zeroth_problem': zeroth_problem,
                 'first_problem': first_problem, 'zeroth_problem2': zeroth_problem2, 'second_choice': second_choice,
                 'second_problem': second_problem, 'third_choice': third_choice, 'third_problem': third_problem,
                 'zeroth_problem3': zeroth_problem3})


def get_text_neg():
    nscs_choice = TexText(
        r"""\begin{minipage}{5 cm} \centering Let us start with IKS corresponding to $T_5$\end{minipage}""",
        font_size=36).to_edge(RIGHT, buff=0.5).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

    nscs_continuous = TexText(
        r"""\begin{minipage}{5 cm} \centering These paths are continuous and correspond to nonsingular change of solutions.\end{minipage}""",
        font_size=36).to_edge(RIGHT, buff=0.5).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

    nscs_repeat = TexText(
        r"""\begin{minipage}{5 cm} \centering These paths are not repeatable.\end{minipage}""",
        font_size=36).to_edge(RIGHT, buff=0.5).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()

    return dict({'nscs_choice': nscs_choice, 'nscs_con': nscs_continuous, 'nscs_repeat': nscs_repeat})


def get_path_axis():
    axes = Axes(
        # x-axis ranges from -1 to 10, with a default step size of 1
        x_range=(0, 10),
        # y-axis ranges from -2 to 2 with a step size of 0.5
        y_range=(-3, 3),
        # The axes will be stretched to match the specified
        # height and width
        height=4.5,
        width=7.5,
        # Axes is made of two NumberLine mobjects.  You can specify
        # their configuration with axis_config
        axis_config=dict(
            stroke_color=GREY_A,
            stroke_width=2,
            numbers_to_exclude=[0],
        ),
        # Alternatively, you can specify configuration for just one
        # of them, like this.
        y_axis_config=dict(
            numbers_with_elongated_ticks=[-2, 2],
        ),
        x_axis_config=dict(
            include_ticks=False,
        )
    )
    axes.x_axis.shift(DOWN * axes.y_axis.get_length() * 1.05 / 2)
    axes.shift(LEFT * 2.5)
    x_label2 = TexText("path", font_size=30, color=YELLOW_D).next_to(axes.x_axis, DOWN * 1.8)
    y_label2 = TexText("$\\theta_1 (radians)$", font_size=30, color=YELLOW_D).next_to(axes.y_axis).rotate(
        np.pi / 2).shift(
        LEFT * 1.5).scale(0.8)
    axes.add(x_label2, y_label2)
    axes.add_coordinate_labels(
        font_size=20,
        num_decimal_places=1,
    )
    return axes


def get_value_workspace(instance):
    if instance < 724 / 4:
        return get_interpolation([-2.8, 0], [-2.8, 2], instance - 1, 181)
    elif instance < 724 / 2:
        return get_interpolation([-2.8, 2], [-1.2, 2], instance - 182, 181)
    elif instance < 3 * 724 / 4:
        return get_interpolation([-1.2, 2], [-1.2, 0], instance - 363, 181)
    else:
        return get_interpolation([-1.2, 0], [-2.8, 0], instance - 544, 181)


class JacoSlice1(ThreeDScene):
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
        df = pd.read_csv("resources/data/jaco_slice_zoomed.csv")
        back_plane = NumberPlane(x_range=(-3.4, 0), y_range=(-1.85, 2.50)).set_width(3.4)
        back_plane.shift([-1.7, 0.325, 5.6 + 0.48])
        group_dot = VGroup()
        print(f"length of dataframe is {len(df)}")
        for ii in range(0, len(df), 5):
            if ii % 100 == 0: print(f"done adding Dot at index {ii}")
            group_dot.add(Dot(fill_color=dict_color_code[df['iks'][ii]]).scale(0.19).move_to(back_plane.coords_to_point(
                df['x'][ii] / 100, df['y'][ii] / 100)))

        # animating the robot
        df_n = pd.read_csv("resources/data/saved_data_theta1.csv")
        paths = make_paths(df_n, thresh=0.1)

        first_set = np.array(get_solution(1, paths[1][1], 1))
        robot_top_view = get_robot_instance(theta_list=[-0.2, -2.7, 0.73, -2.59, 2.98, 2.2], offset=0)
        rob_ins = get_robot_instance(theta_list=first_set, offset=0, robot_type="jaco", hide_ee=True)
        faded_robot = get_robot_instance(theta_list=first_set, offset=0,
                                         opacity=0.25, show_frame=True, robot_type="jaco", hide_ee=True)

        ee_trace_2d = Group()
        for xx1, yy1, xx2, yy2 in zip([-2.8, -2.8, -1.2, -1.2], [0, 2, 2, 0], [-2.8, -1.2, -1.2, -2.8], [2, 2, 0, 0]):
            ee_trace_2d.add(Line3D(back_plane.c2p(xx1, yy1), back_plane.c2p(xx2, yy2)))
        ee_trace_2d.shift(UP * 0.05)

        plane_paths = get_path_axis().fix_in_frame()
        line_y_offset = Line(plane_paths.c2p(0, -3), plane_paths.c2p(0, -3.15)).fix_in_frame()
        path_length = 10
        df_n = pd.read_csv("resources/data/saved_data_theta1.csv")
        df_n2 = pd.read_csv("resources/data/saved_data_neg_theta1.csv")
        # full_path = []
        first_x = (len(paths[2]) * path_length) / 724
        second_x = (len(paths[0]) * path_length) / 724
        third_x = ((len(paths[1]) - len(paths[4])) * path_length) / 724

        paths = make_paths(df_n, thresh=0.2)
        full_path2 = Group()
        print(f"number of paths in positive det are {len(paths)}")
        for ii in range(len(paths)):
            print(f"Length of path {ii} is {len(paths[ii])}")
            for i4 in np.arange(1, len(paths[ii]) - 1):
                i3 = paths[ii][0] - len(paths[ii]) + 1 + i4
                full_path2.add(Line(plane_paths.c2p(i3 * path_length / 724, paths[ii][i4]),
                                    plane_paths.c2p((i3 + 1) * path_length / 724, paths[ii][i4 + 1]),
                                    stroke_color=BLUE_C, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

        paths2 = make_paths(df_n2, thresh=0.2)
        full_path22 = Group()
        print(f"number of paths in negative det are {len(paths2)}")
        for ii in range(len(paths2)):
            print(f"Length of path2 {ii} is {len(paths2[ii])}")
            for i4 in np.arange(1, len(paths2[ii]) - 1):
                i3 = paths2[ii][0] - len(paths2[ii]) + 1 + i4
                full_path22.add(Line(plane_paths.c2p(i3 * path_length / 724, paths2[ii][i4]),
                                     plane_paths.c2p((i3 + 1) * path_length / 724, paths2[ii][i4 + 1]),
                                     stroke_color=RED_C, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

        # all dots
        point_plot = Dot().move_to(plane_paths.c2p(0, paths[2][1])).fix_in_frame()
        point_plot2 = Dot().move_to(plane_paths.c2p(0, paths[0][1])).fix_in_frame()
        point_plot3 = Dot(fill_color=GREEN).move_to(plane_paths.c2p(0, paths[1][1])).fix_in_frame()
        point_plot4 = Dot(fill_color=GREEN).move_to(plane_paths.c2p(0, paths[3][1])).fix_in_frame()

        point_plotb = Circle(color=BLUE_C, radius=0.1).move_to(plane_paths.c2p(0, paths[2][1])).fix_in_frame()
        point_plot2b = Circle(color=BLUE_C, radius=0.1).move_to(plane_paths.c2p(0, paths[0][1])).fix_in_frame()
        point_plot3b = Circle(color=BLUE_C, radius=0.1).move_to(plane_paths.c2p(0, paths[1][1])).fix_in_frame()
        point_plot4b = Circle(color=BLUE_C, radius=0.1).move_to(plane_paths.c2p(0, paths[3][1])).fix_in_frame()

        # transition lines
        first_transition = DashedLine(plane_paths.c2p(first_x, -3.5), plane_paths.c2p(first_x, 3.5)).fix_in_frame()
        second_transition = DashedLine(plane_paths.c2p(second_x, -3.5), plane_paths.c2p(second_x, 3.5)).fix_in_frame()
        third_transition = DashedLine(plane_paths.c2p(third_x, -3.5), plane_paths.c2p(third_x, 3.5)).fix_in_frame()

        # transition text
        first_nom = TexText("$(8 \\rightarrow 6)$", font_size=24).move_to(first_transition.get_end()).shift(
            LEFT * 0.5).fix_in_frame()
        second_nom = TexText("$(6 \\rightarrow 4)$", font_size=24).move_to(second_transition.get_end()).shift(
            RIGHT * 0.5).fix_in_frame()
        third_nom = TexText("$(4 \\rightarrow 8)$", font_size=24).move_to(third_transition.get_end()).fix_in_frame()

        t1 = TexText(r"$T_1$", font_size=36).move_to(plane_paths.c2p(7, 3)).fix_in_frame()
        t2 = TexText(r"$T_2$", font_size=36).move_to(plane_paths.c2p(7, 2)).fix_in_frame()
        t3t4 = TexText(r"$T_3$, $T_4$", font_size=36).move_to(plane_paths.c2p(1, 0.5)).fix_in_frame()
        t5 = TexText(r"$T_5$", font_size=36).move_to(plane_paths.c2p(7, 0.5)).fix_in_frame()
        t6 = TexText(r"$T_6$", font_size=36).move_to(plane_paths.c2p(7, -1.1)).fix_in_frame()
        t7t8 = TexText(r"$T_7$, $T_8$", font_size=36).move_to(plane_paths.c2p(1, -2.5)).fix_in_frame()

        # all text objects
        all_texts = get_text()
        show_three = True

        # Animations
        # self.add(fixed_background)
        self.play(*[FadeIn(item) for item in [group_dot, ee_trace_2d]])
        self.wait()
        # # self.wait()
        self.play(FadeIn(robot_top_view))
        self.wait()
        print("Animating frame orientation, scale and position...\n")
        self.play(frame.animate.set_euler_angles(8.18e-01, 1.15, 0).scale(1.3 / previous_scale).shift(
            np.array([-6, -3, 4]) - previous_shift), run_time=2)
        self.add(ee_trace_2d.shift(DOWN * 0.05))
        self.play(FadeIn(slide_dot))
        self.wait()

        self.play(*[FadeIn(item) for item in [plane_paths, line_y_offset]])
        self.play(FadeIn(slide_dot))
        self.wait()
        self.play(ReplacementTransform(robot_top_view, rob_ins), run_time=2)
        self.add(faded_robot)
        slide_error = 16 if JacoSlice1.make_slides else 0
        print("Animating robot path...\n")
        for anim_iter in range(1, len(paths[1]) + slide_error, 8):
            if anim_iter > len(paths[1]) - 1 and not JacoSlice1.make_slides:
                break
            else:
                anim_iter = min(anim_iter, len(paths[1]) - 1)
            inter_set = np.array(get_solution(anim_iter, paths[1][anim_iter], 1))
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set, offset=0,
                                                            show_frame=True, robot_type="jaco", hide_ee=True)),
                      run_time=0.05)
        self.wait()

        # animations for the plots
        print("Animating path fade in...\n")
        self.play(*[FadeIn(path_item) for path_item in [full_path2, full_path22]])
        self.play(*[FadeIn(obj) for obj in [t1, t2, t3t4, t5, t6, t7t8]])
        self.wait()
        self.FadeInFadeOut(all_texts['four_choices'])
        self.FadeIt(*full_path22)
        self.FadeInFadeOut(point_plotb, point_plot2b, point_plot3b, point_plot4b)
        self.FadeInFadeOut(all_texts['first_choice'], wait_time=3)
        self.wait()

        print("Animating trace path 1...\n")
        self.remove(faded_robot)
        first_robot = get_robot_instance(theta_list=np.array(get_solution(1, paths[2][1], 1)), offset=0.48,
                                         show_frame=True, hide_ee=True)
        self.play(Transform(rob_ins, first_robot))

        self.add(TracedPath(point_plot.get_center, stroke_width=5, stroke_color=GOLD_A).fix_in_frame())
        slide_error = 4 if JacoSlice1.make_slides else 0
        for i2 in np.arange(1, len(paths[2]) - 1 + slide_error, 2):
            if i2 > len(paths[2]) - 1 and not JacoSlice1.make_slides:
                break
            else:
                i2 = min(i2, len(paths[2]) - 1)

            inter_set = np.array(get_solution(i2, paths[2][i2], 1))
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set, offset=0.48,
                                                            show_frame=True, hide_ee=True)),
                      run_time=0.05)
            self.play(Transform(point_plot, point_plot.move_to(
                plane_paths.c2p((i2 * path_length / 724), paths[2][i2])), run_time=0.2))
        self.wait()

        self.play(ShowCreation(first_transition))
        self.play(FadeIn(first_nom))
        self.FadeInFadeOut(all_texts['zeroth_problem'], wait_time=5)
        self.FadeInFadeOut(all_texts['first_problem'], wait_time=5)
        self.wait()

        print("Animating trace path 2...\n")
        point_plot.set_fill(RED)
        self.add(TracedPath(point_plot.get_center, stroke_width=5, stroke_color=RED).fix_in_frame())
        last_iter = len(paths[2]) - 1
        prev_start_solution = np.array(get_solution(last_iter - 1, paths[2][last_iter - 1], 1))
        start_solution = np.array(get_solution(last_iter, paths[2][last_iter], 1))
        final_solution = np.array(get_solution(last_iter, paths[3][last_iter], 1))
        ee_trace_2d = Group()
        ee_point, ee_R = get_frame_matrix(theta_list=prev_start_solution, coord_num=6, offset=0.48, robot_type="jaco")
        ee_point_vec = [ee_point]
        error_iter = 20
        slide_error = 2 if JacoSlice1.make_slides else 0
        for i2 in range(error_iter + 1 + slide_error):  # make it + 1 when not using slide
            i2 = min(i2, error_iter)
            inter_set = get_interpolation(start_solution, final_solution, i2, error_iter)
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set,
                                                            offset=0.48, show_frame=True, link_color=RED_D,
                                                            joint_color=RED, hide_ee=True)), run_time=0.05)
            ee_point, ee_R = get_frame_matrix(theta_list=inter_set, coord_num=6, offset=0.48, robot_type="jaco")
            ee_point_vec.append(ee_point)
            ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=RED))
            self.add(ee_trace_2d)

            nh = len(paths[2]) - 1
            eh = paths[3][nh] - paths[2][nh]
            self.play(Transform(point_plot, point_plot.move_to(
                plane_paths.c2p((nh * path_length / 724), paths[2][nh] + (eh * i2 / error_iter))), run_time=0.2))

        i2 = error_iter
        inter_set = get_interpolation(start_solution, final_solution, i2, error_iter)
        self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set,
                                                        offset=0.48, show_frame=True, link_color=RED_D,
                                                        joint_color=RED, hide_ee=True)), run_time=0.05)
        ee_point, ee_R = get_frame_matrix(theta_list=inter_set, coord_num=6, offset=0.48, robot_type="jaco")
        ee_point_vec.append(ee_point)
        ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=RED))
        self.add(ee_trace_2d)

        self.play(FadeIn(slide_dot))
        self.wait()

        self.remove(rob_ins, ee_trace_2d)
        print("Animating trace path 3...\n")
        first_robot_faded = get_robot_instance(theta_list=np.array(get_solution(1, paths[0][1], 1)), offset=0.48,
                                               show_frame=True, hide_ee=True, opacity=0.25)
        rob_ins = get_robot_instance(theta_list=np.array(get_solution(1, paths[0][1], 1)), offset=0.48,
                                     show_frame=True, hide_ee=True)
        self.add(first_robot_faded, rob_ins)

        self.add(
            TracedPath(point_plot2.get_center, stroke_width=5, stroke_color=GOLD_A, time_per_anchor=0.2).fix_in_frame())
        self.FadeInFadeOut(all_texts['second_choice'], wait_time=3)
        slide_error = 4 if JacoSlice1.make_slides else 0
        for i2 in np.arange(1, len(paths[0]) - 1 + slide_error, 4):
            if i2 >= len(paths[0]) and not JacoSlice1.make_slides:
                break
            else:
                i2 = min(i2, len(paths[0]) - 1)
            inter_set = np.array(get_solution(i2, paths[0][i2], 1))
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set, offset=0.48,
                                                            show_frame=True, hide_ee=True)), run_time=0.05)
            self.play(Transform(point_plot2, point_plot2.move_to(
                plane_paths.c2p((i2 * path_length / 724), paths[0][i2])), run_time=0.2))

        self.play(ShowCreation(second_transition))
        self.play(FadeIn(second_nom))
        self.FadeInFadeOut(all_texts['zeroth_problem2'], wait_time=5)
        self.FadeInFadeOut(all_texts['second_problem'], wait_time=5)
        self.play(FadeIn(slide_dot))
        self.wait()

        self.play(FadeIn(slide_dot))
        self.remove(first_robot_faded, rob_ins)
        print("Animating trace path 4...\n")
        self.add(TracedPath(point_plot3.get_center, stroke_width=5, stroke_color=GREEN).fix_in_frame())
        self.add(TracedPath(point_plot4.get_center, stroke_width=5, stroke_color=GREEN).fix_in_frame())
        first_robot_faded = get_robot_instance(theta_list=np.array(get_solution(1, paths[1][1], 1)), offset=0.48,
                                               show_frame=True, hide_ee=True, opacity=0.25)
        rob_ins = get_robot_instance(theta_list=np.array(get_solution(1, paths[1][1], 1)), offset=0.48,
                                     show_frame=True, hide_ee=True)
        self.add(first_robot_faded)
        self.FadeInFadeOut(all_texts['third_choice'], wait_time=3)

        slide_error = 16
        for i2 in range(1, len(paths[1]) + slide_error, 8):
            print(f"value of i2 is {i2}")
            i2 = min(i2, len(paths[1]) - 1)
            if i2 > 724 - 74 and show_three:
                show_three = False
                # self.wait()
                self.play(ShowCreation(third_transition))
                # self.wait()
                self.play(FadeIn(third_nom))
                self.FadeInFadeOut(all_texts['zeroth_problem3'], wait_time=5)

            inter_set = np.array(get_solution(i2, paths[1][i2], 1))
            self.play(Transform(rob_ins, get_robot_instance(theta_list=inter_set, offset=0.48,
                                                            show_frame=True, hide_ee=True)), run_time=0.05)

            self.play(*[Transform(ii, jj) for ii, jj in zip([point_plot3, point_plot4],
                                                            [point_plot3.move_to(
                                                                plane_paths.c2p((i2 * path_length / 724),
                                                                                paths[1][i2])),
                                                                point_plot4.move_to(
                                                                    plane_paths.c2p((i2 * path_length / 724),
                                                                                    paths[3][i2]))])], run_time=0.05)
        self.wait()
        self.play(FadeIn(all_texts['third_problem']))
        self.play(FadeIn(slide_dot))
        self.wait()
        self.play(FadeIn(slide_dot))
        # self.wait(2)

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


class JacoNegTraj(ThreeDScene):

    def construct(self) -> None:
        # setting frame for slice presentation
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        previous_scale = 1.4
        previous_shift = LEFT * UP
        frame.scale(previous_scale)
        frame.shift(previous_shift)

        df = pd.read_csv("resources/data/jaco_slice_zoomed.csv")
        back_plane = NumberPlane(x_range=(-3.4, 0), y_range=(-1.85, 2.50)).set_width(3.4)
        back_plane.shift([-1.7, 0.325, 5.6 + 0.48])
        group_dot = VGroup()

        for ii in range(0, len(df), 5):
            if ii % 100 == 0: print(f"done adding Dot at index {ii}")
            group_dot.add(Dot(fill_color=dict_color_code[df['iks'][ii]]).scale(0.19).move_to(back_plane.coords_to_point(df['x'][ii] / 100, df['y'][ii] / 100)))
        ee_trace_2d = Group()
        for xx1, yy1, xx2, yy2 in zip([-2.8, -2.8, -1.2, -1.2], [0, 2, 2, 0], [-2.8, -1.2, -1.2, -2.8],
                                      [2, 2, 0, 0]):
            ee_trace_2d.add(Line3D(back_plane.c2p(xx1, yy1), back_plane.c2p(xx2, yy2)))
        ee_trace_2d.shift(UP * 0.05)

        plane_paths = get_path_axis().fix_in_frame()
        line_y_offset = Line(plane_paths.c2p(0, -3), plane_paths.c2p(0, -3.15)).fix_in_frame()
        path_length = 10
        df_n = pd.read_csv("resources/data/saved_data_theta1.csv")
        df_n2 = pd.read_csv("resources/data/saved_data_neg_theta1.csv")
        # full_path = []
        paths = make_paths(df_n, thresh=0.2)
        full_path2 = Group()
        first_x = (len(paths[2]) * path_length) / 724
        second_x = (len(paths[0]) * path_length) / 724
        third_x = ((len(paths[1]) - len(paths[4])) * path_length) / 724

        print(f"number of paths in positive det are {len(paths)}")
        for ii in range(len(paths)):
            print(f"Length of path {ii} is {len(paths[ii])}")
            for i4 in np.arange(1, len(paths[ii]) - 1):
                i3 = paths[ii][0] - len(paths[ii]) + 1 + i4
                full_path2.add(Line(plane_paths.c2p(i3 * path_length / 724, paths[ii][i4]),
                                    plane_paths.c2p((i3 + 1) * path_length / 724, paths[ii][i4 + 1]),
                                    stroke_color=BLUE_C, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

        paths2 = make_paths(df_n2, thresh=0.2)
        full_path22 = Group()
        print(f"number of paths in negative det are {len(paths2)}")
        for ii in range(len(paths2)):
            print(f"Length of path2 {ii} is {len(paths2[ii])}")
            for i4 in np.arange(1, len(paths2[ii]) - 1):
                i3 = paths2[ii][0] - len(paths2[ii]) + 1 + i4
                full_path22.add(Line(plane_paths.c2p(i3 * path_length / 724, paths2[ii][i4]),
                                     plane_paths.c2p((i3 + 1) * path_length / 724, paths2[ii][i4 + 1]),
                                     stroke_color=RED_C, stroke_width=5, stroke_opacity=0.6).fix_in_frame())

        # transition lines
        first_transition = DashedLine(plane_paths.c2p(first_x, -3.5),
                                      plane_paths.c2p(first_x, 3.5)).fix_in_frame()
        second_transition = DashedLine(plane_paths.c2p(second_x, -3.5),
                                       plane_paths.c2p(second_x, 3.5)).fix_in_frame()
        third_transition = DashedLine(plane_paths.c2p(third_x, -3.5),
                                      plane_paths.c2p(third_x, 3.5)).fix_in_frame()

        # transition text
        first_nom = TexText("$(8 \\rightarrow 6)$", font_size=24).move_to(first_transition.get_end()).shift(
            LEFT * 0.5).fix_in_frame()
        second_nom = TexText("$(6 \\rightarrow 4)$", font_size=24).move_to(second_transition.get_end()).shift(
            RIGHT * 0.5).fix_in_frame()
        third_nom = TexText("$(4 \\rightarrow 8)$", font_size=24).move_to(
            third_transition.get_end()).fix_in_frame()

        point_plot = Dot().move_to(plane_paths.c2p(0, paths2[1][1])).fix_in_frame()
        point_plot2 = Dot().move_to(plane_paths.c2p(0, paths2[3][1])).fix_in_frame()
        point_plot3 = Dot().move_to(plane_paths.c2p(0, paths2[4][1])).fix_in_frame()

        t1 = TexText(r"$T_1$", font_size=36).move_to(plane_paths.c2p(7, 3)).fix_in_frame()
        t2 = TexText(r"$T_2$", font_size=36).move_to(plane_paths.c2p(7, 2)).fix_in_frame()
        t3t4 = TexText(r"$T_3$, $T_4$", font_size=36).move_to(plane_paths.c2p(1, 0.5)).fix_in_frame()
        t5 = TexText(r"$T_5$", font_size=36).move_to(plane_paths.c2p(7, 0.5)).fix_in_frame()
        t6 = TexText(r"$T_6$", font_size=36).move_to(plane_paths.c2p(7, -1.1)).fix_in_frame()
        t7t8 = TexText(r"$T_7$, $T_8$", font_size=36).move_to(plane_paths.c2p(1, -2.5)).fix_in_frame()

        all_texts = get_text_neg()

        # Animations
        first_robot_faded = get_robot_instance(theta_list=np.array(get_solution(1, paths2[1][1], 0)), offset=0.48,
                                               show_frame=True, hide_ee=True, opacity=0.25)
        rob_ins = get_robot_instance(theta_list=np.array(get_solution(1, paths2[1][1], 0)), offset=0.48,
                                     show_frame=True, hide_ee=True)

        self.play(*[FadeIn(item) for item in [group_dot, ee_trace_2d]])
        self.wait()
        self.play(frame.animate.set_euler_angles(8.18e-01, 1.15, 0).scale(1.3 / previous_scale).shift(np.array([-6, -3, 4]) - previous_shift), run_time=2)
        self.play(*[FadeIn(item) for item in [plane_paths, line_y_offset, first_nom, second_nom, third_nom, first_transition, second_transition, third_transition]])
        self.wait()
        self.play(*[FadeIn(path_item) for path_item in [full_path2, full_path22]])
        self.add(t1, t2, t3t4, t5, t6, t7t8)
        self.wait()
        self.add(first_robot_faded, rob_ins)
        for item in full_path2:
            item.shift([0, 0, -0.05])
        self.FadeIt(*full_path2)
        self.wait()
        # self.embed()

        print("Animating NSCS")
        self.add(TracedPath(point_plot.get_center, stroke_width=5, stroke_color=GOLD_A, time_per_anchor=0.2).fix_in_frame())
        self.add(TracedPath(point_plot2.get_center, stroke_width=5, stroke_color=GOLD_A, time_per_anchor=0.2).fix_in_frame())
        self.add(TracedPath(point_plot3.get_center, stroke_width=5, stroke_color=GOLD_A, time_per_anchor=0.2).fix_in_frame())
        self.FadeInFadeOut(all_texts['nscs_choice'], wait_time=3)

        complete_path_T1 = paths2[3] + paths2[4]
        new_start = 1
        removed_point = False
        for i2 in np.arange(1, min(len(complete_path_T1), len(paths2[1])) + 8, 8):
            i2 = min(i2, min(len(complete_path_T1), len(paths2[1])) - 1)
            if i2 < len(paths2[3]):
                self.play(*[Transform(obj, obj.move_to(plane_paths.c2p(i2 * path_length / 724, y_coord))) for obj, y_coord in zip([point_plot, point_plot2], [paths2[1][i2], paths2[3][i2]])], run_time=0.05)
            else:
                if not removed_point:
                    self.remove(point_plot2)
                    removed_point = True
                self.play(*[Transform(obj, obj.move_to(plane_paths.c2p(i2 * path_length / 724, y_coord))) for obj, y_coord in zip([point_plot, point_plot3], [paths2[1][i2], paths2[4][new_start]])], run_time=0.05)
                new_start += 8
                new_start = min(new_start, len(paths2[4]) - 2)

            self.play(Transform(rob_ins, get_robot_instance(theta_list=np.array(get_solution(i2, paths2[1][i2], 0)), offset=0.48, show_frame=True, hide_ee=True)), run_time=0.05)

        self.wait()
        self.FadeInFadeOut(all_texts['nscs_con'], wait_time=3)
        self.FadeInFadeOut(all_texts['nscs_repeat'], wait_time=3)

        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


class SliceExplanation(Scene):

    def construct(self) -> None:
        # setting frame for lice presentation
        # plotting the slice
        df = pd.read_csv("resources/data/jaco_slice_zoomed.csv")
        axes_mul = 1.3
        back_plane = Axes(x_range=(-3.4, 0, 0.5), y_range=(-1.85, 2.50, 0.5), width=3.4 * axes_mul,
                          height=4.35 * axes_mul).shift(UP * 0.5)
        back_plane.x_axis.shift(DOWN * 1.85 * axes_mul)
        x_label2 = TexText("$x$ coord", font_size=30, color=YELLOW_D).next_to(back_plane.x_axis, DOWN * 1.8)
        y_label2 = TexText("$y$ coord", font_size=30, color=YELLOW_D).next_to(back_plane.y_axis).rotate(
            np.pi / 2).scale(0.8)
        back_plane.add(x_label2, y_label2)
        back_plane.add_coordinate_labels(
            font_size=20,
            num_decimal_places=1,
            buffers=[0.2, -0.4]
        )

        # back_plane.shift([-1.7, 0.325, 5.6 + 0.48])
        group_dot = VGroup()
        print(f"length of dataframe is {len(df)}")
        for ii in range(0, len(df), 5):
            if ii % 1000 == 0: print(f"done adding Dot at index {ii}")
            group_dot.add(
                Dot(fill_color=dict_color_code[df['iks'][ii]]).scale(0.2).move_to(back_plane.coords_to_point(
                    df['x'][ii] / 100, df['y'][ii] / 100)))

        ee_trace_2d = Group()
        for xx1, yy1, xx2, yy2 in zip([-2.8, -2.8, -1.2, -1.2], [0, 2, 2, 0], [-2.8, -1.2, -1.2, -2.8], [2, 2, 0, 0]):
            ee_trace_2d.add(Line3D(back_plane.c2p(xx1, yy1), back_plane.c2p(xx2, yy2), color=RED_D))
        # ee_trace_2d.shift(UP * 0.05)

        caption = Group()
        for iks, shifter in zip([8, 6, 4], [-1, 0.001, 1]):
            color = dict_color_code[iks]
            print(f"Color is {color}")
            circle_dot = Dot(radius=0.2, fill_color=color).to_edge(RIGHT, buff=3).shift(DOWN * shifter)
            text_dot = TexText(str(iks) + " IKS region", font_size=32).next_to(circle_dot, RIGHT * 0.7)
            caption_individual = Group(circle_dot, text_dot)
            caption.add(caption_individual)

        plane_paths = get_path_axis()
        line_y_offset = Line(plane_paths.c2p(0, -3), plane_paths.c2p(0, -3.15))

        # Animations
        # self.add(get_background())
        self.add(back_plane)
        self.play(FadeIn(group_dot))
        self.play(*[FadeIn(item) for item in caption], run_time=3)

        self.wait()
        self.play(ShowCreation(ee_trace_2d), run_time=5)
        self.ScaleBack(ee_trace_2d, scale_factor=1.5)
        self.play(FadeIn(slide_dot))

        self.wait()
        self.play(FadeOut(caption))
        # self.play(*[Transform(ii, ii.shift(RIGHT * 4.3)) for ii in [back_plane, group_dot, ee_trace_2d]], run_time=2)
        self.play(*[ii.animate.shift(RIGHT * 4.3) for ii in [back_plane, group_dot, ee_trace_2d]])
        self.play(*[FadeOut(ii) for ii in [back_plane.coordinate_labels, back_plane.axes, x_label2, y_label2]],
                  run_time=2)
        self.play(*[FadeIn(ii) for ii in [plane_paths, line_y_offset]])
        self.play(FadeIn(slide_dot))

        self.wait()
        for instance in [1, 40, 182, 363, 544, 634, 724]:
            thetas_pos, thetas_neg = get_theta_instance(instance)
            list_inter = get_value_workspace(instance)
            dot_play_workspace = Dot().move_to(back_plane.c2p(list_inter[0], list_inter[1]))
            self.play(FadeIn(dot_play_workspace))
            self.ScaleBack(dot_play_workspace, 2)
            for ii in range(len(thetas_pos)):
                dot_play_pos = Dot(fill_color=BLUE_D).scale(1.01).move_to(
                    plane_paths.c2p(instance * 10 / 724, thetas_pos[ii]))
                dot_play_neg = Dot(fill_color=RED_D).scale(0.8).move_to(
                    plane_paths.c2p(instance * 10 / 724, thetas_neg[ii]))
                self.play(*[FadeIn(ii) for ii in [dot_play_pos, dot_play_neg]], run_time=0.1)

            dot_play_pos = Dot(fill_color=BLUE_D).scale(1.01).move_to(
                plane_paths.c2p(instance * 10 / 724, thetas_pos[ii]))
            dot_play_neg = Dot(fill_color=RED_D).scale(0.8).move_to(
                plane_paths.c2p(instance * 10 / 724, thetas_neg[ii]))
            self.play(*[FadeIn(ii) for ii in [dot_play_pos, dot_play_neg]], run_time=0.1)
        self.play(FadeIn(slide_dot))

        self.wait()
        self.play(FadeOut(slide_dot))
        # self.embed()

    def ScaleBack(self, in_obj, scale_factor):
        self.play(ScaleInPlace(in_obj, scale_factor=scale_factor))
        self.wait(0.2)
        self.play(ScaleInPlace(in_obj, scale_factor=1 / scale_factor))


class TorusTransform(ThreeDScene):

    def construct(self):
        frame = self.camera.frame
        theta, phi, gamma = frame.get_theta(), frame.get_phi(), frame.get_gamma()
        frame.set_euler_angles(theta=-0.55, phi=0.7, gamma=0)
        frame.move_to(ORIGIN)
        axes = ThreeDAxes()
        axes.x_axis.set_color(color=RED_D)
        axes.y_axis.set_color(color=GREEN_D)
        # self.add(axes)

        point_a = np.array([-PI, 1, -PI])
        point_b = np.array([PI, 1, -PI])
        point_c = np.array([PI, 1, PI])
        point_d = np.array([-PI, 1, PI])

        line_left = Line3D(point_a, point_d, color=YELLOW_D)
        line_right = Line3D(point_b, point_c, color=YELLOW_D)
        line_bottom = Line3D(point_a, point_b, color=RED_D)
        line_top = Line3D(point_c, point_d, color=RED_D)

        big_radius = 1
        small_radius = 1
        border_buff = 0.1

        # formation of torus        
        torus = ParametricSurface(lambda u, v: self.torus_func(u, v, R=big_radius, r=small_radius), u_range=(0, TAU),
                                  v_range=(0, 0.01)).set_color(color=BLUE_D)
        torus_right = ParametricSurface(lambda u, v: self.torus_func(u, v, R=big_radius, r=small_radius),
                                        u_range=(0, TAU), v_range=(TAU - border_buff, TAU)).set_color(color=YELLOW_D)
        torus_left = ParametricSurface(lambda u, v: self.torus_func(u, v, R=big_radius, r=small_radius),
                                       u_range=(0, TAU), v_range=(0, border_buff)).set_color(color=YELLOW_D)
        torus.set_reflectiveness(0.5)

        # Animations
        # self.add(get_background().fix_in_frame())

        for ii in np.arange(0, TAU + 0.4, 0.4):
            new_torus = ParametricSurface(lambda u, v: self.torus_func(u, v, R=big_radius, r=small_radius),
                                          u_range=(0, TAU), v_range=(0, ii)).set_color(color=BLUE_D)
            self.play(Transform(torus, new_torus), run_time=0.1)
        self.wait(2)
        self.play(frame.animate.set_euler_angles(theta=theta, phi=phi, gamma=gamma))
        # self.embed()

        # cutting the torus to cylinder
        total_iterations = 50
        for iteration in range(total_iterations):
            ii = PI - iteration * (179 * DEGREES) / total_iterations
            new_radius = PI * big_radius / ii
            opacity = 1 - 0.5 * iteration / total_iterations
            new_torus = ParametricSurface(lambda u, v: self.torus_func(u, v, R=new_radius, r=small_radius),
                                          u_range=(0, TAU), v_range=(PI - ii, PI + ii)).set_color(color=BLUE_D,
                                                                                                  opacity=opacity)
            new_torus_left = ParametricSurface(lambda u, v: self.torus_func(u, v, R=new_radius, r=small_radius),
                                               u_range=(0, TAU), v_range=(
                    PI - ii - border_buff * big_radius / (new_radius), PI - ii)).set_color(color=YELLOW_D,
                                                                                           opacity=opacity)
            new_torus_right = ParametricSurface(lambda u, v: self.torus_func(u, v, R=new_radius, r=small_radius),
                                                u_range=(0, TAU), v_range=(
                    PI + ii, PI + ii + border_buff * big_radius / (new_radius))).set_color(color=YELLOW_D,
                                                                                           opacity=opacity)
            self.play(*[Transform(obj, obj2) for obj, obj2 in
                        zip([torus, torus_right, torus_left], [new_torus, new_torus_right, new_torus_left])],
                      run_time=0.1)

        cylinder = ParametricSurface(lambda u, v: np.array([u, small_radius * (cos(v) + 1), small_radius * sin(v)]),
                                     u_range=(-PI * big_radius, PI * big_radius), v_range=(0, TAU)).set_color(
            color=BLUE_D, opacity=0.5)
        cylinder_top = ParametricSurface(lambda u, v: np.array([u, small_radius * (cos(v) + 1), small_radius * sin(v)]),
                                         u_range=(-PI * big_radius, PI * big_radius),
                                         v_range=(0, border_buff)).set_color(color=BLUE_D, opacity=0.5)
        cylinder_bottom = ParametricSurface(
            lambda u, v: np.array([u, small_radius * (cos(v) + 1), small_radius * sin(v)]),
            u_range=(-PI * big_radius, PI * big_radius), v_range=(0, -border_buff)).set_color(color=BLUE_D, opacity=0.5)
        cylinder_left_cover = ParametricSurface(
            lambda u, v: np.array([u, small_radius * (cos(v) + 1), small_radius * sin(v)]),
            u_range=(-PI * big_radius - border_buff, -PI * big_radius), v_range=(0, TAU)).set_color(color=YELLOW_D,
                                                                                                    opacity=0.5)
        cylinder_right_cover = ParametricSurface(
            lambda u, v: np.array([u, small_radius * (cos(v) + 1), small_radius * sin(v)]),
            u_range=(PI * big_radius, PI * big_radius + border_buff), v_range=(0, TAU)).set_color(color=YELLOW_D,
                                                                                                  opacity=0.5)

        # self.play(FadeOut(torus), FadeIn(cylinder), run_time=0.05)
        self.play(*[FadeIn(obj) for obj in [cylinder, cylinder_left_cover, cylinder_right_cover]], run_time=0.05)
        self.remove(torus, torus_left, torus_right)
        # self.play(FadeOut(torus_left), FadeIn(cylinder_left_cover), run_time=0.05)
        # self.play(FadeOut(torus_right), FadeIn(cylinder_right_cover), run_time=0.05)
        self.wait(2)

        # cutting the cylinder to a plane
        added_top = False
        for iteration in range(total_iterations):
            ii = PI - iteration * (90 * DEGREES) / total_iterations
            new_radius = PI * small_radius / ii
            z_nudge = (PI - 1 - small_radius) * iteration / total_iterations
            # y_pull = new_radius * cos(v) * iteration / total_iterations
            new_cylinder = ParametricSurface(lambda u, v: np.array(
                [u, new_radius * cos(v) * (iteration - total_iterations) / total_iterations + small_radius,
                 (new_radius + z_nudge) * sin(v)]), u_range=(-PI * big_radius, PI * big_radius),
                                             v_range=(PI - ii, PI + ii)).set_color(color=BLUE_D, opacity=0.5)
            cylinder_top_cover = ParametricSurface(lambda u, v: np.array(
                [u, new_radius * cos(v) * (iteration - total_iterations) / total_iterations + small_radius,
                 (new_radius + z_nudge) * sin(v)]), u_range=(
                -PI * big_radius - border_buff, PI * big_radius + border_buff), v_range=(
                PI - ii - border_buff * small_radius / new_radius, PI - ii)).set_color(color=RED_D, opacity=0.5)
            cylinder_bottom_cover = ParametricSurface(lambda u, v: np.array(
                [u, new_radius * cos(v) * (iteration - total_iterations) / total_iterations + small_radius,
                 (new_radius + z_nudge) * sin(v)]), u_range=(
                -PI * big_radius - border_buff, PI * big_radius + border_buff), v_range=(
                PI + ii, PI + ii + border_buff * small_radius / new_radius)).set_color(color=RED_D, opacity=0.5)
            cylinder_left_cover2 = ParametricSurface(lambda u, v: np.array(
                [u, new_radius * cos(v) * (iteration - total_iterations) / total_iterations + small_radius,
                 (new_radius + z_nudge) * sin(v)]), u_range=(-PI * big_radius - border_buff, -PI * big_radius),
                                                     v_range=(PI - ii, PI + ii)).set_color(color=YELLOW_D, opacity=0.5)
            cylinder_right_cover2 = ParametricSurface(lambda u, v: np.array(
                [u, new_radius * cos(v) * (iteration - total_iterations) / total_iterations + small_radius,
                 (new_radius + z_nudge) * sin(v)]), u_range=(PI * big_radius, PI * big_radius + border_buff),
                                                      v_range=(PI - ii, PI + ii)).set_color(color=YELLOW_D, opacity=0.5)

            if iteration >= total_iterations - 1:
                z_gap = PI - (new_radius + z_nudge) * abs(sin(PI + ii))
                self.remove(line_top, line_bottom)
                line_bottom = Line3D(point_a, point_b, color=RED_D)
                line_top = Line3D(point_c, point_d, color=RED_D)
                self.add(line_top.shift(np.array([0, -0.1, -z_gap])), line_bottom.shift(np.array([0, -0.1, z_gap])))
                # self.embed()
                # added_top = True

            self.play(*[Transform(obj, obj2) for obj, obj2 in
                        zip([cylinder, cylinder_right_cover, cylinder_left_cover, cylinder_top, cylinder_bottom],
                            [new_cylinder, cylinder_right_cover2, cylinder_left_cover2, cylinder_top_cover,
                             cylinder_bottom_cover])], run_time=0.1)

        plane = ParametricSurface(lambda u, v: np.array([u, small_radius, v]),
                                  u_range=(-PI * big_radius, PI * big_radius),
                                  v_range=(-PI * big_radius, PI * big_radius)).set_color(color=BLUE_D, opacity=0.5)
        self.play(FadeOut(cylinder), FadeIn(plane), run_time=0.1)
        self.remove(line_top, line_bottom)
        line_bottom = Line3D(point_a, point_b, color=RED_D)
        line_top = Line3D(point_c, point_d, color=RED_D)
        self.add(line_top, line_bottom, line_left, line_right)
        self.remove(cylinder_left_cover, cylinder_right_cover, cylinder_top, cylinder_bottom)
        self.wait(2)

        # Animating camera frame
        self.play(frame.animate.set_euler_angles(theta=0, phi=PI / 2, gamma=0).shift([0, 1, 0]))
        self.wait(2)
        self.embed()

    def torus_func(self, u, v, R=3, r=1):

        x_val = -(R + r * cos(u)) * sin(v)
        y_val = (R + r * cos(u)) * cos(v) + R + r
        z_val = r * sin(u)

        return np.array([x_val, y_val, z_val])


class FlatPlots(Scene):

    def construct(self) -> None:

        xconfig = dict(stroke_width=0.001, opacity=0, include_ticks=False, stroke_color=BLACK)
        yconfig = dict(stroke_width=0.001, opacity=0, include_ticks=False, stroke_color=BLACK)
        plot_to_return = Axes(
            x_range=(-3, 3),
            y_range=(-3, 3),
            y_axis_config=yconfig,
            x_axis_config=xconfig,
            width=6.28,
            height=6.28
        )

        square = Square(side_length=6.28, fill_color=BLUE_D, fill_opacity=0.5, stroke_opacity=0)
        line_top = Line(square.get_corner(UL), square.get_corner(UR), stroke_color=RED_D)
        line_bottom = Line(square.get_corner(DL), square.get_corner(DR), stroke_color=RED_D)
        line_left = Line(square.get_corner(DL), square.get_corner(UL), stroke_color=YELLOW_D)
        line_right = Line(square.get_corner(DR), square.get_corner(UR), stroke_color=YELLOW_D)

        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=WHITE, x_range=(-3.14, 3.14),
                                           y_range=(-3.14, 3.14))
        x_label = TexText(r"$\theta_2$").move_to(line_bottom.get_center()).shift(DOWN * 0.3)
        y_label = TexText(r"$\theta_3$").move_to(line_left.get_center()).shift(LEFT * 0.3)

        aspect_def = TexText(
            r"""\begin{minipage}{5cm} \centering An aspect is a singularity free region in the joint space of the robot \end{minipage}""",
            font_size=36).add_background_rectangle(color=BLACK, opacity=0.8)
        pos_det = TexText(r"""\begin{minipage}{5cm} \centering Aspect with $\det(\mathbf{J}) > 0$ \end{minipage}""",
                          font_size=36).add_background_rectangle(color=BLACK, opacity=0.8)
        neg_det = TexText(r"""\begin{minipage}{5cm} \centering Aspect with $\det(\mathbf{J}) < 0$ \end{minipage}""",
                          font_size=36).add_background_rectangle(color=BLACK, opacity=0.8)
        singularities = TexText(
            r"""\begin{minipage}{5cm} \centering Singularities in the joint space are the locus of critical points of the forward kinematic map $\det(\mathbf{J}) = 0$ \end{minipage}""",
            font_size=36).add_background_rectangle(color=BLACK, opacity=0.8)

        neg_dots = Group()
        pos_dots = Group()
        for t2 in np.arange(-3.1, 3.14, 0.1):
            for t3 in np.arange(-3.1, 3.14, 0.1):
                det_val = -4.5 * sin(t3) * cos(t2) * cos(t3) - 6.0 * sin(t3) * cos(t2) - 2.25 * sin(t3) * cos(
                    t3) - 3.0 * sin(t3) + 2.25 * cos(t2) * cos(t3) ** 2 + 3.0 * cos(t2) * cos(t3)
                radius = 0.04
                if det_val < 0:
                    neg_dots.add(Dot(fill_color=RED_D, radius=radius).move_to(np.array([t2, t3, 0])))
                else:
                    pos_dots.add(Dot(fill_color=GREEN_D, radius=radius).move_to(np.array([t2, t3, 0])))

        # self.add(get_background())
        self.add(square, line_left, line_bottom, line_top, line_right)
        self.wait()
        self.play(FadeIn(x_label), FadeIn(y_label))
        self.wait(2)
        self.FadeInFadeOut(singularities)
        self.play(ShowCreation(critical_points), run_time=2)
        self.wait()
        self.FadeInFadeOut(aspect_def)
        self.FadeInFadeOut(pos_det, wait_time=2)
        self.play(FadeIn(pos_dots))
        self.add(critical_points)
        self.wait()
        self.FadeInFadeOut(neg_det, wait_time=2)
        self.play(FadeIn(neg_dots))
        self.add(critical_points)
        self.wait()

        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[ReplacementTransform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


class ClassCube(ThreeDScene):

    def construct(self) -> None:
        frame = self.camera.frame
        theta, phi, gamma = frame.get_theta(), frame.get_phi(), frame.get_gamma()
        frame.set_euler_angles(theta=-0.52, phi=1.22, gamma=0)
        frame.move_to([0.92, 0.5, 2.46])
        axes = ThreeDAxes()
        axes.x_axis.set_color(color=RED_D)
        axes.y_axis.set_color(color=GREEN_D)
        # self.add(axes)

        cube_length = 4
        color = BLUE_D
        opac = 0.25
        range_subs = (0, cube_length)

        plane1 = ParametricSurface(lambda u, v: np.array([u, v, 0]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)
        plane2 = ParametricSurface(lambda u, v: np.array([u, v, cube_length]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)
        plane3 = ParametricSurface(lambda u, v: np.array([u, 0, v]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)
        plane4 = ParametricSurface(lambda u, v: np.array([u, cube_length, v]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)
        plane5 = ParametricSurface(lambda u, v: np.array([0, u, v]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)
        plane6 = ParametricSurface(lambda u, v: np.array([cube_length, u, v]), u_range=range_subs, v_range=range_subs,
                                   color=color, opacity=opac)

        # labels
        fs = 32
        ls = 54
        origin = TexText(r"$(0, 0, 0)$", font_size=fs).shift([0, 0, -0.25]).fix_in_frame()
        extreme_x = TexText(r"$(100, 0, 0)$", font_size=fs).move_to([cube_length, 0, -0.25]).fix_in_frame()
        extreme_y = TexText(r"$(0, \dfrac{\pi}{2}, 0)$", font_size=fs).move_to([0, cube_length, -0.25]).fix_in_frame()
        extreme_z = TexText(r"$(0, 0, \dfrac{\pi}{2})$", font_size=fs).move_to([-0.25, 0, cube_length]).fix_in_frame()
        label_d5 = TexText(r"$d_5$", font_size=ls, color=YELLOW_D).move_to([cube_length / 2, 0, -0.35]).fix_in_frame()
        label_alpha3 = TexText(r"$\alpha_3$", font_size=ls, color=YELLOW_D).move_to([-0.35, cube_length, cube_length / 2]).fix_in_frame()
        label_alpha4 = TexText(r"$\alpha_4$", font_size=ls, color=YELLOW_D).move_to([-0.35, cube_length / 2, 0]).fix_in_frame()

        # Generating spheres
        sph_length = 5
        shift_ratio = cube_length / sph_length
        sphere_group = Group()
        border_spheres = Group()
        for ii in range(sph_length + 1):
            for jj in range(sph_length + 1):
                for kk in range(sph_length + 1):
                    move_to_arr = np.array([ii, jj, kk])
                    move_to_arr = move_to_arr * shift_ratio
                    if ii * jj * kk == 0 or max(ii, jj, kk) == sph_length:
                        border_spheres.add(Sphere(radius=0.1, color=RED_D).move_to(move_to_arr))
                    else:
                        sphere_group.add(Sphere(radius=0.1, color=YELLOW_D).move_to(move_to_arr))

        self.wait(2)
        self.play(*[FadeIn(obj) for obj in [plane1]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plane2, plane3, plane4, plane5, plane6]])
        self.wait()
        self.add(origin, extreme_x, extreme_y, extreme_z)
        self.add(label_d5, label_alpha3, label_alpha4)
        for item in [origin, extreme_x, extreme_y, extreme_z, label_d5, label_alpha3, label_alpha4]:
            item.unfix_from_frame()
            item.rotate(angle=PI / 2, axis=RIGHT, about_point=item.get_center())
        self.wait(3)
        self.remove(plane3, plane5)
        self.play(FadeIn(border_spheres))
        # self.next_slide()
        self.wait()
        self.play(*[FadeIn(obj) for obj in [sphere_group, plane3, plane5]])
        self.wait()
        # self.next_slide()

        self.play(FadeOut(border_spheres))
        self.wait()
        list_animation = [plane2.animate.rotate(angle=PI / 2, axis=UP, about_point=[cube_length, 0, 0]),
                          plane3.animate.rotate(angle=PI / 2, axis=RIGHT, about_point=[0, 0, 0]),
                          plane4.animate.rotate(angle=-PI / 2, axis=RIGHT, about_point=[0, cube_length, 0]),
                          plane5.animate.rotate(angle=PI / 2, axis=DOWN, about_point=[0, 0, 0]),
                          plane6.animate.rotate(angle=-PI / 2, axis=DOWN, about_point=[cube_length, 0, 0])]

        self.play(*list_animation, run_time=4)
        self.play(plane2.animate.rotate(angle=PI / 2, axis=UP, about_point=[cube_length * 2, 0, 0]), run_time=1)
        self.play(*[FadeOut(obj) for obj in [origin, extreme_x, extreme_y, extreme_z]])
        self.wait(0.5)
        # self.play(*[Rotate(obj, axis=LEFT, about_point=obj.get_center(), angle=PI / 2) for obj in [label_d5, label_alpha3, label_alpha4]])
        self.play(*[FadeOut(obj) for obj in [label_d5, label_alpha3, label_alpha4]])
        self.wait(0.5)

        self.play(frame.animate.scale(1.6).shift([2, 0, -3]))
        for plane_iter in [plane1, plane4, plane5]:
            plane_iter.set_opacity(0.25)
        self.wait(2)
        self.play(FadeOut(sphere_group))
        self.wait()
        prev_zoom = frame.get_scale()
        self.play(frame.animate.set_euler_angles(theta=0, phi=0, gamma=0).scale(1.75 / prev_zoom).move_to([7, 2, -1]))
        self.wait()

        self.embed()


class ClassCubePlanar(Scene):

    def construct(self) -> None:

        # squares
        square_one = self.my_square().to_edge(LEFT)
        square_two = square_one.copy().next_to(square_one, RIGHT, buff=0.01)
        square_three = square_one.copy().next_to(square_two, RIGHT, buff=0.01)
        square_four = square_one.copy().next_to(square_three, RIGHT, buff=0.01)
        square_five = square_two.copy().next_to(square_two, UP, buff=0.01)
        square_six = square_two.copy().next_to(square_two, DOWN, buff=0.01)

        # edges
        wrist_robots_edge_left = Line(square_one.get_corner(UL), square_one.get_corner(DL), stroke_color=YELLOW_D)
        wrist_robots_edge_right = Line(square_four.get_corner(UR), square_four.get_corner(DR), stroke_color=YELLOW_D)
        ur5_edge = Line(square_two.get_corner(UL), square_two.get_corner(UR), stroke_color=GREEN_D)
        offset_edge_up = Line(square_five.get_corner(UL), square_five.get_corner(UR), stroke_color=PINK)
        offset_edge_down = Line(square_four.get_corner(UL), square_four.get_corner(UR), stroke_color=PINK)

        # labels
        cube_length = square_one.get_width()
        ls = 42
        label_d5 = TexText(r"$d_5$", font_size=ls, color=YELLOW_D).move_to(np.array([0, -cube_length / 2, 0]) + square_two.get_center())
        label_alpha3 = TexText(r"$\alpha_3$", font_size=ls, color=YELLOW_D).move_to(np.array([cube_length / 2, cube_length, 0]) + square_two.get_center())
        label_alpha4 = TexText(r"$\alpha_4$", font_size=ls, color=YELLOW_D).move_to(np.array([-cube_length / 2, 0, 0]) + square_two.get_center())

        # coordinate labels
        buff = cube_length / 10
        fs = 30
        label_zero = TexText(r"$0$", font_size=fs).move_to(square_two.get_corner(DL) + np.array([-buff, -buff, 0]))
        label_hundred = TexText(r"$100$", font_size=fs).move_to(square_two.get_corner(DR) + np.array([buff, -buff, 0]))
        label_pi2 = TexText(r"$\dfrac{\pi}{2}$", font_size=fs).move_to(square_two.get_corner(UL) + np.array([-buff, 0, 0]))
        label_zero_alpha = TexText(r"$0$", font_size=fs).move_to(square_two.get_corner(UR) + np.array([buff, buff, 0]))
        label_pi22 = TexText(r"$\dfrac{\pi}{2}$", font_size=fs).move_to(square_five.get_corner(UR) + np.array([buff, 0, 0]))

        # stars
        jaco_buff = 55 * cube_length/ 90
        dot_orth_wrist = Dot().move_to(square_one.get_corner(UL))
        dot_jaco = Dot().move_to(square_four.get_corner(UL) + np.array([jaco_buff, -jaco_buff, 0]))
        # Dashed line to jaco point
        edge_vector = square_four.get_edge_center(LEFT)
        edge_vector2 = square_four.get_edge_center(DOWN)
        jaco_vector = dot_jaco.get_center()
        dashed_line1 = DashedLine(np.array([edge_vector[0], jaco_vector[1], 0]), dot_jaco.get_center())
        dashed_line2 = DashedLine(dot_jaco.get_center(), np.array([jaco_vector[0], edge_vector2[1], 0]))
        jaco_dashed_lines = Group(dashed_line1, dashed_line2)

        # comments
        degenerate_robots = TexText(r"degenerate robots \\ $\det(\mathbf{J}) = 0$", font_size=fs).move_to(square_six.get_center())
        wrist_partitioned_orth = TexText(r"""\begin{minipage}{2.5 cm}\centering wrist-partitioned robots with orthogonal wrist \end{minipage}""", font_size=fs).move_to(square_one.get_center() + UP * cube_length)
        wrist_partitioned_nonorth = TexText(r"""\begin{minipage}{2.5 cm}\centering wrist-partitioned robots \end{minipage}""", font_size=fs).move_to(square_four.get_corner(DR) + (DOWN * 0.2) * cube_length)
        wrist_partitioned_offset = TexText(r"""\begin{minipage}{2.5 cm}\centering robots with an offset in the wrist \end{minipage}""", font_size=fs).move_to(square_five.get_center() + RIGHT * 1.2 * cube_length)
        ur5_structure = TexText(r"""\begin{minipage}{2.5 cm}\centering UR5 like robots \end{minipage}""", font_size=fs).move_to(square_two.get_edge_center(UP) + UP * 0.2)
        jaco_structure = TexText(r"""\begin{minipage}{2 cm}\centering Jaco Gen2 like robots \end{minipage}""", font_size=fs).move_to(dot_jaco.get_center() + LEFT)

        # image objects
        image_path = "resources/raster_images/"
        image_kr5 = ImageMobject(image_path + "kuka_kr5.png").to_edge(RIGHT)
        image_jaco = ImageMobject(image_path + "jaco.png").to_edge(RIGHT)
        image_crx = ImageMobject(image_path + "crx.png").to_edge(RIGHT, buff=LARGE_BUFF)
        image_ur5 = ImageMobject(image_path + "ur5.png").to_edge(RIGHT)

        # transformed images
        image_kr52 = image_kr5.copy().move_to(square_one.get_center()).scale(cube_length / image_kr5.get_width())
        image_ur52 = image_ur5.copy().move_to(square_five.get_center()).scale(cube_length / image_ur5.get_height() / 1.5)
        image_crx2 = image_crx.copy().move_to(wrist_partitioned_offset.get_center() + RIGHT * 1.6).scale(cube_length / image_crx.get_height())
        image_jaco2 = image_jaco.copy().move_to(jaco_structure.get_center() + DOWN * 2).scale(cube_length / image_jaco.get_height())

        self.add(square_one, square_two, square_three, square_four, square_five, square_six)
        self.add(label_d5, label_alpha3, label_alpha4)
        self.add(label_zero, label_zero_alpha, label_hundred, label_pi2, label_pi22)

        self.wait(3)

        self.play(FadeIn(square_six.set_fill(color=RED_D)))
        self.wait()
        self.play(FadeIn(degenerate_robots))
        self.wait()

        self.play(FadeIn(dot_orth_wrist))
        self.wait()
        self.play(FadeIn(wrist_partitioned_orth), FadeIn(image_kr5))
        self.wait()
        self.play(ReplacementTransform(image_kr5, image_kr52))
        self.wait()

        self.play(FadeIn(dot_jaco))
        self.play(ShowCreation(jaco_dashed_lines))
        self.wait()
        self.play(FadeIn(jaco_structure), FadeIn(image_jaco))
        self.wait()
        self.play(ReplacementTransform(image_jaco, image_jaco2))
        self.wait()

        self.play(FadeIn(wrist_robots_edge_right), FadeIn(wrist_robots_edge_left))
        self.wait()
        self.play(FadeIn(wrist_partitioned_nonorth))


        self.play(FadeIn(offset_edge_up), FadeIn(offset_edge_down))
        self.wait()
        self.play(FadeIn(wrist_partitioned_offset), FadeIn(image_crx))
        self.wait()
        self.play(ReplacementTransform(image_crx, image_crx2))
        self.wait()

        self.play(FadeIn(ur5_edge))
        self.wait()
        self.play(FadeIn(ur5_structure), FadeIn(image_ur5))
        self.wait()
        self.play(ReplacementTransform(image_ur5, image_ur52))
        self.wait()

        # self.add(wrist_partitioned_orth, wrist_partitioned_nonorth, wrist_partitioned_offset, jaco_structure, ur5_structure)
        self.embed()

    def my_square(self):
        return Square(side_length=2.3, fill_opacity=0.25, fill_color=BLUE_D, stroke_opacity=0)
