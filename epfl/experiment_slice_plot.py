from functions.phd_functions.robot_functions import *
from functions.phd_functions.functions_epfl import *
from manimlib import *
from manim_slides.slide import Slide, ThreeDSlide
import pandas as pd

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


class JacoSlice1(ThreeDSlide):
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

        fixed_background = get_background().fix_in_frame()
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

        # all text objects
        all_texts = get_text()
        show_three = True

        # Animations
        self.add(fixed_background)
        self.play(*[FadeIn(item) for item in [group_dot, ee_trace_2d]])
        self.next_slide()
        # # self.wait()
        self.play(FadeIn(robot_top_view))
        self.wait()
        print("Animating frame orientation, scale and position...\n")
        self.play(frame.animate.set_euler_angles(8.18e-01, 1.15, 0).scale(1.3 / previous_scale).shift(np.array([-6, -3, 4]) - previous_shift), run_time=2)
        self.add(ee_trace_2d.shift(DOWN * 0.05))
        self.play(FadeIn(slide_dot))
        self.next_slide()

        self.play(*[FadeIn(item) for item in [plane_paths, line_y_offset]])
        self.play(FadeIn(slide_dot))
        self.next_slide()
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
        self.next_slide()

        # animations for the plots
        print("Animating path fade in...\n")
        self.play(*[FadeIn(path_item) for path_item in [full_path2, full_path22]])
        self.next_slide()
        self.FadeInFadeOut(all_texts['four_choices'])
        self.FadeIt(*full_path22)
        self.FadeInFadeOut(point_plotb, point_plot2b, point_plot3b, point_plot4b)
        self.FadeInFadeOut(all_texts['first_choice'], wait_time=3)
        self.next_slide()

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
        self.next_slide()

        self.play(ShowCreation(first_transition))
        self.play(FadeIn(first_nom))
        self.FadeInFadeOut(all_texts['zeroth_problem'], wait_time=5)
        self.FadeInFadeOut(all_texts['first_problem'], wait_time=5)
        self.next_slide()

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
        self.next_slide()

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
        self.next_slide()

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
            if i2 >= len(paths[1]) and not JacoSlice1.make_slides:
                break
            else:
                i2 = min(i2, len(paths[1]))
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
        self.next_slide()
        self.play(FadeIn(all_texts['third_problem']))
        self.play(FadeIn(slide_dot))
        self.next_slide()
        self.play(FadeIn(slide_dot))
        # self.wait(2)

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


class SliceExplanation(Slide):

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
        for ii in range(0, len(df), 10):
            if ii % 100 == 0: print(f"done adding Dot at index {ii}")
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
            circle_dot = Circle(radius=0.2, stroke_color=color).to_edge(RIGHT, buff=3).shift(DOWN * shifter)
            circle_dot.set_fill(color=color, opacity=1)
            text_dot = TexText(str(iks) + " IKS region", font_size=32).next_to(circle_dot, RIGHT * 0.7)
            caption_individual = Group(circle_dot, text_dot)
            caption.add(caption_individual)

        plane_paths = get_path_axis()
        line_y_offset = Line(plane_paths.c2p(0, -3), plane_paths.c2p(0, -3.15))

        # Animations
        self.add(get_background())
        self.add(back_plane)
        self.play(FadeIn(group_dot))
        self.play(*[FadeIn(item) for item in caption], run_time=3)

        self.next_slide()
        self.play(ShowCreation(ee_trace_2d), run_time=5)
        self.ScaleBack(ee_trace_2d, scale_factor=1.5)
        self.play(FadeIn(slide_dot))

        self.next_slide()
        self.play(FadeOut(caption))
        # self.play(*[Transform(ii, ii.shift(RIGHT * 4.3)) for ii in [back_plane, group_dot, ee_trace_2d]], run_time=2)
        self.play(*[ii.animate.shift(RIGHT * 4.3) for ii in [back_plane, group_dot, ee_trace_2d]])
        self.play(*[FadeOut(ii) for ii in [back_plane.coordinate_labels, back_plane.axes, x_label2, y_label2]],
                  run_time=2)
        self.play(*[FadeIn(ii) for ii in [plane_paths, line_y_offset]])
        self.play(FadeIn(slide_dot))

        self.next_slide()
        for instance in [1, 40, 182, 363, 544, 634, 724, 724]:
            thetas_pos, thetas_neg = get_theta_instance(instance)
            list_inter = get_value_workspace(instance)
            dot_play_workspace = Dot().move_to(back_plane.c2p(list_inter[0], list_inter[1]))
            self.play(FadeIn(dot_play_workspace))
            self.ScaleBack(dot_play_workspace, 2)
            for ii in range(len(thetas_pos)):
                dot_play_pos = Dot(fill_color=BLUE_D).scale(1.01).move_to(
                    plane_paths.c2p(instance * 10 / 724, thetas_pos[ii]))
                dot_play_neg = Dot(fill_color=RED_D).scale(0.8).move_to(plane_paths.c2p(instance * 10 / 724, thetas_neg[ii]))
                self.play(*[FadeIn(ii) for ii in [dot_play_pos, dot_play_neg]], run_time=0.1)

            dot_play_pos = Dot(fill_color=BLUE_D).scale(1.01).move_to(
                plane_paths.c2p(instance * 10 / 724, thetas_pos[ii]))
            dot_play_neg = Dot(fill_color=RED_D).scale(0.8).move_to(
                plane_paths.c2p(instance * 10 / 724, thetas_neg[ii]))
            self.play(*[FadeIn(ii) for ii in [dot_play_pos, dot_play_neg]], run_time=0.1)
        self.play(FadeIn(slide_dot))

        self.next_slide()
        self.play(FadeOut(slide_dot))
        # self.embed()

    def ScaleBack(self, in_obj, scale_factor):
        self.play(ScaleInPlace(in_obj, scale_factor=scale_factor))
        self.wait(0.2)
        self.play(ScaleInPlace(in_obj, scale_factor=1 / scale_factor))
