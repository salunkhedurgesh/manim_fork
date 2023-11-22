import numpy as np
from functions.phd_functions.robot_functions import *
from functions.phd_functions.functions_epfl import *
from functions.phd_functions.robots_3r import *
from manimlib import *
from numpy import sin, cos, sqrt
import pandas as pd


def DashedLine3D(start, end, instances=20, buff=0.2):
    all_lines = Group()
    start = np.array(start)
    end = np.array(end)

    for ii in range(instances):
        start_point = start + (end - start) * ii / instances
        end_point = start + (end - start) * (ii + (1 - buff)) / instances
        all_lines.add(Line3D(start_point, end_point))

    return all_lines


class NoTheta1(ThreeDScene):

    def construct(self) -> None:
        frame = self.camera.frame
        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
               [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
               [1.72565092649699, -0.352336624163169, -2.014417810173952],
               [-1.44839814046463, -1.5, -1.57]]

        iks_plot = [-1.44839814046463, -1.5, 1.57]

        rob_ins = get_robot_instance(theta_list=iks_plot, robot_type="orthogonal3r", offset=-1.5, show_frame=False, add_plane=False)
        ee_position = get_coordinates([1, 1, 0], iks_plot, [1, 2, 1.5], [-PI / 2, PI / 2, 0], coord_num=3, offset=-1.5)
        ee_position_ground = np.array([ee_position[0], ee_position[1], -1.98])
        robot_origin = np.array([0, 0, -1.98])

        line_rho = DashedLine(robot_origin, ee_position_ground)
        line_z = DashedLine3D(ee_position, ee_position_ground)

        rho = TexText(r"$\rho = \sqrt{x^2 + y^2}$", font_size=36).fix_in_frame().shift(RIGHT * 1.5 + DOWN * 2)
        z = TexText(r"$z$", font_size=36).fix_in_frame().shift(RIGHT * 3)
        rho.rotate(angle=PI / 8, axis=UP, about_point=rho.get_center())

        theta1 = TexText(r"$\theta_1$", font_size=36).fix_in_frame().shift(LEFT * 0.4 + DOWN)
        theta2 = TexText(r"$\theta_2$", font_size=36).fix_in_frame().shift(RIGHT * 1.1 + DOWN * 1.2)
        theta3 = TexText(r"$\theta_3$", font_size=36).fix_in_frame().shift(RIGHT * 2 + UP * 0.8)
        caption = TexText(r"An example of a 3R robot", font_size=36).fix_in_frame().to_edge(BOTTOM).shift(DOWN)
        singularity_comment = TexText(r"""\begin{minipage}{5 cm}\centering $\det(\mathbf{J})$ is not a function of $\theta_1$\end{minipage}""", font_size=36).fix_in_frame().to_edge(RIGHT)
        torus_comment = TexText(r"""\begin{minipage}{5 cm}\centering $\theta_2, \theta_3 \in \mathbb{T}^2$\end{minipage}""", font_size=36).fix_in_frame().next_to(singularity_comment, DOWN)

        self.add(rob_ins, caption)
        self.wait()
        self.play(*[FadeIn(obj) for obj in [theta1, theta2, theta3]])
        self.wait()
        self.play(ShowCreation(line_rho))
        self.play(ShowCreation(line_z))
        self.wait()
        self.play(*[FadeIn(obj) for obj in [rho, z]])
        self.play(*[FadeOut(obj) for obj in [theta1, theta2, theta3]])
        self.wait(0.5)
        self.play(*[FadeOut(obj) for obj in [rho, z]])

        total_ii = 90
        self.add(TracedPath(line_rho.get_end, stroke_color=GOLD_E, stroke_width=3))
        for ii in range(total_ii + 1):
            self.play(*[Rotate(obj, axis=OUT, angle=TAU / total_ii, about_point=robot_origin) for obj in [line_rho, line_z, rob_ins]], run_time=0.05)

        self.wait()

        self.embed()


class IkmExplanation(Scene):

    def construct(self) -> None:
        line_gap = 3
        text_R = TexText(r"$R = \rho^2 + z^2$").shift(UP * 3)
        text_Rz = TexText(r"$R, \,z = g(\theta_2, \theta_3$)").next_to(text_R, DOWN * line_gap)

        text_eqR = TexText(r"$R = (F_1 \cos\theta_2 + F_2 \sin\theta_2)2a_1 + F_3$").next_to(text_Rz, DOWN * line_gap)
        text_eqz = TexText(r"$z = (F_1 \sin\theta_2 - F_2 \cos\theta_2)\sin\alpha_1 + F_4$").next_to(text_eqR, DOWN * line_gap)

        info_f = TexText(r"$F_i = g_i(\theta_3)$").next_to((text_eqR.get_edge_center(RIGHT) + text_eqz.get_edge_center(RIGHT)) / 2 + RIGHT)

        text_eqR2 = TexText(r"$R - F_3 = (F_1 \cos\theta_2 + F_2 \sin\theta_2)2a_1$").next_to(text_Rz, DOWN * line_gap)
        text_eqz2 = TexText(r"$z - F_4 = (F_1 \sin\theta_2 - F_2 \cos\theta_2)\sin\alpha_1$").next_to(text_eqR, DOWN * line_gap)

        text_eqR3 = TexText(r"$\dfrac{R - F_3}{2a_1} = (F_1 \cos\theta_2 + F_2 \sin\theta_2)$").next_to(text_Rz, DOWN * line_gap)
        text_eqz3 = TexText(r"$\dfrac{z - F_4}{\sin\alpha_1} = (F_1 \sin\theta_2 - F_2 \cos\theta_2)$").next_to(text_eqR, DOWN * line_gap)

        final_eqn = TexText(r"$\left(\dfrac{R - F_3}{2a_1}\right)^2 + \left(\dfrac{z - F_4}{\sin\alpha_1}\right)^2 = F_1^2 + F_2^2$").next_to(text_eqz3, DOWN * line_gap)
        surrounding_final_eq3 = SurroundingRectangle(final_eqn, buff=0.2)
        exceptions = TexText(r"$a_1, \sin\alpha_1 \neq 0$").next_to(surrounding_final_eq3, RIGHT).shift(RIGHT * 0.7)
        # Animations
        for obj in [text_R, text_Rz, text_eqR, text_eqz, info_f]:
            self.play(FadeIn(obj))
            self.wait()

        self.play(ReplacementTransform(text_eqR, text_eqR2))
        self.wait(2)
        self.play(ReplacementTransform(text_eqz, text_eqz2))
        self.wait(2)
        self.play(ReplacementTransform(text_eqR2, text_eqR3))
        self.wait(2)
        self.play(ReplacementTransform(text_eqz2, text_eqz3))
        self.wait(2)
        self.play(FadeIn(final_eqn))
        self.play(FadeIn(exceptions))
        self.play(ShowCreation(surrounding_final_eq3))
        self.wait()

        self.embed()


class ConicExplanation(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        conic_sol_set = get_conic_solutions(R=R, z=z)
        # print(conic_sol_set)
        conic_intersections = []
        for sol in conic_sol_set:
            conic_intersections.append(Dot().move_to(np.array([sol[0], sol[1], 0])))

        # planes for the plots
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), xvalue=(-2, 2), yvalue=(-2, 2),
                                                  label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        rval = 2
        conic_plot = ImplicitFunction(lambda c3,
                                             s3: 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625,
                                      color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval))
        circle = Circle(radius=2 / rval, stroke_color=PURPLE_D)

        text_conic_space = TexText(r"$c_3s_3$ plane \\$c_3 \gets \cos\theta_3, s_3 \gets \sin\theta_3$").next_to(plot_conic_space, DOWN * 3)

        conic_explanation = TexText(r"The equation represents a quadric in $\cos\theta_3$-$\sin\theta_3$ plane")
        conic_equation = TexText(r"$A_{xx}\,c_3^2 + 2A_{xy}\,c_3s_3 + A_{yy}\,s_3^2 + 2B_x\,c_3 + 2B_y\,s_3 + C = 0$").next_to(conic_explanation, DOWN * 2)
        list_first_index = [0, 8, 16, 23, 29, 35]
        list_second_index = [3, 11, 19, 26, 32, 36]
        list_value = [6.75, 9, 2.2, 2.2, 4.1, -1.9]
        [conic_equation[start:end].set_color(YELLOW_D) for start, end in zip(list_first_index, list_second_index)]

        # Animations
        self.play(*[FadeIn(obj) for obj in [conic_explanation, conic_equation]])
        self.wait(5)
        self.play(FadeOut(conic_explanation), conic_equation.animate.to_edge(UP))
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_conic_space, box_cs]])
        self.wait()
        self.play(FadeIn(text_conic_space))
        self.wait(2)
        # self.play(*[ReplacementTransform(conic_equation[start:end], TexText(r"$" + str(obj2) + r"$").move_to(conic_equation[start].get_center())) for start, end, obj2 in zip(list_first_index, list_second_index, list_value)])
        conic_equation_valued = TexText(r"$6.75\,c_3^2 + 9\,c_3s_3 + 2.25\,s_3^2 + 2.21\,c_3 + 4.11\,s_3 -1.91 = 0$").move_to(conic_equation.get_center())
        self.play(ReplacementTransform(conic_equation, conic_equation_valued))
        self.wait(0.5)
        self.play(ShowCreation(conic_plot))
        self.wait()
        circle_equation = TexText(r"$c_3^2 + s_3^2 = 1$").next_to(conic_equation, DOWN)
        self.play(FadeIn(circle_equation))
        self.wait()
        self.play(ShowCreation(circle))
        self.wait()
        self.play(*[FadeIn(obj) for obj in conic_intersections])
        self.wait()

        self.embed()


class PseudoSingularity(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5)
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4), label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2))
        pseudo_singularity = ImplicitFunction(get_ps(), color=RED_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2))
        rval = 2
        critical_value = ImplicitFunction(get_critv(), color=BLUE_D, x_range=(0, 6), y_range=(-4, 4))
        critical_value.stretch(2 / 3, 0).stretch(4 / 8, 1).move_to(
            plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)

        vector_theta_list = []
        first_rho, first_z = rho, z
        first_theta_list = theta_list
        joint_dot = Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(first_theta_list[0], first_theta_list[1]))
        work_dot = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(first_rho, first_z))

        # text for graphs
        text_joint_space = TexText("joint space", font_size=36).next_to(plot_joint_space, DOWN * 3)
        text_work_space = TexText(r"work space projection \\on $\rho$-$z$ plane \\($\rho = \sqrt{x^2 + y^2}$)", font_size=36).next_to(plot_work_space, DOWN * 3)

        # text for explanation

        # iks = get_ikin(theta_list=[-3, -0.5])
        # print(iks)
        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
               [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
               [1.72565092649699, -0.352336624163169, -2.014417810173952],
               [-1.44839814046463, -3.00000000000000, -0.49999999999999994]]

        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]

        start_position = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))

        # Animations
        # self.add(get_background().fix_in_frame())
        for obj in [plot_joint_space, plot_work_space, box_js, box_ws]:
            self.add(obj.fix_in_frame())

        for obj in [text_joint_space, text_work_space]:
            self.play(FadeIn(obj.fix_in_frame()))

        for obj in [critical_points.match_plot(plot_joint_space), critical_value]:
            self.play(ShowCreation(obj.fix_in_frame()))
            self.wait()

        # for obj in [pseudo_singularity.match_plot(plot_joint_space)]:
        #     self.play(ShowCreation(obj.fix_in_frame()))
        #     self.wait()

        ps_transformed = pseudo_singularity.copy().match_plot(plot_joint_space)
        self.play(ReplacementTransform(critical_value.copy(), ps_transformed), run_time=3)
        self.add(critical_points)
        self.embed()


class ReducedAspects(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(label=True, box_opacity=0.5)
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4), label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")
        plot_work_space2, box_ws2 = get_small_plot(edge=LEFT, label=True, xvalue=(0, 6), yvalue=(-4, 4), label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        pseudo_singularity = ImplicitFunction(get_ps(), color=RED_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space)

        rval = 2
        critical_value = ImplicitFunction(get_critv(), color=PURPLE_D, x_range=(0, 6), y_range=(-4, 4))
        critical_value2 = ImplicitFunction(get_critv(), color=PURPLE_D, x_range=(0, 6), y_range=(-4, 4))
        critical_value.stretch(2 / 3, 0).stretch(4 / 8, 1).move_to(plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)
        critical_value2.stretch(2 / 3, 0).stretch(4 / 8, 1).move_to(plot_work_space2.get_origin() + RIGHT * critical_value.get_width() / 2)

        vector_theta_list = []
        first_rho, first_z = rho, z
        first_theta_list = theta_list
        joint_dot = Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(first_theta_list[0], first_theta_list[1]))
        work_dot = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(first_rho, first_z))

        # reduced aspects
        reduced_aspect1 = Group()
        reduced_aspect2 = Group()
        reduced_aspect3 = Group()
        right_workspace = Group()
        left_workspace = Group()
        right_workspace_two = Group()
        left_workspace_two = Group()
        ps_eqn = get_ps()
        det_eqn = get_det(robot_type="philippe")
        radius = 0.04
        radius2 = 0.04
        for t2 in np.arange(-3.14, 3.15, 0.1):
            for t3 in np.arange(-3.14, 3.15, 0.1):
                if det_eqn(t2, t3) > 0:
                    ee_pos = get_fkin(theta_list=[0, t2, t3], robot_type="philippe")
                    rho_temp = np.sqrt(ee_pos[0] ** 2 + ee_pos[1] ** 2)
                    z_temp = ee_pos[2]

                    if ps_eqn(t2, t3) < 0 and (t3 > 1 or t3 < -2):
                        color = LIGHT_PINK
                        reduced_aspect1.add(Dot(fill_color=color, radius=radius).move_to(plot_joint_space.c2p(t2, t3)))
                        right_workspace.add(Dot(fill_color=color, radius=radius2).move_to(plot_work_space.c2p(rho_temp, z_temp)))
                    elif ps_eqn(t2, t3) < 0 and t3 < 1:
                        color = YELLOW_D
                        reduced_aspect3.add(Dot(fill_color=color, radius=radius).move_to(plot_joint_space.c2p(t2, t3)))
                        left_workspace.add(Dot(fill_color=color, radius=radius2).move_to(plot_work_space2.c2p(rho_temp, z_temp)))
                    else:
                        color = BLUE_D
                        reduced_aspect2.add(Dot(fill_color=color, radius=radius).move_to(plot_joint_space.c2p(t2, t3)))
                        right_workspace_two.add(Dot(fill_color=color, radius=radius2).move_to(plot_work_space.c2p(rho_temp, z_temp)))
                        left_workspace_two.add(Dot(fill_color=color, radius=radius2).move_to(plot_work_space2.c2p(rho_temp, z_temp)))

        # texts
        frame.scale(1.02)
        text_reduced_aspects = Text("In the domain of reduced aspects, the forward kinematic function is a bijection", line_width=12).to_edge(DOWN, buff=0.1)
        text_reduced_aspects[58:67].set_color(YELLOW_D)

        # Animations
        self.add(plot_joint_space, plot_work_space, plot_work_space2, box_js, box_ws, box_ws2)
        self.add(critical_points, critical_value, critical_value2)
        self.add(pseudo_singularity, critical_points)
        self.wait(2)

        self.play(FadeIn(text_reduced_aspects))
        self.wait(2)

        self.play(FadeIn(reduced_aspect1))
        reduced_aspect1_copy = reduced_aspect1.copy()
        self.play(Transform(reduced_aspect1_copy, right_workspace))
        self.add(pseudo_singularity, critical_points, critical_value, critical_value2)
        self.wait(2)

        self.play(FadeIn(reduced_aspect3))
        reduced_aspect3_copy = reduced_aspect3.copy()
        self.play(Transform(reduced_aspect3_copy, left_workspace))
        self.add(pseudo_singularity, critical_points, critical_value, critical_value2)
        self.wait(2)

        self.play(FadeIn(reduced_aspect2))
        reduced_aspect2_copy = reduced_aspect2.copy()
        reduced_aspect2_copy2 = reduced_aspect2.copy()
        self.play(Transform(reduced_aspect2_copy, right_workspace_two))
        self.play(Transform(reduced_aspect2_copy2, left_workspace_two))
        self.add(pseudo_singularity, critical_points, critical_value, critical_value2)
        self.wait(2)

        self.embed()


class ComponentsCV(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)

        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5, big_plot=True)
        plot_conic_space, box_cs = get_small_plot(edge=LEFT, label=True, yconfig=dict(include_ticks=True), xconfig=dict(include_ticks=True), label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center", big_plot=True)
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4), label_list=[r"""$z$""", r"""$\rho$"""], label_position="center", big_plot=True)

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        pseudo_singularity = ImplicitFunction(get_ps(), color=RED_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        rval = 3.2

        critical_value = ImplicitFunction(get_critv(), color=PURPLE_D, x_range=(0, 6), y_range=(-4, 4)).shift(RIGHT)
        cv_left = ImplicitFunction(get_critv(), color=YELLOW_D, x_range=(1.3, 1.8), y_range=(-0.5001, 0.5001)).shift(RIGHT)
        cv_right = ImplicitFunction(get_critv(), color=WHITE, x_range=(2.5, 2.95), y_range=(-1.961, 1.961)).shift(RIGHT)
        cv_top = ImplicitFunction(get_critv(), color=GREEN_D, x_range=(1.3, 2.5), y_range=(0.5, 1.975)).shift(RIGHT)
        cv_bottom = ImplicitFunction(get_critv(), color=BLUE_D, x_range=(1.3, 2.5), y_range=(-1.975, -0.5)).shift(RIGHT)

        iks_center = [[-1.491861476, 3.141592654, -0.5796802909], [-0.07893485129, 3.141592654, 2.150476617], [0.7965961208, 0., 2.743519169], [1.701495424, 0., -2.100018061]]
        iks_up = [[0.2840706613, -1.922334877, 2.245492930], [0.2685061975, -1.951916476, 2.235176544], [1.765963442, -0.6860193979, -1.857738340], [-1.391245082, -2.841537799, -0.4086336919]]
        iks_down = [[-1.391245082, 2.841537799, -0.4086336919], [1.765963442, 0.6860193979, -1.857738340], [0.2685061975, 1.951916476, 2.235176544], [0.2840706613, 1.922334877, 2.245492930]]
        iks_left = [[-1.790999727, 3.141592654, -1.141170999], [0.2202034002, 3.141592654, 2.711967326], [1.209676663, 0., -2.863544433], [1.288414881, 0., -2.776139767]]
        iks_right = [[-0.8871662877, 3.141592654, 0.5877006003], [-0.6836300391, 3.141592654, 0.9830957265], [0.7563817740, 0., 2.309275856], [1.741709771, 0., -1.665774753]]

        group_dots_js = Group()
        for ii in iks_center:
            group_dots_js.add(Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(ii[1], ii[2])))

        dot_ws = Dot(fill_color=WHITE).move_to([3.262, 0, 0])

        # cusp points
        cusp_points = [[1.384, 0.5], [2.478, 1.959], [1.384, -0.5], [2.478, -1.959]]
        cusps_group = Group()
        for ii in cusp_points:
            cusps_group.add(Dot(fill_color=RED_D).move_to([ii[0] + 1, ii[1], 0]))

        # texts
        components_cv = Text("components of locus of critical value").to_edge(BOTTOM, buff=0)

        # Animations

        self.add(critical_points, pseudo_singularity)
        self.add(critical_value)
        self.add(plot_joint_space, box_js)
        self.play(FadeIn(components_cv))
        self.wait()

        self.play(FadeIn(cusps_group))
        self.wait(3)

        for obj in [cv_left, cv_right, cv_top, cv_bottom]:
            self.play(FadeIn(obj))
            self.wait()

        self.play(FadeIn(group_dots_js), FadeIn(dot_ws))
        self.wait(2)

        for obj in [cv_left, cv_right, cv_top, cv_bottom, cusps_group]:
            self.play(FadeOut(obj))
            self.wait()

        for ws_points, iks_group in zip([[2.262, 1], [2.262, -1], [1.6, 0.272], [2.9, 0.272]], [iks_up, iks_down, iks_left, iks_right]):
            group_dots_js_new = Group()
            for ii in iks_group:
                group_dots_js_new.add(Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(ii[1], ii[2])))
            self.remove(group_dots_js)
            self.wait(0.1)
            self.play(FadeIn(dot_ws.move_to([ws_points[0] + 1, ws_points[1], 0])), FadeIn(group_dots_js_new))
            self.wait(3)
            group_dots_js = group_dots_js_new

        group_joint = Group(plot_joint_space, box_js, critical_points, pseudo_singularity, group_dots_js)
        self.wait()
        self.play(group_joint.animate.shift(UP * 12))

        R = 2.262 ** 2
        z = 0
        conic_plot = ImplicitFunction(lambda c3,
                                             s3: 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625,
                                      color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval)).shift(LEFT*4)
        circle = Circle(radius=1, stroke_color=PURPLE_D).shift(LEFT*4)
        conic_sol_set = get_conic_solutions(R=R, z=z)
        conic_intersections = []
        for sol in conic_sol_set:
            conic_intersections.append(Dot().fix_in_frame().move_to(np.array([sol[0] - 4, sol[1], 0])))
        conic_intersections_prev = conic_intersections
        self.add(circle, conic_plot, dot_ws.move_to([3.262, 0, 0]), *conic_intersections)
        self.wait(3)

        for ws_points in [[2.25, 0.98], [1.63, 0.272], [2.25, -0.98], [2.915, 0.272]]:
            R = ws_points[0] ** 2
            if abs(ws_points[1]) > 0.9:
                print("adjusted z")
                z = np.sign(ws_points[1]) * 0.8
            else:
                z = ws_points[1]

            conic_plot_new = ImplicitFunction(lambda c3,
                                                     s3: 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625,
                                              color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval)).shift(LEFT*4)

            conic_sol_set = get_conic_solutions(R=R, z=z)
            # print(conic_sol_set)
            conic_intersections = []
            for sol in conic_sol_set:
                conic_intersections.append(Dot().fix_in_frame().move_to(np.array([sol[0] - 4, sol[1], 0])))

            self.remove(conic_plot, *conic_intersections_prev)
            self.wait(0.1)
            self.play(FadeIn(dot_ws.move_to([ws_points[0] + 1, ws_points[1], 0])))
            self.add(*conic_intersections, conic_plot_new)
            conic_intersections_prev = conic_intersections
            conic_plot = conic_plot_new
            self.wait(3)

        self.embed()


class CuspContradict(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        a_val = 1.5
        equation_curve = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(PI/4, 3*PI/4, 0.1))
        equation_curve_b = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(5*PI/4, 7*PI/4, 0.1))
        equation_curve_closing_left = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(3*PI/4, 5*PI/4, 0.1)).shift(LEFT*a_val*0.95)
        equation_curve_closing_right = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(7*PI/4, 9*PI/4, 0.1)).shift(RIGHT*a_val*0.95)

        eq_group = Group(equation_curve, equation_curve_b)
        eq_curve_right = eq_group.copy().shift(RIGHT * a_val * 0.95)
        eq_curve_left = eq_group.copy().shift(LEFT * a_val * 0.95)
        circle = Circle(radius=2, stroke_color=PURPLE_D)
        circle2 = Circle(radius=0.5, fill_color=YELLOW_D, fill_opacity=0.75, stroke_color=YELLOW_D, stroke_opacity=0.75).shift(LEFT*0.7)
        circle3 = Dot().move_to(circle2.get_center())

        top_curves_a = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(PI/4, 2*PI/4, 0.1), color=YELLOW_D)
        top_curves_b = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t)**2), a_val * sin(2*t) / (2 * (1 + sin(t)**2)), 0]), t_range=(5*PI/4, 6*PI/4, 0.1), color=YELLOW_D).shift(RIGHT*a_val*0.95)
        top_curves = Group(top_curves_a, top_curves_b)
        top_curves_left = top_curves.copy().shift(LEFT*a_val*0.95)

        top_curves_a2 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(PI / 4, 2 * PI / 4, 0.1), color=RED_D)
        top_curves_b2 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(5 * PI / 4, 6 * PI / 4, 0.1), color=RED_D).shift(RIGHT * a_val * 0.95)
        top_curves2 = Group(top_curves_a2, top_curves_b2)

        bottom_curves_a = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(6*PI / 4, 7 * PI / 4, 0.1), color=RED_D)
        bottom_curves_b = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(2 * PI / 4, 3*PI/4, 0.1), color=RED_D).shift(RIGHT * a_val * 0.95)
        bottom_curves = Group(bottom_curves_a, bottom_curves_b)
        bottom_curves_left = bottom_curves.copy().shift(LEFT*a_val*0.95)

        bottom_curves_a2 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(6 * PI / 4, 7 * PI / 4, 0.1), color=YELLOW_D)
        bottom_curves_b2 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(2 * PI / 4, 3 * PI / 4, 0.1), color=YELLOW_D).shift(RIGHT * a_val * 0.95)
        bottom_curves2 = Group(bottom_curves_a2, bottom_curves_b2)

        # ellipse plot
        major = 1.5
        e = np.sqrt(0.75)
        cen_x, cen_y = 0, 0
        circle_ellipse = Circle(stroke_color=PURPLE_D)
        equation_ellipse = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)

        group_dots = Group()
        solutions = solve_conic_circle()
        for sol in solutions:
            group_dots.add(Dot().move_to([sol[0], sol[1], 0]))

        # Animations
        self.add(equation_curve,  equation_curve_b, circle, eq_curve_right, eq_curve_left, equation_curve_closing_left, equation_curve_closing_right)
        self.wait(5)
        self.play(FadeIn(circle2))
        self.wait()
        self.play(FadeIn(circle3))
        self.wait()
        self.play(FadeOut(circle2))
        self.add(top_curves, bottom_curves, top_curves_left, bottom_curves_left)
        self.wait(5)
        self.play(FadeOut(top_curves), FadeOut(bottom_curves))
        self.wait(5)
        self.play(FadeIn(top_curves2), FadeIn(bottom_curves2))
        self.wait()

        frame.shift(LEFT*4)
        jj = [[0, 0.6, 0.1], [0.5, -0.6, -0.1], [0, 0.26, 0.05], [0.25, -0.26, -0.05], [-0.25, 0.01, 0.05]]
        ll = 0
        list_color = [YELLOW_D, RED_D, YELLOW_D, RED_D, WHITE]
        offset = 8
        self.add(circle_ellipse.shift(LEFT*offset))
        for kk in jj:
            for ii in np.arange(kk[0], kk[1], kk[2]):
                if ll >= 2:
                    y_cen = 0
                    x_cen = ii
                else:
                    x_cen = 0
                    y_cen = ii
                solutions = solve_conic_circle(x_cen=x_cen, y_cen=y_cen)
                cen_x = x_cen
                cen_y = y_cen
                equation_ellipse_new = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)

                group_dots_new = Group()
                for sol in solutions:
                    group_dots_new.add(Dot().move_to([sol[0], sol[1], 0]))



                self.remove(equation_ellipse, group_dots)
                self.add(equation_ellipse_new.shift(LEFT*offset), group_dots_new.shift(LEFT*offset))
                self.wait(0.12)
                # self.play(*[FadeOut(obj) for obj in [equation_ellipse, group_dots]], run_time=0.05)
                # self.play(*[FadeIn(obj) for obj in [equation_ellipse_new, group_dots_new]], run_time=0.05)
                equation_ellipse = equation_ellipse_new
                group_dots = group_dots_new

            if ll < 4:
                box_new = SurroundingRectangle(equation_ellipse, color=list_color[ll])
                self.add(box_new)
                box = box_new
                self.wait(3)
                self.remove(box)
            ll += 1

        major = 2
        equation_ellipse_new = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
        self.remove(group_dots)
        dot1 = Dot().move_to(LEFT * offset + RIGHT)
        dot2 = Dot().move_to(LEFT * offset - RIGHT)
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip([equation_ellipse, circle3], [equation_ellipse_new.shift(LEFT*offset), circle3.shift(RIGHT*a_val)])])
        self.add(dot1, dot2)
        circle3.move_to(circle.get_center())
        text_node = Text("The node represents two simultaneous singularities", line_width=4).next_to(equation_ellipse, DOWN)
        self.play(FadeIn(text_node))
        self.wait(3)
        major = 1.5
        equation_ellipse_new = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
        self.remove(dot1, dot2)
        self.play(Transform(equation_ellipse, equation_ellipse_new.shift(LEFT * offset)))
        self.add(group_dots)
        self.play(FadeOut(text_node))
        self.wait(2)

        circle3.move_to(circle2.get_center())
        self.add(TracedPath(circle3.get_center, stroke_width=3, stroke_color=WHITE))
        for ii in np.arange(0, TAU, 0.2):
            self.play(circle3.animate.move_to([cos(PI - ii) * 0.75, sin(PI - ii) * 0.75, 0]), run_time=0.1)
        self.embed()


class CandyCase(Scene):

    def construct(self) -> None:
        frame = self.camera.frame
        a_val = 1
        small_circle = Circle(radius=0.5, stroke_color=WHITE)
        equation_curve_a = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(PI/2, 3 * PI/2, 0.1))
        # equation_curve_b = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(0, TAU, 0.1))
        equation_close_left = Group(equation_curve_a)
        equation_curve2 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(0, PI/2, 0.1))
        equation_curve3 = ParametricCurve(lambda t: np.array([a_val * cos(t) / (1 + sin(t) ** 2), a_val * sin(2 * t) / (2 * (1 + sin(t) ** 2)), 0]), t_range=(3*PI/2, TAU, 0.1))
        equation_close_right = Group(equation_curve3, equation_curve2)
        circle = Circle(radius=2, stroke_color=PURPLE_D)
        circle3 = Dot()

        side_dots = Group(Dot(fill_color=RED_D).shift(LEFT * 1.5), Dot(fill_color=RED_D).shift(RIGHT * 1.5))
        circle_side = Circle(radius=1, stroke_color=PURPLE_D).shift(LEFT*5)
        line_tangent = Line(LEFT*6 + UP*3, LEFT*6 + DOWN * 6)
        dot_tangent = Dot(fill_color=RED_D).move_to(LEFT*6)

        # Texts
        text_candy = Text("The candy case: a possible solution for nonsingular change of solutions with only nodes in the workspace", line_width=12).to_edge(BOTTOM, buff=0.1)
        text_candy[0:12].set_color(YELLOW_D)

        # Animations
        self.add(circle, equation_close_left.shift(RIGHT*1.5), small_circle, equation_close_right.shift(LEFT*1.5))
        self.play(FadeIn(text_candy))
        self.wait(5)
        self.add(TracedPath(circle3.get_center, stroke_width=3, stroke_color=WHITE))
        for ii in np.arange(0, TAU, 0.2):
            self.play(circle3.animate.move_to([cos(PI - ii) * 0.9 + 0.8, sin(PI - ii) * 0.75, 0]), run_time=0.1)
        self.wait(5)
        self.play(FadeIn(side_dots))
        self.wait(3)
        self.play(*[FadeIn(obj) for obj in [circle_side, line_tangent, dot_tangent]])
        self.wait(2)
        self.embed()


class AllConics(Scene):

    def construct(self) -> None:
        # Hyperbola
        list_d = [0, 1, 0]
        list_a = [1, 2, 3/2]
        list_alpha = [PI/3, PI/6, 0]
        ee_position  = get_fkin([0, -0.742, 2.628])
        value_R = ee_position[0] ** 2 + ee_position[1] ** 2 + ee_position[2] ** 2
        value_z = ee_position[2]
        conic_hyperbola_lambda = get_conic(value_R, value_z, list_d, list_a, list_alpha)
        c3, s3 = var('c3, s3')
        robot_hyperbola = get_robot_instance(theta_list=[0, -0.742, 2.628], robot_type="philippe", offset=0, show_frame=False)
        robot_hyperbola.rotate(angle=-PI / 6, axis=RIGHT, about_point=[0, 0, -0.48])
        robot_hyperbola.shift(DOWN)
        conic_hyperbola = ImplicitFunction(conic_hyperbola_lambda, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
        box_hyperbola = Square(side_length=6.8, stroke_color=WHITE)
        circle_hyperbola = Circle(stroke_color=PURPLE_D)
        dots_hyperbola = Group()
        intersections_hyperbola = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = []
        for sol in intersections_hyperbola:
            first_sol_imag = sol[c3].as_real_imag()
            second_sol_imag = sol[s3].as_real_imag()
            if abs(first_sol_imag[1]) < 1e-5 and abs(second_sol_imag[1]) < 1e-5:
                real_intersections.append([first_sol_imag[0], second_sol_imag[0]])

        for ii in real_intersections:
            dots_hyperbola.add(Dot().move_to([ii[0], ii[1], 0]))

        text_hyperbola = Text("Hyperbola").next_to(box_hyperbola, UP * 0.5)
        caption_hyperbola = TexText(r"$\mathbf{d} = [0, 1, 0], \mathbf{a} = [1, 2, 1.5], \mathbf{\alpha} = [\frac{\pi}{2}, \frac{\pi}{2}, 0]$", font_size=28).next_to(box_hyperbola, DOWN * 0.5)
        group_hyperbola = Group(conic_hyperbola, circle_hyperbola, dots_hyperbola, box_hyperbola, text_hyperbola, caption_hyperbola)

        # Ellipse
        list_d = [0, 1, 0]
        list_a = [1, 3, 3 / 2]
        list_alpha = [PI / 6, PI / 3, 0]
        ee_position = get_fkin(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        value_R = ee_position[0] ** 2 + ee_position[1] ** 2 + ee_position[2] ** 2
        value_z = ee_position[2]
        c3, s3 = var('c3, s3')
        robot_ellipse = get_robot_instance(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha, offset=0, show_frame=False)
        robot_ellipse.rotate(angle=-PI / 3, axis=RIGHT, about_point=[0, 0, -0.48])
        robot_ellipse.shift(DOWN+LEFT)
        conic_ellipse = ImplicitFunction(get_conic(value_R, value_z, list_d, list_a, list_alpha), x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
        box_ellipse = Square(side_length=6.8, stroke_color=WHITE)
        circle_ellipse = Circle(stroke_color=PURPLE_D)
        dots_ellipse = Group()
        intersections_ellipse = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = []
        for sol in intersections_ellipse:
            first_sol_imag = sol[c3].as_real_imag()
            second_sol_imag = sol[s3].as_real_imag()
            if abs(first_sol_imag[1]) < 1e-5 and abs(second_sol_imag[1]) < 1e-5:
                real_intersections.append([first_sol_imag[0], second_sol_imag[0]])

        for ii in real_intersections:
            dots_ellipse.add(Dot().move_to([ii[0], ii[1], 0]))

        text_ellipse = Text("Ellipse").next_to(box_ellipse, UP * 0.5)
        caption_ellipse = TexText(r"$\mathbf{d} = [0, 1, 0], \mathbf{a} = [1, 3, 1.5], \mathbf{\alpha} = [\frac{\pi}{6}, \frac{\pi}{3}, 0]$", font_size=28).next_to(box_ellipse, DOWN * 0.5)
        group_ellipse = Group(conic_ellipse, circle_ellipse, dots_ellipse, box_ellipse, text_ellipse, caption_ellipse)

        # Parabola
        list_d = [0, 2.828, 0.5]
        list_a = [1, 2, 1.5]
        list_alpha = [PI / 6, PI / 3, 0]
        ee_position = get_fkin(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        value_R = 13
        value_z = 3
        c3, s3 = var('c3, s3')
        robot_parabola = get_robot_instance(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha, offset=0, show_frame=False)
        robot_parabola.rotate(angle=-PI / 2, axis=RIGHT, about_point=[0, 0, -0.48])
        robot_parabola.shift(DOWN)
        conic_parabola = ImplicitFunction(get_conic(value_R, value_z, list_d, list_a, list_alpha), x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
        box_parabola = Square(side_length=6.8, stroke_color=WHITE)
        circle_parabola = Circle(stroke_color=PURPLE_D)
        dots_parabola = Group()
        intersections_parabola = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = []
        for sol in intersections_parabola:
            first_sol_imag = sol[c3].as_real_imag()
            second_sol_imag = sol[s3].as_real_imag()
            if abs(first_sol_imag[1]) < 1e-5 and abs(second_sol_imag[1]) < 1e-5:
                real_intersections.append([first_sol_imag[0], second_sol_imag[0]])

        for ii in real_intersections:
            dots_parabola.add(Dot().move_to([ii[0], ii[1], 0]))
        text_parabola = Text("Parabola").next_to(box_parabola, UP * 0.5)
        caption_parabola = TexText(r"$\mathbf{d} = [0, 2.828, 0.5], \mathbf{a} = [1, 2, 1.5], \mathbf{\alpha} = [\dfrac{\pi}{3}, \dfrac{\pi}{6}, 0]$", font_size=28).next_to(box_parabola, DOWN * 0.5)
        group_parabola = Group(conic_parabola, circle_parabola, dots_parabola, box_parabola, text_parabola, caption_parabola)

        # Animations
        frame = self.camera.frame
        self.add(robot_hyperbola)
        self.wait(2)
        self.play(FadeOut(robot_hyperbola))
        self.add(group_hyperbola)
        self.wait(3)
        self.play(group_hyperbola.animate.shift(LEFT*7))
        self.wait()

        self.add(robot_ellipse)
        self.wait(2)
        self.play(FadeOut(robot_ellipse))
        self.add(group_ellipse)
        self.wait(3)
        self.play(group_ellipse.animate.shift(RIGHT * 7))
        self.wait()

        self.add(robot_parabola)
        self.wait(2)
        self.play(FadeOut(robot_parabola))
        self.add(group_parabola)
        self.wait(3)

        self.play(frame.animate.scale(1.5))
        self.wait()

        self.embed()


class DegenerateConics(Scene):

    def construct(self) -> None:
        # conic explanation
        conic_equation = TexText(r"$A_{xx}\,c_3^2 + 2A_{xy}\,c_3s_3 + A_{yy}\,s_3^2 + 2B_x\,c_3 + 2B_y\,s_3 + C = 0$").to_edge(UP, buff=1)
        matrix_nature = Tex(r"\mathbf{N} = \begin{bmatrix} A_{xx} & A_{xy} \\ A_{xy} & A_{yy}\end{bmatrix}").next_to(conic_equation, DOWN * 3)
        matrix_nature[3:len(matrix_nature)-1].set_color(YELLOW_D)
        caption_matrix_nature = Text("nature of the conic", font_size=28, line_width=3).next_to(matrix_nature, DOWN)
        # bracing_matrix_nature = Brace(matrix_nature, direction=RIGHT).shift(RIGHT * 0.2)
        conic_natures = TexText(r"$\mathbf{N} < 0$:  hyperbola\\ $\mathbf{N} > 0$: ellipse\\ $\mathbf{N} = 0$: parabola", alignment="", font_size=32).next_to(matrix_nature, buff=0.6)
        matrix_degeneracy = Tex(r"\mathbf{D} = \begin{bmatrix} A_{xx} & A_{xy} & B_x \\ A_{xy} & A_{yy} & B_y \\ B_x & B_y & C\end{bmatrix}").next_to(caption_matrix_nature, DOWN * 3)
        matrix_degeneracy[4:len(matrix_degeneracy)-2].set_color(YELLOW_D)
        caption_matrix_degeneracy = Text("degeneracy of the conic", font_size=28, line_width=5).next_to(matrix_degeneracy, DOWN)
        # bracing_matrix_degeneracy = Brace(matrix_degeneracy, direction=RIGHT).shift(RIGHT * 0.2)

        # Animations
        self.play(FadeIn(conic_equation))
        self.wait()
        self.play(*[FadeIn(obj) for obj in [matrix_nature, caption_matrix_nature, conic_natures]])
        self.wait(3)
        self.play(*[FadeIn(obj) for obj in [matrix_degeneracy, caption_matrix_degeneracy]])
        self.wait(5)
        self.embed()


class DegenerateParabola(Scene):

    def construct(self) -> None:
        # degenerate conditions
        equation_parabola_degeneration = Tex(r"\det(\mathbf{N}) = 0 \\\det(\mathbf{D}) = 0\\B_x^2 + B_y^2 - (A_{xx} + A_{yy})\,C = 0", alignment=R"\centering").to_edge(UP, buff=1)
        title_parabola_degeneration = Text("conditions for a parabola to degenerate into two coincident lines").next_to(equation_parabola_degeneration, UP)
        bracing_equation_parabola_degeneration = Brace(equation_parabola_degeneration, direction=RIGHT).shift(RIGHT * 0.2)

        # det N
        text_detN = TexText(r"Solving for $\det(\mathbf{N}) = 0$ for $d_2$ yields:\\~\\ $d_2 = \dfrac{\pm\sqrt{(a_1 + a_2)(a_1 - a_2)(sa_1 + sa_2)(sa_1 - sa_2)}}{sa_1sa_2}$").next_to(equation_parabola_degeneration, DOWN, buff=1)
        text_replaced2 = TexText(r"Upon replacing $d_2$ into $\det(\mathbf{D}) = 0$").next_to(equation_parabola_degeneration, DOWN, buff=1).shift(LEFT)
        text_Rd2 = TexText(r"$R$ and $z$ take the following form:").next_to(text_replaced2, DOWN)
        text_Rd2_eqn = TexText(r"$R = \dfrac{f(z)}{sa_1^2 - sa_2^2}$\\$z = \dfrac{ca_1}{ca_2}(d_3\,sa_1\,sa_2 + d_2\,ca_2)$").next_to(text_Rd2, DOWN)
        bracing_eqn = Brace(text_Rd2_eqn, RIGHT)
        text_condition = TexText(r"\begin{minipage}{4.5cm}except $sa_1^2 = sa_2^2, ca_2 = 0$, the parabola always degenerates\end{minipage}").next_to(bracing_eqn, RIGHT)

        # Animations
        self.play(*[FadeIn(obj) for obj in [title_parabola_degeneration, equation_parabola_degeneration, bracing_equation_parabola_degeneration]])
        self.wait(5)
        self.play(FadeIn(text_detN))
        self.wait(7)
        self.play(FadeOut(text_detN))
        for obj in [text_replaced2, text_Rd2, text_Rd2_eqn]:
            self.play(FadeIn(obj))
            self.wait()
        self.wait(2)
        self.play(FadeIn(bracing_eqn))
        self.wait()
        self.play(FadeIn(text_condition))
        self.wait()
        self.embed()


class DegenerateParabolaFigures(Scene):

    def construct(self) -> None:
        # figures
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        # Parabola
        list_d = [0, 2.828, 0.5]
        list_a = [1, 2, 1.5]
        list_alpha = [PI / 6, PI / 3, 0]
        ee_position = get_fkin(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        value_R = 1.471**2 + 3.315**2
        value_z = 3.315
        c3, s3 = var('c3, s3')
        # robot_parabola = get_robot_instance(theta_list=[0, -0.742, 2.628], d_list=list_d, a_list=list_a, alpha_list=list_alpha, offset=0, show_frame=False)
        # robot_parabola.rotate(angle=-PI / 2, axis=RIGHT, about_point=[0, 0, -0.48])
        # robot_parabola.shift(DOWN)
        # conic_parabola = ImplicitFunction(get_conic(value_R, value_z, list_d, list_a, list_alpha), x_range=(-2, 2), y_range=(-2, 2), color=BLUE_D)
        conic_parabola = ImplicitFunction(lambda x, y: y + 0.75 * x + 0.5, x_range=(-2, 2), y_range=(-2, 2), color=BLUE_D)
        box_parabola = Square(side_length=6.8, stroke_color=WHITE)
        circle_parabola = Circle(stroke_color=PURPLE_D)
        dots_parabola = Group()
        # intersections_parabola = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = [[-0.973212111192934, 0.229909083394701], [0.493212111192934, -0.869909083394701]]

        for ii in real_intersections:
            dots_parabola.add(Dot().move_to([ii[0], ii[1], 0]))

        dot_ws = Dot(fill_color=RED_D).move_to(RIGHT*4.1+ UP*0.51)

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5)
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), xvalue=(-2, 2), yvalue=(-2, 2),
                                                  label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4),
                                                 label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(d_list=list_d, a_list=list_a, alpha_list=list_alpha), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        circle = Circle(radius=1, stroke_color=PURPLE_D)
        dots_js = Group()

        solutions_js = [[1.74532925199433, 1.05774295820984], [1.74532925199433, -2.25212924503868]]
        for ii in solutions_js:
            dots_js.add(Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(ii[0], ii[1])))

        # critical_value = ImplicitFunction(get_critv_parabola(), color=BLUE_D, x_range=(0, 6), y_range=(0, 6))
        # critical_value.stretch(2 / 3, 0).stretch(4 / 6, 1).move_to(
        #     plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)

        critv_img = ImageMobject("resources/raster_images/parabola_critv_bold.png").move_to(plot_work_space.get_center()).scale(0.91)

        # text
        caption_parabola = Text("Degenerate parabola case: two coincident lines").next_to(box_cs, DOWN)

        # Animations
        self.play(*[FadeIn(obj) for obj in [plot_joint_space, box_js, critical_points, dots_js, caption_parabola]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_conic_space, box_cs, conic_parabola, circle_parabola, dots_parabola]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_work_space, box_ws, critv_img, dot_ws]])
        self.wait()

        self.embed()


class BinaryEllipse(Scene):

    def construct(self) -> None:
        # theorem_ellipse = Text(r"Theorem 3: There are infinitely many binary robots whose associated conic is an ellipse (e > 0)", line_width=12).to_edge(UP, buff=1)
        # theorem_ellipse[7:9].set_color(YELLOW_D)

        # ellipse plot
        major = 1.35
        e = np.sqrt(0.75)
        cen_x, cen_y = 0, 0
        circle_ellipse = Circle(stroke_color=PURPLE_D)
        equation_ellipse = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)

        # Animations
        self.add(equation_ellipse, circle_ellipse)

        for ii in np.arange(0.86, 0.1, -0.02):
            e = ii
            # equation_ellipse_new = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)
            self.play(Transform(equation_ellipse, ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)), run_time=0.1)
            print(ii)

        for ii in np.arange(1.35, 0.85, -0.05):
            major = ii
            self.play(Transform(equation_ellipse, ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)), run_time=0.1)
            print(ii)

        self.embed()


class EllipseLemma(Scene):

    def construct(self) -> None:
        lemma = Text("Lemma:", alignment="", color=YELLOW_D).shift(UP + LEFT * 5.5)
        statement = TexText(r"Consider the unit circle $\mathcal{S}^1$ and a number $e \in (0, 1)$ \begin{enumerate}\item There is an ellipse $\mathcal{C}$ with eccentricity $e$ such that $\#\mathcal{C}\cap\mathcal{S} = 4$ \item As $e \rightarrow 0$, the ellipses with eccentricity $e$ with property $1$ will have centers that approach the center of $\mathcal{S}$ and have minor axes and major axes approach the length of $1$\end{enumerate}", alignment="")

        self.add(lemma)
        self.add(statement.next_to(lemma, DOWN).shift(RIGHT * 5.5))
        self.wait(2)
        self.embed()


class BinaryEllipseFigures(Scene):

    def construct(self) -> None:
        # figures
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        # Parabola
        list_d = [0, 0, 3]
        list_a = [-1503/1879, -1, -1]
        list_alpha = [-2.21, -PI/2, 0]
        ee_position = get_fkin(theta_list=[1, 1, 1], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        value_R = ee_position[0] ** 2 + ee_position[1] ** 2 + ee_position[2] ** 2
        value_z = ee_position[2]
        c3, s3 = var('c3, s3')
        conic_ellipse = ImplicitFunction(get_conic(value_R, value_z, list_d, list_a, list_alpha), x_range=(-2, 2), y_range=(-2, 2), color=BLUE_D)
        box_ellipse = Square(side_length=4, stroke_color=WHITE)
        circle_ellipse = Circle(stroke_color=PURPLE_D)
        dots_ellipse = Group()
        intersections_ellipse = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = []
        for sol in intersections_ellipse:
            first_sol_imag = sol[c3].as_real_imag()
            second_sol_imag = sol[s3].as_real_imag()
            if abs(first_sol_imag[1]) < 1e-5 and abs(second_sol_imag[1]) < 1e-5:
                real_intersections.append([first_sol_imag[0], second_sol_imag[0]])

        for ii in real_intersections:
            dots_ellipse.add(Dot().move_to([ii[0], ii[1], 0]))

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5)
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), xvalue=(-2, 2), yvalue=(-2, 2),
                                                  label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4),
                                                 label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(d_list=list_d, a_list=list_a, alpha_list=list_alpha), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        circle = Circle(radius=1, stroke_color=PURPLE_D)
        dots_js = Group()

        solutions_js = get_ikin(theta_list=[1, 1, 1], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        for ii in solutions_js:
            dots_js.add(Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(ii[1], ii[2])))

        critv_img = ImageMobject("resources/raster_images/ellipse_3.png").move_to(plot_work_space.get_center()).scale(0.85)

        # text
        caption_ellipse = Text("Binary ellipse case").next_to(box_cs, DOWN)

        # Animations
        self.play(*[FadeIn(obj) for obj in [plot_joint_space, box_js, critical_points, dots_js, caption_ellipse]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_conic_space, box_cs, conic_ellipse, circle_ellipse, dots_ellipse]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_work_space, box_ws, critv_img]])
        self.wait()

        self.embed()


class QuatHyperbolaFigures(Scene):

    def construct(self) -> None:
        # figures
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=0, gamma=0)
        frame.scale(1)

        # data for initial start
        # Parabola
        list_d = [0, 1, 2]
        list_a = [1, 2, 3]
        list_alpha = [-PI/2, PI/2, 0]
        # ee_position = get_fkin(theta_list=[1, 1, 1], d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        value_R = 8
        value_z = 2
        c3, s3 = var('c3, s3')
        conic_ellipse = ImplicitFunction(get_conic(value_R, value_z, list_d, list_a, list_alpha), x_range=(-2, 2), y_range=(-2, 2), color=BLUE_D)
        box_ellipse = Square(side_length=4, stroke_color=WHITE)
        circle_ellipse = Circle(stroke_color=PURPLE_D)
        dots_ellipse = Group()
        intersections_ellipse = solve([get_conic(value_R, value_z, list_d, list_a, list_alpha, _sym=True), c3 ** 2 + s3 ** 2 - 1])
        real_intersections = []
        for sol in intersections_ellipse:
            first_sol_imag = sol[c3].as_real_imag()
            second_sol_imag = sol[s3].as_real_imag()
            if abs(first_sol_imag[1]) < 1e-5 and abs(second_sol_imag[1]) < 1e-5:
                real_intersections.append([first_sol_imag[0], second_sol_imag[0]])

        for ii in real_intersections:
            dots_ellipse.add(Dot().move_to([ii[0], ii[1], 0]))

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5)
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), xvalue=(-2, 2), yvalue=(-2, 2),
                                                  label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4),
                                                 label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(d_list=list_d, a_list=list_a, alpha_list=list_alpha), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        circle = Circle(radius=1, stroke_color=PURPLE_D)
        dots_js = Group()

        # solutions_js = get_ikin(rho=2, z=2, d_list=list_d, a_list=list_a, alpha_list=list_alpha)
        solutions_js = [[1.34438802942243, 0.886046225437089, -2.956863088639908], [0.0846272262876878, 0.0, 2.8042851721600264], [2.41346430815699, 8.70499969759453e-16, -2.160784063366742], [-2.91518436793916, -1.25393313851144, -1.7555258917447816]]

        for ii in solutions_js:
            dots_js.add(Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(ii[1], ii[2])))

        critv_img = ImageMobject("resources/raster_images/quat_hyperbola.png").move_to(plot_work_space.get_center()).scale(0.85)

        critical_value = ImplicitFunction(get_critv_hyperbola(), color=BLUE_D, x_range=(0, 8), y_range=(-6, 6))
        critical_value.stretch(4 / 8, 0).stretch(4 / 12, 1).move_to(
            plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)

        # text
        caption_ellipse = Text("Quaternary hyperbola").next_to(box_cs, DOWN)

        # Animations
        self.play(*[FadeIn(obj) for obj in [plot_joint_space, box_js, critical_points, dots_js, caption_ellipse]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_conic_space, box_cs, conic_ellipse, circle_ellipse, dots_ellipse]])
        self.wait()
        self.play(*[FadeIn(obj) for obj in [plot_work_space, box_ws, critical_value]])
        self.wait()

        self.embed()


class QuatHyperbola(Scene):

    def construct(self) -> None:
        # theorem_ellipse = Text(r"Theorem 3: There are infinitely many binary robots whose associated conic is an ellipse (e > 0)", line_width=12).to_edge(UP, buff=1)
        # theorem_ellipse[7:9].set_color(YELLOW_D)

        # degenerate conditions
        title_parabola_degeneration = Text("Sufficient condition for a 3R serial robot to be quaternary").to_edge(UP, buff=0.5)
        equation_parabola_degeneration = Tex(r"\det(\mathbf{N}) < 0 \\~\\\det(\mathbf{D}) = 0\\~\\c_x^2 + c_y^2 \leq 1", alignment=R"\centering").next_to(title_parabola_degeneration, DOWN).shift(DOWN)
        bracing_equation_parabola_degeneration = Brace(equation_parabola_degeneration, direction=RIGHT).shift(RIGHT * 0.2)

        description = Text("hyperbola that degenerates with the intersection point inside the circle", line_width=5).next_to(bracing_equation_parabola_degeneration)


        # Animations
        self.play(*[FadeIn(obj) for obj in [title_parabola_degeneration, equation_parabola_degeneration, bracing_equation_parabola_degeneration]])
        self.wait(3)
        self.play(FadeIn(description))
        self.wait(2)
        self.embed()