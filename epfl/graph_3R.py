import math

from functions.phd_functions.functions_epfl import *
from functions.phd_functions.robots_3r import *

from manimlib import *


class PlotSetup(ThreeDScene):
    small_plot_size = 4

    def construct(self):
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
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), xvalue=(-2, 2), yvalue=(-2, 2),
                                                  label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4),
                                                 label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2))
        rval = 2
        conic_plot = ImplicitFunction(lambda c3,
                                             s3: 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625,
                                      color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval))
        circle = Circle(radius=2 / rval, stroke_color=PURPLE_D)
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
        text_conic_space = TexText(r"$c_3s_3$ plane \\$c_3 \gets \cos\theta_3, s_3 \gets \sin\theta_3$", font_size=36).next_to(plot_conic_space, DOWN * 3)
        text_work_space = TexText(r"work space projection \\on $\rho$-$z$ plane \\($\rho = \sqrt{x^2 + y^2}$)", font_size=36).next_to(plot_work_space, DOWN * 3)

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
        for obj in [plot_joint_space, plot_conic_space, plot_work_space, box_js, box_cs, box_ws]:
            self.add(obj.fix_in_frame())

        for obj in [text_joint_space, text_conic_space, text_work_space]:
            self.play(FadeIn(obj.fix_in_frame()))

        for obj in [critical_points.match_plot(plot_joint_space), critical_value, circle, conic_plot]:
            self.play(ShowCreation(obj.fix_in_frame()))
            self.wait()

        trace_joint_dot = TracedPath(joint_dot.get_center, stroke_color=GOLD_E).fix_in_frame()
        trace_work_dot = TracedPath(work_dot.get_center, stroke_color=GOLD_E).fix_in_frame()
        self.add(trace_joint_dot)
        self.add(trace_work_dot)

        self.play(FadeIn(start_position.fix_in_frame()))
        self.wait()
        dot_wkspc_list = [start_position.copy().fix_in_frame() for _ in iks]
        dot_jspc_list = [Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(sol[1], sol[2])).fix_in_frame() for sol
                         in iks]
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip(dot_wkspc_list, dot_jspc_list)], run_time=3)
        conic_intersections = Group()
        conic_intersections_prev = Group()
        conic_track = Circle(radius=0.1, color=PURPLE_D).fix_in_frame()
        track_point = [0, 0]
        total_iterations = 100
        # self.embed()

        for citer in range(total_iterations + 1):
            if len(vector_theta_list) == 0:
                vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, total_iterations))

            theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, total_iterations)
            vector_theta_list.append(theta_list)

            ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
            rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
            z = ee[2]
            R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

            new_conic_plot = ImplicitFunction(lambda c3,
                                                     s3: 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625,
                                              color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval))
            conic_sol_set = self.get_conic_solutions(R=R, z=z)
            # print(conic_sol_set)
            conic_intersections = []
            for sol in conic_sol_set:
                if abs(math.atan2(sol[1], sol[0]) - theta_list[1]) < 0.05:
                    track_point = sol
                    conic_intersections.append(Dot(fill_color=RED_D).fix_in_frame().move_to(np.array([sol[0], sol[1], 0])))
                else:
                    conic_intersections.append(Dot().fix_in_frame().move_to(np.array([sol[0], sol[1], 0])))

            objects_fadeout = [conic_plot.fix_in_frame()]
            objects_fadein = [new_conic_plot.fix_in_frame()]
            # conic_track.fix_in_frame().move_to(np.array([track_point[0], track_point[1], 0]))
            # self.play(*[FadeOut(conic_intersections_prev), FadeIn(conic_intersections)], run_time=0.01)

            self.play(
                *[Transform(item.fix_in_frame(), item.move_to(plane.c2p(p[0], p[1])).fix_in_frame(), run_time=0.1) for
                  item, plane, p in
                  zip([joint_dot, work_dot], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])
            # self.play(*[FadeOut(obj) for obj in objects_fadeout], *[FadeIn(obj) for obj in objects_fadein], run_time=0.05)
            # self.play(*[FadeIn(obj) for obj in objects_fadein], run_time=0.05)
            self.remove(*objects_fadeout, *conic_intersections_prev)
            self.add(*objects_fadein, *conic_intersections)
            conic_plot = new_conic_plot.fix_in_frame()
            conic_intersections_prev = conic_intersections

        self.wait()
        self.embed()
        self.play(*[FadeOut(obj) for obj in conic_intersections])

        # 3D animations
        frame = self.camera.frame
        frame.set_euler_angles(theta=-8.56932113e-01, phi=1.39950825e+00, gamma=0)
        frame.scale(1.2)
        frame.move_to(np.array([0.43, 0.87, 1.2]))

        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
               [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
               [1.72565092649699, -0.352336624163169, -2.014417810173952],
               [-1.44839814046463, -3.00000000000000, -0.49999999999999994]]

        self.play(*[obj.animate.shift(UP * 8) for obj in [plot_conic_space, box_cs, conic_plot, circle]])
        self.play(FadeOut(text_conic_space))
        self.wait()
        rob_ins = get_robot_instance(theta_list=iks[3], robot_type="philippe", offset=0, show_frame=False)
        rob_ins_faded = get_robot_instance(theta_list=iks[3], robot_type="philippe", offset=0, opacity=0.25,
                                           show_frame=False)

        # Animations
        self.play(*[FadeIn(obj) for obj in [rob_ins, rob_ins_faded]])
        self.wait()
        self.remove(trace_joint_dot, trace_work_dot)

        # reintroducing the traced path
        joint_dot2 = Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(first_theta_list[0], first_theta_list[1]))
        work_dot2 = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(first_rho, first_z))
        trace_joint_dot2 = TracedPath(joint_dot2.get_center, stroke_color=GOLD_E).fix_in_frame()
        trace_work_dot2 = TracedPath(work_dot2.get_center, stroke_color=GOLD_E).fix_in_frame()
        self.add(trace_joint_dot2)
        self.add(trace_work_dot2)

        ee_trace_2d = Group()
        ee_point, ee_R = get_frame_matrix(theta_list=iks[3], coord_num=3, offset=0.48, robot_type="philippe")
        ee_point_vec = [ee_point]

        for citer in range(51):
            if len(vector_theta_list) == 0:
                vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50))

            theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, 50)
            theta1_inter = -1.44839814046463 + (0.716267677272732 + 1.44839814046463) * citer / 50
            theta_3element = [theta1_inter]
            theta_3element.extend(theta_list)
            vector_theta_list.append(theta_list)

            ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
            rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
            z = ee[2]
            R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

            self.play(Transform(rob_ins, get_robot_instance(theta_list=theta_3element, robot_type="philippe", offset=0,
                                                            show_frame=False)), run_time=0.1)
            ee_point, ee_R = get_frame_matrix(theta_list=theta_3element, coord_num=3, offset=0.48,
                                              robot_type="philippe")
            ee_point_vec.append(ee_point)
            ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=GOLD_D))
            self.add(ee_trace_2d)

            self.play(
                *[Transform(item.fix_in_frame(), item.move_to(plane.c2p(p[0], p[1])).fix_in_frame(), run_time=0.1) for
                  item, plane, p in
                  zip([joint_dot2, work_dot2], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])

        self.wait(2)

        self.embed()

    def get_conic_solutions(self, R, z):
        c3, s3 = var('c3, s3')
        R = R
        z = z
        conic_eqn = 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625
        circle_eqn = c3 ** 2 + s3 ** 2 - 1

        return real_solutions(solve([conic_eqn, circle_eqn], [c3, s3]))


class CuspidalConfig(Scene):
    small_plot_size = 4

    def construct(self):
        frame = self.camera.frame
        frame.scale(1)

        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5, buff=2)
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4),
                                                 label_list=[r"""$z$""", r"""$\rho$"""], label_position="center", buff=2)

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2))
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

        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
               [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
               [1.72565092649699, -0.352336624163169, -2.014417810173952],
               [-1.44839814046463, -3.00000000000000, -0.49999999999999994]]

        A = TexText("A").move_to(plot_joint_space.c2p(-2.9, 0))
        B_preA = TexText("A").move_to(plot_joint_space.c2p(-0.4, 2.8))
        B = TexText("B").move_to(plot_joint_space.c2p(-0.4, 2.8))
        start_position = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))

        for obj in [plot_joint_space, plot_work_space, box_js, box_ws, text_joint_space, text_work_space, critical_points.match_plot(plot_joint_space), critical_value]:
            self.add(obj.fix_in_frame())

        dot_wkspc_list = [start_position.copy().fix_in_frame() for _ in iks]
        dot_jspc_list = [Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(sol[1], sol[2])) for sol
                         in iks]
        self.add(work_dot)
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip(dot_wkspc_list, dot_jspc_list)], run_time=2)
        self.wait(5)
        self.play(FadeIn(A), FadeIn(B_preA))
        self.wait(8)
        self.play(Transform(B_preA, B))
        self.wait()

        list_new_theta = [-0.5, 1.5]
        # iks = get_ikin(theta_list=list_new_theta)
        iks = [[0.851172293618727, -0.499999999999999, 1.500000000000002], [1.61070246744738, -0.347895108725723, -0.8747742217556498]]

        ee = get_fkin(list_new_theta, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        for obj in dot_wkspc_list:
            self.FadeIt(obj)

        self.wait(2)
        self.FadeIt(work_dot)

        work_dot2 = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))
        dot_wkspc_list2 = [work_dot2.copy() for _ in iks]
        dot_jspc_list2 = [Dot(fill_color=PINK).move_to(plot_joint_space.c2p(sol[1], sol[2])) for sol in iks]

        self.add(work_dot2)
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip(dot_wkspc_list2, dot_jspc_list2)], run_time=2)
        self.wait()

        question = TexText(r"??", font_size=32).move_to(plot_joint_space.c2p(0, 1.5))
        question_side = TexText(r"A or B", font_size=32).next_to(question, RIGHT)
        C = TexText("C").next_to(question, RIGHT)
        self.wait(2)
        self.play(FadeIn(question))
        self.wait(3)

        ar1 = Arrow(plot_joint_space.c2p(list_new_theta[0], list_new_theta[1]), plot_joint_space.c2p(-0.74, 2.6), stroke_width=1)
        ar2 = Arrow(plot_joint_space.c2p(list_new_theta[0], list_new_theta[1]), plot_joint_space.c2p(-3, -0.5), stroke_width=1)

        self.play(*[ShowCreation(obj) for obj in [ar1, ar2]])
        self.wait()
        self.play(FadeIn(question_side))
        self.wait(10)

        pseudo_singularity = ImplicitFunction(get_ps(), color=RED_D, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space).shift(RIGHT * 0.05)
        self.play(FadeIn(pseudo_singularity))
        self.wait(7)
        self.play(Transform(question_side, C))
        self.wait()

        self.embed()
        # text for graphs
        text_joint_space = TexText("joint space", font_size=36).next_to(plot_joint_space, DOWN * 3)
        text_work_space = TexText(r"work space projection \\on $\rho$-$z$ plane \\($\rho = \sqrt{x^2 + y^2}$)", font_size=36).next_to(plot_work_space, DOWN * 3)

    def FadeIt(self, *in_obj):
        self.play(*[ReplacementTransform(k2, k2.copy().set_opacity(0.4)) for k2 in in_obj], run_time=0.05)


class ThreeR_static(ThreeDScene):
    small_plot_size = 4

    def construct(self):
        frame = self.camera.frame

        # frame.scale(1)

        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
               [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
               [1.72565092649699, -0.352336624163169, -2.014417810173952],
               [-1.44839814046463, -1.5, -1.57]]

        rob_ins = get_robot_instance(theta_list=iks[3], robot_type="orthogonal3r", offset=-1.5, show_frame=False, add_plane=True)
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
        self.play(FadeIn(singularity_comment))
        self.wait()
        self.play(FadeIn(torus_comment))


        self.embed()


class PlotSetup_EPFL(ThreeDScene):
    small_plot_size = 4

    def construct(self):
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

        trace_joint_dot = TracedPath(joint_dot.get_center, stroke_color=GOLD_E).fix_in_frame()
        trace_work_dot = TracedPath(work_dot.get_center, stroke_color=GOLD_E).fix_in_frame()
        self.add(trace_joint_dot)
        self.add(trace_work_dot)

        self.play(FadeIn(start_position.fix_in_frame()))
        self.wait()
        dot_wkspc_list = [start_position.copy().fix_in_frame() for _ in iks]
        dot_jspc_list = [Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(sol[1], sol[2])).fix_in_frame() for sol
                         in iks]
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip(dot_wkspc_list, dot_jspc_list)], run_time=3)


        # 3D animations
        # frame = self.camera.frame
        # frame.set_euler_angles(theta=-8.56932113e-01, phi=1.39950825e+00, gamma=0)
        # frame.scale(1.2)
        # frame.move_to(np.array([0.43, 0.87, 1.2]))
        #
        # iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872],
        #        [-0.0662252608042159, -2.75577548636259, 2.0992885041182454],
        #        [1.72565092649699, -0.352336624163169, -2.014417810173952],
        #        [-1.44839814046463, -3.00000000000000, -0.49999999999999994]]
        #
        # rob_ins = get_robot_instance(theta_list=iks[3], robot_type="philippe", offset=0, show_frame=False)
        # rob_ins_faded = get_robot_instance(theta_list=iks[3], robot_type="philippe", offset=0, opacity=0.25,
        #                                    show_frame=False)
        #
        # # Animations
        # self.play(*[FadeIn(obj) for obj in [rob_ins, rob_ins_faded]])
        # self.wait()
        # self.remove(trace_joint_dot, trace_work_dot)
        #
        # # reintroducing the traced path
        # joint_dot2 = Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(first_theta_list[0], first_theta_list[1]))
        # work_dot2 = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(first_rho, first_z))
        # trace_joint_dot2 = TracedPath(joint_dot2.get_center, stroke_color=GOLD_E).fix_in_frame()
        # trace_work_dot2 = TracedPath(work_dot2.get_center, stroke_color=GOLD_E).fix_in_frame()
        # self.add(trace_joint_dot2)
        # self.add(trace_work_dot2)
        #
        # ee_trace_2d = Group()
        # ee_point, ee_R = get_frame_matrix(theta_list=iks[3], coord_num=3, offset=0.48, robot_type="philippe")
        # ee_point_vec = [ee_point]
        #
        # for citer in range(51):
        #     if len(vector_theta_list) == 0:
        #         vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50))
        #
        #     theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, 50)
        #     theta1_inter = -1.44839814046463 + (0.716267677272732 + 1.44839814046463) * citer / 50
        #     theta_3element = [theta1_inter]
        #     theta_3element.extend(theta_list)
        #     vector_theta_list.append(theta_list)
        #
        #     ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        #     rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        #     z = ee[2]
        #     R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2
        #
        #     self.play(Transform(rob_ins, get_robot_instance(theta_list=theta_3element, robot_type="philippe", offset=0,
        #                                                     show_frame=False)), run_time=0.1)
        #     ee_point, ee_R = get_frame_matrix(theta_list=theta_3element, coord_num=3, offset=0.48,
        #                                       robot_type="philippe")
        #     ee_point_vec.append(ee_point)
        #     ee_trace_2d.add(Line3D(start=ee_point_vec[-2], end=ee_point_vec[-1], color=GOLD_D))
        #     self.add(ee_trace_2d)
        #
        #     self.play(
        #         *[Transform(item.fix_in_frame(), item.move_to(plane.c2p(p[0], p[1])).fix_in_frame(), run_time=0.1) for
        #           item, plane, p in
        #           zip([joint_dot2, work_dot2], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])
        #
        # self.wait(2)

        self.embed()

    def get_conic_solutions(self, R, z):
        c3, s3 = var('c3, s3')
        R = R
        z = z
        conic_eqn = 0.25 * R ** 2 - 3.0 * R * c3 - 1.5 * R * s3 - 4.125 * R + 6.75 * c3 ** 2 + 9.0 * c3 * s3 + 18.75 * c3 + 2.25 * s3 ** 2 - 1.83697019872103e-16 * s3 * z + 12.375 * s3 + 1.0 * z ** 2 - 1.22464679914735e-16 * z + 13.015625
        circle_eqn = c3 ** 2 + s3 ** 2 - 1

        return real_solutions(solve([conic_eqn, circle_eqn], [c3, s3]))