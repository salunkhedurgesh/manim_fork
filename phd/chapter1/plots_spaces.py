from functions.phd_functions.robot_functions import *
from functions.phd_functions.maple_functions import *
from functions.phd_functions.robots_3r import *
from manimlib import *


class PlotSetup(Scene):
    small_plot_size = 4

    def construct(self):
        # data for initial start
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        # planes for the plots
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True, box_opacity=0.5)
        plot_conic_space, box_cs = get_small_plot(label=True, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True), label_list=[r"""$c_3$""", r"""$s_3$"""], label_position="center")
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=True, xvalue=(0, 6), yvalue=(-4, 4), label_list=[r"""$z$""", r"""$\rho$"""], label_position="center")

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2))
        rval = 2
        conic_plot = ImplicitFunction(lambda c3, s3: 0.25*R**2 - 3.0*R*c3 - 1.5*R*s3 - 4.125*R + 6.75*c3**2 + 9.0*c3*s3 + 18.75*c3 + 2.25*s3**2 - 1.83697019872103e-16*s3*z + 12.375*s3 + 1.0*z**2 - 1.22464679914735e-16*z + 13.015625,
                                             color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval))
        circle = Circle(radius=2 / rval, stroke_color=PURPLE_D)
        critical_value = ImplicitFunction(get_critv(), color=BLUE_D, x_range=(0, 6), y_range=(-4, 4))
        critical_value.stretch(2 / 3, 0).stretch(4 / 8, 1).move_to(
            plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)

        vector_theta_list = []
        joint_dot = Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(theta_list[0], theta_list[1]))
        work_dot = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))

        # iks = get_ikin(theta_list=[-3, -0.5])
        # print(iks)
        iks = [[0.716267677272732, -0.742115942207296, 2.6294267416438872], [-0.0662252608042159, -2.75577548636259, 2.0992885041182454], [1.72565092649699, -0.352336624163169, -2.014417810173952], [-1.44839814046463, -3.00000000000000, -0.49999999999999994]]

        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]

        start_position = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))

        # Animations
        self.add(plot_joint_space, plot_conic_space, plot_work_space, box_js, box_cs, box_ws)
        for obj in [critical_points.match_plot(plot_joint_space), critical_value, circle, conic_plot]:
            self.play(ShowCreation(obj))
            self.wait()

        self.add(TracedPath(joint_dot.get_center, stroke_color=GOLD_E))
        self.add(TracedPath(work_dot.get_center, stroke_color=GOLD_E))

        self.play(FadeIn(start_position))
        self.wait()
        dot_wkspc_list = [start_position.copy() for _ in iks]
        dot_jspc_list = [Dot(fill_color=PURPLE_D).move_to(plot_joint_space.c2p(sol[1], sol[2])) for sol in iks]
        self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip(dot_wkspc_list, dot_jspc_list)], run_time=3)

        for citer in range(51):
            if len(vector_theta_list) == 0:
                vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50))

            theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, 50)
            vector_theta_list.append(theta_list)

            ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
            rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
            z = ee[2]
            R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

            new_conic_plot = ImplicitFunction(lambda c3, s3: 0.25*R**2 - 3.0*R*c3 - 1.5*R*s3 - 4.125*R + 6.75*c3**2 + 9.0*c3*s3 + 18.75*c3 + 2.25*s3**2 - 1.83697019872103e-16*s3*z + 12.375*s3 + 1.0*z**2 - 1.22464679914735e-16*z + 13.015625,
                                             color=BLUE_D, x_range=(-rval, rval), y_range=(-rval, rval))

            self.play(*[Transform(item, item.move_to(plane.c2p(p[0], p[1])), run_time=0.1) for item, plane, p in
                        zip([joint_dot, work_dot], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])
            self.play(*[FadeOut(conic_plot), FadeIn(new_conic_plot)], run_time=0.05)
            conic_plot = new_conic_plot

        self.wait(2)
        self.embed()
