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
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True)
        plot_conic_space, box_cs = get_small_plot(label=False, yconfig=dict(include_ticks=True),
                                                  xconfig=dict(include_ticks=True))
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=False, xvalue=(0, 6), yvalue=(-4, 4))

        # graphs
        critical_points = ImplicitFunction(get_det(robot_type="philippe"), color=BLUE_D, x_range=(-3.2, 3.2),
                                           y_range=(-3.2, 3.2))
        rval = 2
        conic_plot = ImplicitFunction(get_conic(k_R=R, z=z, robot_type="philippe"), color=BLUE_D, x_range=(-rval, rval),
                                      y_range=(-rval, rval))
        circle = Circle(radius=2 / rval, stroke_color=PURPLE_D)
        critical_value = ImplicitFunction(get_critv(), color=BLUE_D, x_range=(0, 6), y_range=(-4, 4))
        critical_value.stretch(2 / 3, 0).stretch(4 / 8, 1).move_to(
            plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)

        vector_theta_list = []
        joint_dot = Dot(fill_color=PINK).move_to(plot_joint_space.c2p(theta_list[0], theta_list[1]))
        work_dot = Dot(fill_color=DARK_BROWN).move_to(plot_work_space.c2p(rho, z))

        # Animations
        self.add(plot_joint_space, plot_conic_space, plot_work_space, box_js, box_cs, box_ws)
        self.play(*[ShowCreation(ii) for ii in
                    [critical_points.match_plot(plot_joint_space), critical_value, circle, conic_plot]], run_time=5)
        self.add(TracedPath(joint_dot.get_center, stroke_width=2, stroke_color=GOLD_E).fix_in_frame())
        self.add(TracedPath(work_dot.get_center, stroke_width=2, stroke_color=GOLD_E).fix_in_frame())

        for citer in range(51):
            if len(vector_theta_list) == 0:
                vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50))

            theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, 50)
            vector_theta_list.append(theta_list)

            ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
            rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
            z = ee[2]
            R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

            new_conic_plot = ImplicitFunction(get_conic(k_R=R, z=z, robot_type="philippe"), color=GREEN,
                                              x_range=(-rval, rval), y_range=(-rval, rval))

            self.play(*[Transform(item, item.move_to(plane.c2p(p[0], p[1])), run_time=0.05) for item, plane, p in
                        zip([joint_dot, work_dot], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])
            self.play(*[FadeOut(conic_plot), FadeIn(new_conic_plot)], run_time=0.05)
            conic_plot = new_conic_plot

        self.wait(2)
        self.embed()
