import numpy as np

from functions.phd_functions.robot_functions import *
from functions.phd_functions.maple_functions import *
from functions.phd_functions.robots_3r import *
from manimlib import *


class CoordinateSystemExample(Scene):
    def construct(self):
        axes = Axes(
            # x-axis ranges from -1 to 10, with a default step size of 1
            x_range=(-1, 10),
            # y-axis ranges from -2 to 2 with a step size of 0.5
            y_range=(-2, 2, 0.5),
            # The axes will be stretched to match the specified
            # height and width
            height=6,
            width=10,
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
            )
        )
        # Keyword arguments of add_coordinate_labels can be used to
        # configure the DecimalNumber mobjects which it creates and
        # adds to the axes
        axes.add_coordinate_labels(
            font_size=20,
            num_decimal_places=1,
        )
        self.add(axes)
        self.embed()


class PlotSetup_old(Scene):
    NumberPlane.default_axis_config = dict(
        stroke_color=BLACK,
        stroke_width=0.01,
        include_ticks=False,
        include_tip=False,
        line_to_number_buff=SMALL_BUFF,
        line_to_number_direction=DL)
    NumberPlane.default_x_axis_config = dict(stroke_width=2, stroke_color=WHITE, opacity=1)

    def construct(self):
        plot_joint_space, box_js = self.get_small_plot(LEFT)
        plot_conic_space, box_cs = self.get_small_plot()
        plot_work_space, box_ws = self.get_small_plot(RIGHT)

        self.add(plot_joint_space.add_coordinate_labels(x_values=(-1, 0, 1, 2, PI), y_values=(-3, 4)))
        critical_values = plot_work_space.get_graph(lambda x: 0.1 * x ** 2 - 3, color=RED, x_range=(-2, 2))
        critical_values2 = plot_work_space.get_graph(lambda x: 0.05 * x ** 2 - 3, color=GREEN, x_range=(-2, 2))

        self.add(plot_joint_space, plot_conic_space, plot_work_space, box_js, box_cs, box_ws)
        self.play(*[ShowCreation(ii) for ii in [critical_values, critical_values2]], run_time=5)
        self.embed()

    def get_small_plot(self, edge=None):
        plot_to_return = NumberPlane(
            background_line_style=dict(stroke_color=RED_D, stroke_width=0.5, stroke_opacity=0.5, ),
            faded_line_style=dict(stroke_color=GREY_BROWN, stroke_width=1, stroke_opacity=0),
            x_range=(-3.14, 3.14, 0.5),
            y_range=(-3.14, 3.14, 0.5),
        )
        plot_to_return.set_height(PlotSetup.small_plot_size)
        plot_to_return.set_width(PlotSetup.small_plot_size)
        if edge is not None:
            plot_to_return.to_edge(edge, buff=SMALL_BUFF)
        box = SurroundingRectangle(plot_to_return)

        return plot_to_return, box


class PlotSetup(Scene):

    def construct(self):
        theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50)
        ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]
        R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2

        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True)
        # plot_conic_space, box_cs = get_small_plot()
        # plot_det_space, box_ds = get_det_plot(value='pos')
        plot_conic_space, box_cs = get_small_plot(label=False)
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=False, xvalue=(0, 6), yvalue=(-4, 4))

        print(get_conic(robot_type="philippe", k_R=R, z=z))
        # self.embed()
        critical_values = plot_work_space.get_graph(lambda x: 0.1 * x ** 2 - 3, color=RED, x_range=(-2, 2))
        rval = 2

        conic_plot = ImplicitFunction(get_conic(k_R=R, z=z, robot_type="philippe"), color=GREEN, x_range=(-rval, rval),
                                      y_range=(-rval, rval))
        critical_points = ImplicitFunction(
            lambda t2, t3: 4.5 * sin(t3) * cos(t2) * cos(t3) + 6.0 * sin(t3) * cos(t2) + 2.25 * sin(t3) * cos(
                t3) + 3.0 * sin(t3) - 2.25 * cos(t2) * cos(t3) ** 2 - 3.0 * cos(t2) * cos(t3), color=BLUE,
            x_range=(-3.2, 3.2), y_range=(-3.2, 3.2))
        critical_value = ImplicitFunction(lambda x, y: 65536 * x ** 16 + 524288 * y ** 2 * x ** 14 + 1835008 * y ** 4 * x ** 12 + 3670016 * y ** 6 * x ** 10 + 4587520 * y ** 8 * x ** 8 + 3670016 * y ** 10 * x ** 6 + 1835008 * y ** 12 * x ** 4 + 524288 * y ** 14 * x ** 2 + 65536 * y ** 16 - 2228224 * x ** 14 - 16121856 * y ** 2 * x ** 12 - 49938432 * y ** 4 * x ** 10 - 85852160 * y ** 6 * x ** 8 - 88473600 * y ** 8 * x ** 6 - 54657024 * y ** 10 * x ** 4 - 18743296 * y ** 12 * x ** 2 - 2752512 * y ** 14 + 20692992 * x ** 12 + 133332992 * y ** 2 * x ** 10 + 369901568 * y ** 4 * x ** 8 + 560136192 * y ** 6 * x ** 6 + 483934208 * y ** 8 * x ** 4 + 224559104 * y ** 10 * x ** 2 + 43499520 * y ** 12 - 82698240 * x ** 10 - 98885632 * y ** 2 * x ** 8 - 253280256 * y ** 4 * x ** 6 - 842514432 * y ** 6 * x ** 4 - 907239424 * y ** 8 * x ** 2 - 301817856 * y ** 10 + 305178112 * x ** 8 - 2765285376 * y ** 2 * x ** 6 - 3004079104 * y ** 4 * x ** 4 + 480122880 * y ** 6 * x ** 2 + 749282816 * y ** 8 - 714739200 * x ** 6 + 7322492416 * y ** 2 * x ** 4 - 6911564288 * y ** 4 * x ** 2 + 160135680 * y ** 6 + 1117824960 * x ** 4 + 964516736 * y ** 2 * x ** 2 + 195134400 * y ** 4 - 1675600160 * x ** 2 + 99657312 * y ** 2 + 12271009,
                                          color=BLUE, x_range=(0, 6), y_range=(-4, 4))

        critical_value.stretch(2 / 3, 0).stretch(5.33 / 8, 1).move_to(
            plot_work_space.get_origin() + RIGHT * critical_value.get_width() / 2)
        circle = Circle(radius=2 / rval)
        self.add(circle)
        self.add(critical_value)

        self.add(Dot().move_to(plot_work_space.get_origin()))
        dots_workspace = Group()
        vector_theta_list = []

        joint_dot = Dot(fill_color=PURPLE).move_to(plot_joint_space.c2p(theta_list[0], theta_list[1]))
        work_dot = Dot(fill_color=RED_D).move_to(plot_work_space.c2p(rho, z))
        self.add(TracedPath(joint_dot.get_center, stroke_width=2, stroke_color=GOLD_E).fix_in_frame())
        self.add(TracedPath(work_dot.get_center, stroke_width=2, stroke_color=GOLD_E).fix_in_frame())
        self.add(plot_joint_space, plot_conic_space, plot_work_space, box_js, box_cs, box_ws)
        self.play(*[ShowCreation(ii) for ii in
                    [critical_points.match_plot(plot_joint_space), critical_value, conic_plot]], run_time=5)

        for citer in range(51):
            if len(vector_theta_list) == 0:
                vector_theta_list.append(get_interpolation([-3, -0.5], [-0.742, 2.628], 0, 50))
            theta_list = get_interpolation([-3, -0.5], [-0.742, 2.628], citer, 50)
            vector_theta_list.append(theta_list)
            ee = get_fkin(theta_list, [0, 1, 0], [1, 2, 1.5], [-PI / 2, PI / 2, 0])
            rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
            z = ee[2]
            R = ee[0] ** 2 + ee[1] ** 2 + ee[2] ** 2
            print(rho, z)
            new_conic_plot = ImplicitFunction(get_conic(k_R=R, z=z, robot_type="philippe"), color=GREEN,
                                              x_range=(-rval, rval), y_range=(-rval, rval))
            dots_workspace.add(Dot(color=BLUE, opacity=1).move_to(np.array([rho, z, 0])))
            self.play(*[Transform(item, item.move_to(plane.c2p(p[0], p[1])), run_time=0.05) for item, plane, p in
                        zip([joint_dot, work_dot], [plot_joint_space, plot_work_space], [theta_list, [rho, z]])])
            self.play(*[FadeOut(conic_plot), FadeIn(new_conic_plot)], run_time=0.05)
            conic_plot = new_conic_plot
        self.wait(2)
        self.embed()


class TorusTransform(ThreeDScene):

    def construct(self):
        a = Circle()
        self.add(a)
        self.embed()
        # axes = ThreeDAxes()
        # self.add(axes)
        #
        # torus = Torus(r1=3, r2=1)
        # sphere = Sphere()
        # # self.add(torus)
        #
        # surface_1 = ParametricSurface(lambda u, v: np.array([(3 + np.cos(u)) * np.cos(v), 3 * np.sin(v) * np.sin(u), np.sin(u)]), u_range=(0, TAU), v_range=(0, TAU))
        # self.add(surface_1)


class IcraCheck(ThreeDScene):

    def construct(self):
        frame = self.camera.frame
        frame.set_euler_angles(theta=0, phi=85 * DEGREES, gamma=0)
        frame.shift([0, 0, 3])
        frame.scale(1.5)

        # JACO robot
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
        point1_comp = [-3.1201, 0.7082, 1.4904, 2.62, -1.9637, -1.8817]
        point5_comp = [2.4730, 0.0943, 2.0281, -1.4916, -2.4244, 2.4362]

        theta_list = adapt_2_pi(point1_comp)
        theta_list2 = adapt_2_pi(point5_comp)

        det_plot = ImplicitFunction(lambda x, y: x ** 5 - y ** 2, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2))
        robot = get_robot_instance(d_list, theta_list, a_list, alpha_list, offset=0)
        plane = ParametricSurface(lambda u, v: np.array([u, v, 0]), u_range=(-1, 1), v_range=(-1, 1), color=GREY_E)
        self.add(plane, robot)
        total_iterations = 10
        for ii in range(total_iterations):
            interpolated_theta_list = get_interpolation(theta_list, theta_list2, ii, total_iterations)
            self.play(Transform(robot, get_robot_instance(d_list, interpolated_theta_list, a_list,
                                                          alpha_list, offset=0, show_frame=True)), run_time=0.05)

        # self.embed()
