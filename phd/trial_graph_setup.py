from functions.phd_functions.robot_functions import *
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


def get_small_plot(edge=None, label=False):
    plot_to_return = Axes(
        x_range=(-3, 3),
        y_range=(-3, 3),
        y_axis_config=dict(stroke_width=0.001, opacity=0, include_ticks=False, stroke_color=BLACK),
        x_axis_config=dict(include_ticks=False),
    )
    height = PlotSetup.small_plot_size
    width = height
    plot_to_return.set_height(height)
    plot_to_return.set_width(PlotSetup.small_plot_size)
    shift_buff = 0.85
    if edge is not None:
        plot_to_return.to_edge(edge, buff=SMALL_BUFF * 2)
    box = SurroundingRectangle(plot_to_return, stroke_width=1, buff=0)
    top_right = TexText(r"""$(\pi, \pi)$""""").move_to(plot_to_return.get_center()).scale(0.6)
    bottom_left = TexText(r"""(-$\pi$, -$\pi$)""""").move_to(plot_to_return.get_center()).scale(0.6)
    top_right.shift(np.array([width * shift_buff / 2, height * 1.1 / 2, 0]))
    bottom_left.shift(np.array([-width * shift_buff / 2, -height * 1.1 / 2, 0]))
    second_element = Group(box, top_right, bottom_left) if label else box
    return plot_to_return, second_element


def get_det_plot(edge=None, value=None):
    if value is None:
        y_range = (-5, 5)
    elif value == 'pos':
        y_range = (-1, 10)
    else:
        y_range = (-10, 1)

    plot_to_return = Axes(
        x_range=(-1, 10),
        y_range=y_range,
        x_axis_config=dict(include_ticks=False),
    )
    plot_to_return.set_height(PlotSetup.small_plot_size)
    plot_to_return.set_width(PlotSetup.small_plot_size)
    if edge is not None:
        plot_to_return.to_edge(edge, buff=SMALL_BUFF * 2)
    box = SurroundingRectangle(plot_to_return, stroke_width=1, buff=0)

    return plot_to_return, box


class PlotSetup(Scene):
    small_plot_size = 4

    def construct(self):
        plot_joint_space, box_js = get_small_plot(edge=LEFT, label=True)
        # plot_conic_space, box_cs = get_small_plot()
        plot_det_space, box_ds = get_det_plot(value='pos')
        plot_work_space, box_ws = get_small_plot(edge=RIGHT, label=False)

        critical_values = plot_work_space.get_graph(lambda x: 0.1 * x ** 2 - 3, color=RED, x_range=(-2, 2))
        critical_values2 = plot_det_space.get_graph(lambda x: np.sin(x) + 5, color=GREEN, x_range=(0, 10))
        critical_points = ImplicitFunction(lambda x, y: x * y ** 2 - x ** 2 * y - 2, color=BLUE,
                                           x_range=(-3.2, 3.2), y_range=(-3.2, 3.2)).match_plot(plot_joint_space)
        # critical_points.set_width(plot_joint_space.get_width()).move_to(plot_joint_space.get_center())

        self.add(plot_joint_space, plot_det_space, plot_work_space, box_js, box_ds, box_ws)
        self.play(*[ShowCreation(ii) for ii in [critical_points, critical_values, critical_values2]], run_time=5)
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
