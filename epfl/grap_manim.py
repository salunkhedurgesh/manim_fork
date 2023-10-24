from functions.phd_functions.robot_functions import *
from manim import *


class Graph3R(Scene):

    def construct(self):
        plane1 = NumberPlane(x_range=[-3.2, 3.2], y_range=[-3.2, 3.2], x_length=6.4, y_length=6.4,
                             background_line_style={
                                 "stroke_color": TEAL,
                                 "stroke_width": 0.1,
                                 "stroke_opacity": 0.6
                             }, x_axis_config={
                "exclude_origin_tick": False
            })
        plane1.x_axis.exclude_origin_tick = False
        plane1.add_coordinates(range(-3, 4), range(-3, 4))
        plane1.coordinate_labels[1][0:3].shift(LEFT * 0.5)
        plane1.coordinate_labels[1][3:7].shift(LEFT * 0.35)
        plane1.y_axis.shift(LEFT * 3.2)
        plane1.x_axis.shift(DOWN * 3.2)
        curve = plane1.plot_implicit_curve(lambda t2, t3: -3 * (3 * np.cos(t3) + 4) * (
                np.sin(t3) * (2 + 4 * np.cos(t2)) - 2 * np.cos(t2) * np.cos(t3)))

        self.add(plane1, curve)