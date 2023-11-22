import numpy as np
from functions.phd_functions.robot_functions import *
from functions.phd_functions.functions_epfl import *
from functions.phd_functions.robots_3r import *
from manimlib import *
from numpy import sin, cos, sqrt
import pandas as pd

class ReducedAspectTheorem(Scene):

    def construct(self) -> None:
        theorem1 = Text(r"Theorem 1: In an arbitrary generic 3R serial robot, the inverse kinematic solutions lie always in distinct reduced aspects.", line_width=14).to_edge(TOP).shift(UP)
        proof = Text(r"Proof:").next_to(theorem1, DOWN).align_to(theorem1, LEFT)
        theorem1[7].set_color(YELLOW_D)

        argument1 = Text("Assuming that the IKS are not separated by reduced aspects", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument2 = Text("Let's consider a robot related to an ellipse", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument3 = Text("The movement of the conic is limited within a reduced aspect by singularities and/or pseudo singularities", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument4 = Text("If multiple IKS lie in a reduced aspect, then we can change IKS without crossing a singularity (or pseudosingularity)", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument5 = Text("This suggests that there should exist a motion of the ellipse in the c3-s3 plane such that I can swap the points", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument6 = Text("This can be obtained only by rotating the conic", line_width=12).to_edge(BOTTOM, buff=0.1)
        argument7 = Text("But, the conic does not rotate!", line_width=12).to_edge(BOTTOM, buff=0.1)

        # conic explanation
        conic_equation = TexText(r"$A_{xx}\,c_3^2 + 2A_{xy}\,c_3s_3 + A_{yy}\,s_3^2 + 2B_x\,c_3 + 2B_y\,s_3 + C = 0$").to_edge(RIGHT)
        list_first_index = [0, 8, 16, 23, 29, 35]
        list_second_index = [3, 11, 19, 26, 32, 36]
        list_value = [6.75, 9, 2.2, 2.2, 4.1, -1.9]
        [conic_equation[start:end].set_color(YELLOW_D) for start, end in zip(list_first_index, list_second_index)]
        axx = conic_equation[0:3]
        axy = conic_equation[8:11]
        ayy = conic_equation[16:19]
        group_nature = Group(axx, axy, ayy)
        bracing_coefficients = Brace(group_nature).shift(DOWN * 0.6)

        only_dh = Text("These parameters are dependent only on D-H parameters and thus are constant for a given robot", font_size=32, line_width=6).next_to(bracing_coefficients, DOWN)

        matrix_nature = Tex(r"\begin{bmatrix} A_{xx} & A_{xy} \\ A_{xy} & A_{yy}\end{bmatrix}").to_edge(RIGHT, buff=0.5).shift(DOWN * 1.5)
        matrix_nature[1:len(matrix_nature)-1].set_color(YELLOW_D)
        bracing_matrix = Brace(matrix_nature).shift(DOWN * 0.2)
        matrix_caption = Text("constant for a given robot", font_size=28, line_width=3).next_to(bracing_matrix, DOWN).shift(LEFT * 0.25)
        # ellipse calculations
        major = 1.5
        e = np.sqrt(0.75)
        cen_x, cen_y = 0, 0
        circle = Circle(stroke_color=PURPLE_D)
        equation_ellipse = ImplicitFunction(lambda c3, s3: ((c3 - cen_x) ** 2 / ((major ** 2) * (1 - e ** 2))) + ((s3 - cen_y) ** 2 / major ** 2) - 1, x_range=(-3.2, 3.2), y_range=(-3.2, 3.2), color=BLUE_D)

        group_dots = Group()
        solutions = solve_conic_circle()
        for sol in solutions:
            group_dots.add(Dot().move_to([sol[0], sol[1], 0]))


        # Animations
        self.play(FadeIn(theorem1))
        self.wait(7)
        self.play(FadeIn(proof))
        self.wait()

        self.play(FadeIn(argument1))
        self.wait(3)
        argument1.shift(UP * 0.6)
        self.play(FadeIn(argument2))
        self.wait(1)
        self.play(*[ShowCreation(obj) for obj in [circle, equation_ellipse]])
        self.play(*[FadeIn(obj) for obj in [group_dots]])
        self.wait(4)
        self.play(FadeOut(argument1), FadeOut(argument2))
        self.play(FadeIn(argument3))
        self.wait(3)

        jj = [[0, 0.6, 0.1], [0.5, -0.6, -0.1], [0, 0.26, 0.05], [0.25, -0.26, -0.05], [-0.25, 0.01, 0.05]]
        ll = 0
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
                self.add(equation_ellipse_new, group_dots_new)
                self.wait(0.12)
                # self.play(*[FadeOut(obj) for obj in [equation_ellipse, group_dots]], run_time=0.05)
                # self.play(*[FadeIn(obj) for obj in [equation_ellipse_new, group_dots_new]], run_time=0.05)
                equation_ellipse = equation_ellipse_new
                group_dots = group_dots_new
            ll += 1

        for aa, bb in zip([argument3, argument4, argument5], [argument4, argument5, argument6]):
            self.play(FadeOut(aa))
            self.play(FadeIn(bb))
            self.wait(5)

        self.remove(group_dots)
        group_dots = Group()
        solutions = solve_conic_circle()
        jj = 0
        for sol in solutions:
            if jj == 1:
                group_dots.add(Dot(fill_color=YELLOW_D).move_to([sol[0], sol[1], 0]))
            else:
                group_dots.add(Dot().move_to([sol[0], sol[1], 0]))
            jj += 1

        self.play(FadeIn(group_dots))
        self.wait()
        self.play(*[Rotate(obj, angle=PI, axis=OUT) for obj in [equation_ellipse, group_dots]])

        self.wait()
        self.play(FadeOut(argument6), FadeIn(argument7))
        self.wait(2)

        plot_group = Group(circle, equation_ellipse, group_dots)
        self.play(plot_group.animate.shift(LEFT * 4))

        self.add(conic_equation)
        self.wait(5)
        # self.play(*[obj.animate.shift(DOWN * 0.5) for obj in [axx, axy, ayy]])
        self.play(group_nature.animate.shift(DOWN * 0.5))
        self.wait(2)
        self.play(FadeIn(bracing_coefficients))
        self.wait(2)
        self.play(FadeIn(only_dh))
        self.wait()
        self.play(FadeIn(only_dh[27:36].set_color(YELLOW_D)))
        self.wait(5)

        self.embed()
        self.play(Transform(axx.copy(), matrix_nature))
        self.wait()
        self.play(*[FadeIn(obj) for obj in [bracing_matrix, matrix_caption]])
        self.wait()
        self.play(ShowCreation(SurroundingRectangle(argument7)))
        self.wait()

        self.embed()

    def wriggle(self, obj):
        self.play(*[objs.animate.shift(UP * 0.1 + RIGHT * 0.1) for objs in obj])
