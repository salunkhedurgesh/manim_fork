from functions.phd_functions.robot_functions import *
from functions.phd_functions.maple_functions import *
from functions.phd_functions.robots_3r import *
from manimlib import *


def diamond(year=1988):
    position = [year - 1988, 0, 0]
    return Square(side_length=0.2, fill_color=YELLOW_D, stroke_color=YELLOW_D, fill_opacity=1).rotate(PI / 4).move_to(position)

def diamond_dot(year=1988):
    position = [year - 1988, 0, 0]
    return Dot(radius=0.1, fill_color=GREEN_D, stroke_color=BLACK, fill_opacity=1).move_to(position)


def cite(obj, cite_position=0, citation=""):
    ref = TexText(r"$" + str(Timeline.cites) + "$", color=YELLOW_D, font_size=36).move_to(obj.get_corner(UR) + [0.2, 0, 0])
    citation_a = TexText(r"\begin{minipage}{18cm}" + citation + r"\end{minipage}", font_size=24).to_edge(BOTTOM, buff=0).fix_in_frame()
    citation_b = TexText(r"" + str(Timeline.cites) + "", color=YELLOW_D, font_size=24).move_to(citation_a.get_corner(UL) + LEFT * 0.2).fix_in_frame()
    citation = Group(citation_a, citation_b)

    Timeline.cites += 1

    return ref, citation


class Timeline(Scene):
    cites = 1

    def construct(self) -> None:
        frame = self.camera.frame
        # pre 1988
        dotted_line = DashedLine([-9, 0, 0], ORIGIN)
        diamond_1988 = diamond(year=1988)
        # 1988 : Vincenzo Parenti Castelli
        year_1988 = TexText(r"1988").next_to(diamond_1988, DOWN)
        description_pre1988 = Text("Before 1988: Inverse kinematic solutions (IKS) are separated by locus of critical points", font_size=36, line_width=14).to_edge(UP, buff=0.5)
        description_1988 = Text("Two counterexamples are presented", font_size=36).next_to(year_1988, DOWN, buff=1).fix_in_frame()
        bbl_1988 = "Parenti-Castelli. V, Innocenti. C, Position analysis of robot manipulators: Regions and subregions, In Proceedings of Symposium on Advances in Robot Kinematics"
        ref_1988, citation_1988 = cite(description_1988, citation=bbl_1988)
        # 1991 : Burdick
        line_burdick = my_line(1988, 1991)
        diamond_1991 = diamond(year=1991)
        year_1991 = TexText(r"1991").next_to(diamond_1991, DOWN).shift(LEFT * 0.1)
        description_1991 = Text("Burdick presents nonsingular change of solutions in 3R robots", font_size=36, line_width=12).next_to(year_1991, DOWN, buff=1)
        bbl_1991 = "J. W. Burdick, A classification of 3R regional manipulator singularities and geometries, Proceedings. 1991 IEEE International Conference on Robotics and Automation, Sacramento, CA, USA, 1991"
        ref_1991, citation_1991 = cite(description_1991, cite_position=1991, citation=bbl_1991)
        # 1992 : new formalism
        line_1992 = my_line(1991, 1992)
        diamond_1992 = diamond(1992)
        year_1992 = TexText(r"1992").next_to(diamond_1992, DOWN).shift(RIGHT * 0.1)
        description_1992 = Text("Wenger presents new formalism to analyze the joint space and the workspace of non-redundant robots", font_size=36, line_width=12).next_to(year_1992, DOWN, buff=1)
        bbl_1992 = "P. Wenger, A new general formalism for the kinematic analysis of all nonredundant manipulators, Proceedings 1992 IEEE International Conference on Robotics and Automation, Nice, France, 1992"
        ref_1992, citation_1992 = cite(description_1992, cite_position=1992, citation=bbl_1992)
        # 1996 : Definition of cuspidal robots
        line_1996 = my_line(1992, 1996)
        diamond_1996 = diamond(1996)
        year_1996 = TexText(r"1996").next_to(diamond_1996, DOWN)
        description_1996 = Text("The word \'cuspidal\'  is defined for robots performing nonsingular change of solutions", font_size=36, line_width=12).next_to(year_1996, DOWN, buff=1)
        bbl_1996 = "P. Wenger and J. El Omri, Changing posture for cuspidal robot manipulators, Proceedings of IEEE International Conference on Robotics and Automation, Minneapolis, MN, USA, 1996"
        ref_1996, citation_1996 = cite(description_1996, cite_position=1996, citation=bbl_1996)
        # 1998 : homotopy classification
        line_1998 = my_line(1996, 1998)
        diamond_1998 = diamond(1998)
        year_1998 = TexText(r"1998").next_to(diamond_1998, DOWN)
        description_1998 = Text("Homotopy based classification of 3R robots is presented", font_size=36, line_width=12).next_to(year_1998, DOWN, buff=1)
        bbl_1998 = "Wenger, P. (June 1, 1998). Classification of 3R Positioning Manipulators. ASME. J. Mech. Des. June 1998"
        ref_1998, citation_1998 = cite(description_1998, cite_position=1998, citation=bbl_1998)
        # 2004 : Maher Baili
        line_2004 = my_line(1998, 2004)
        diamond_2004 = diamond(2004)
        year_2004 = TexText(r"2004").next_to(diamond_2004, DOWN)
        description_2004 = Text("Baili et.al presents classification based on number of cusps in the workspace of 3R robots", font_size=36, line_width=12).next_to(year_2004, DOWN, buff=1)
        bbl_2004 = "M. Baili, P. Wenger and D. Chablat, A classification of 3R orthogonal manipulators by the topology of their workspace, IEEE International Conference on Robotics and Automation, 2004. Proceedings. ICRA '04. 2004, New Orleans, LA, USA, 2004"
        ref_2004, citation_2004 = cite(description_2004, cite_position=2004, citation=bbl_2004)
        # 2004 : Solen Corvez
        diamond_2004_corvez = diamond(2004).shift(UP * 0.4)
        description_2004_corvez = Text("Corvez et.al shows that a cusp in the worksapce is a sufficient condition for a 3R robot to be cuspidal. Computer algebra tools for classification.", font_size=36, line_width=12).next_to(year_2004, DOWN, buff=1)
        bbl_2004_corvez = "Corvez, S., Rouillier, F. (2004). Using Computer Algebra Tools to Classify Serial Manipulators. In: Winkler, F. (eds) Automated Deduction in Geometry. ADG 2002. Lecture Notes in Computer Science(), vol 2930. Springer"
        ref_2004_corvez, citation_2004_corvez = cite(description_2004_corvez, cite_position=2004, citation=bbl_2004_corvez)
        # 2004 : Philippe
        diamond_2004_wenger = diamond(2004).shift(UP * 0.8)
        description_2004_wenger = Text("Wenger presents uniqueness domains and the consequences on path planning in a 3R cuspidal robot", font_size=36, line_width=12).next_to(year_2004, DOWN, buff=1)
        bbl_2004_wenger = "P. Wenger, Uniqueness domains and regions of feasible paths for cuspidal manipulators, in IEEE Transactions on Robotics, vol. 20, no. 4, pp. 745-750, Aug. 2004"
        ref_2004_wenger, citation_2004_wenger = cite(description_2004_wenger, cite_position=2004, citation=bbl_2004_wenger)
        # 2006 : Mazen Zein
        line_2006 = my_line(2004, 2006)
        diamond_2006 = diamond(2006)
        year_2006 = TexText(r"2006").next_to(diamond_2006, DOWN)
        description_2006 = Text("Zein et.al presents exhaustive study of workspace topologies of orthogonal 3R robots", font_size=36, line_width=12).next_to(year_2006, DOWN, buff=1)
        bbl_2006 = "Mazen Zein, Philippe Wenger, Damien Chablat, An exhaustive study of the workspace topologies of all 3R orthogonal manipulators with geometric simplifications, Mechanism and Machine Theory, Volume 41, Issue 8, 2006"
        ref_2006, citation_2006 = cite(description_2006, cite_position=2006, citation=bbl_2006)
        # 2008 : Paganelli
        line_2008 = my_line(2006, 2008)
        diamond_2008 = diamond(2008)
        year_2008 = TexText(r"2008").next_to(diamond_2008, DOWN)
        description_2008 = Text("Paganelli presents further results based on homotopy classification of singularities in joint space of generic 3R robots.", font_size=36, line_width=12).next_to(year_2008, DOWN, buff=1)
        bbl_2008 = "Paganelli, D., 2008, “Topological Analysis of Singularity Loci for Serial and Parallel. Manipulators,”Phd. Thesis, University of Bologna."
        ref_2008, citation_2008 = cite(description_2008, cite_position=2008, citation=bbl_2008)
        # 2011 : Thomas
        line_2011 = my_line(2008, 2011)
        diamond_2011 = diamond(2011)
        year_2011 = TexText(r"2011").next_to(diamond_2011, DOWN)
        description_2011 = Text("Thomas and Wenger presents topological characterization of singularity loci by using catastrophe theory", font_size=36, line_width=12).next_to(year_2011, DOWN, buff=1)
        bbl_2011 = "F. Thomas and P. Wenger, On the topological characterization of robot singularity loci. a catastrophe-theoretic approach, 2011 IEEE International Conference on Robotics and Automation, Shanghai, China, 2011"
        ref_2011, citation_2011 = cite(description_2011, cite_position=2011, citation=bbl_2011)
        # 2016 : Thomas
        line_2016 = my_line(2011, 2016)
        diamond_2016 = diamond(2016)
        year_2016 = TexText(r"2016").next_to(diamond_2016, DOWN)
        description_2016 = Text("Thomas presents singularity analysis by using distance geometry", font_size=36, line_width=12).next_to(year_2016, DOWN, buff=1)
        bbl_2016 = "Thomas, F. (August 18, 2015). A Distance Geometry Approach to the Singularity Analysis of 3R Robots. ASME. J. Mechanisms Robotics. February 2016"
        ref_2016, citation_2016 = cite(description_2016, cite_position=2016, citation=bbl_2016)
        # 2020 : Capco
        line_2020 = my_line(2016, 2020)
        diamond_2020 = diamond(2020)
        year_2020 = TexText(r"2020").next_to(diamond_2020, DOWN)
        description_2020 = Text("Capco et. al show that UR5 is a noncuspidal robot using computer algebra tools", font_size=36, line_width=12).next_to(year_2020, DOWN, buff=1)
        bbl_2020 = "Jose Capco, Mohab Safey El Din, and Josef Schicho. 2020. Robots, computer algebra and eight connected components. In Proceedings of the 45th International Symposium on Symbolic and Algebraic Computation. ACM, New York, NY, USA"
        ref_2020, citation_2020 = cite(description_2020, cite_position=2020, citation=bbl_2020)
        # 2023 : Salunkhe
        line_2023 = my_line(2020, 2023, dashed=True)
        diamond_2023 = diamond_dot(2023)
        year_2023 = TexText(r"2023").next_to(diamond_2023, DOWN)
        description_2023 = Text("Cuspidal robots: theoretical study, classification and applications to commercial robots", font_size=36, line_width=12).next_to(year_2023, DOWN, buff=1)

        # Robot 2R code
        link_lengths = [2, 3]
        offset = np.array([0, -1, 0])
        theta1, theta2 = 55 * DEGREES, -60 * DEGREES
        l1, l2 = link_lengths
        point1 = np.array([0, 0, 0]) + offset
        point2 = point1 + np.array([l1 * cos(theta1), l1 * sin(theta1), 0])
        ee_point = point2 + np.array([l2 * cos(theta1 + theta2), l2 * sin(theta1 + theta2), 0])
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)
        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point2)
        ee = Dot().move_to(ee_point)

        link1 = Line(point1, point2)
        link2 = Line(point2, ee_point)
        mirror_ee_point = ee_point - offset
        mirroring_line = Line(-3 * mirror_ee_point, 3 * mirror_ee_point, stroke_width=0.5, stroke_color=YELLOW).shift(offset)
        axis_config = dict(include_ticks=True, stroke_color=WHITE)
        plot_joint_space, labels = get_small_plot(edge=LEFT, label=True, xconfig=axis_config, yconfig=axis_config, only_labels=True)
        x_label2 = TexText(r"""$\theta_1$""", font_size=36).next_to(plot_joint_space.x_axis, DOWN * 0.5).shift(RIGHT * 1.5)
        y_label2 = TexText(r"""$\theta_2$""", font_size=36).next_to(plot_joint_space.y_axis).rotate(np.pi / 2).shift(UP * 1.7 + LEFT * 0.8)
        singularity_curves = Group()
        for val in [-PI, 0, PI]:
            singularity_curves.add(plot_joint_space.get_graph(lambda x: val).set_color(color=BLUE_D))

        dot_js = Dot(fill_color=PURPLE).move_to(plot_joint_space.c2p(theta1, theta2))

        # text objects
        elbow_up = TexText("Elbow Up configuration", font_size=42).move_to(np.array([2, 2, 0]))
        elbow_down = TexText("Elbow Down configuration", font_size=42).move_to(np.array([2, -2.5, 0]))
        singularity = TexText(
            r"""\begin{minipage}{5 cm}\centering A singular configuration is met while changing IKS \end{minipage}""",
            font_size=36).to_edge(RIGHT, buff=LARGE_BUFF).add_background_rectangle(color=BLACK, opacity=0.8).fix_in_frame()
        singularity_curves_text = TexText("singularity curves").to_edge(LEFT, buff=LARGE_BUFF).shift(UP).add_background_rectangle(color=BLACK, opacity=1)

        # Animations
        # Before 1988
        self.play(ShowCreation(dotted_line))
        self.wait(0.5)
        self.play(FadeIn(diamond_1988))
        self.wait(0.5)
        self.play(FadeIn(description_pre1988))
        self.wait(2)

        # Robot 2R animation
        # Animations

        # self.add(get_background())
        self.play(FadeOut(dotted_line), FadeOut(diamond_1988))
        self.add(link1, link2, joint1, joint2, ee)
        self.add(plot_joint_space, labels, x_label2, y_label2, dot_js, dot_js.copy())
        self.wait()
        self.play(FadeIn(singularity_curves))
        self.wait()
        self.FadeInFadeOut(singularity_curves_text)

        theta1_mirror, theta2_mirror = self.draw_mirror(point1, point2, ee_point, l1, _add_obj=False)
        self.wait(2)
        self.play(ShowCreation(mirroring_line), run_time=3)
        self.wait()
        self.draw_mirror(point1, point2, ee_point, l1, _add_obj=True)
        self.wait()
        self.play(FadeIn(elbow_up))
        self.wait()
        self.play(FadeIn(elbow_down))
        self.wait()
        self.play(FadeOut(plot_joint_space.y_axis))

        pause = False
        last_sign = theta2
        self.add(TracedPath(dot_js.get_center, stroke_width=3, stroke_color=GOLD_A))
        for ii, jj in zip(np.linspace(theta1, theta1_mirror, 40), np.linspace(theta2, theta2_mirror, 40)):
            new_point2 = point1 + np.array([l1 * cos(ii), l1 * sin(ii), 0])
            new_ee_point = new_point2 + np.array([l2 * cos(ii + jj), l2 * sin(ii + jj), 0])
            if last_sign * jj < 0:
                stroke_color = RED_D
                pause = True
            else:
                stroke_color = WHITE
            last_sign = jj
            temp_link1 = Line(point1, new_point2, stroke_color=stroke_color)
            temp_link2 = Line(new_point2, new_ee_point, stroke_color=stroke_color)
            temp_joint2 = joint2.copy().move_to(new_point2)
            temp_ee = ee.copy().move_to(new_ee_point)
            temp_dotjs = dot_js.copy().move_to(plot_joint_space.c2p(ii, jj))
            if pause:
                temp_ee.set_fill(color=RED_D, opacity=1)
            self.play(*[Transform(obj1, obj2) for obj1, obj2 in zip([link1, link2, joint2, ee, dot_js], [temp_link1, temp_link2, temp_joint2, temp_ee, temp_dotjs])], run_time=0.2)
            if pause:
                self.play(Transform(dot_js, dot_js.scale(2)))
                self.play(Transform(dot_js, dot_js.scale(0.5)))
                self.FadeInFadeOut(singularity)
                pause = False

        self.wait(2)

        self.clear()

        self.add(dotted_line, diamond_1988, description_pre1988)

        # Parenti-Castelli eureka
        self.play(FadeIn(year_1988))
        self.play(*[FadeIn(obj) for obj in [description_1988, ref_1988, citation_1988]])
        self.wait(8)
        self.play(*[FadeOut(obj) for obj in [description_1988, citation_1988, ref_1988, description_pre1988]])
        # Burdick
        self.play(frame.animate.shift(RIGHT * 3), ShowCreation(line_burdick))
        self.add(diamond_1988)
        self.play(FadeIn(year_1991), FadeIn(diamond_1991))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_1991, ref_1991, citation_1991]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_1991, citation_1991, ref_1991]])
        # Philippe 1992
        self.play(frame.animate.shift(RIGHT), ShowCreation(line_1992))
        self.add(diamond_1991)
        self.play(FadeIn(year_1992), FadeIn(diamond_1992))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_1992, ref_1992, citation_1992]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_1992, citation_1992, ref_1992]])
        # Philippe 1996
        self.play(frame.animate.shift(RIGHT * 4), ShowCreation(line_1996))
        self.add(diamond_1992)
        self.play(FadeIn(year_1996), FadeIn(diamond_1996))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_1996, ref_1996, citation_1996]])
        self.wait(10)
        self.play(*[FadeOut(obj) for obj in [description_1996, ref_1996, citation_1996]])
        # Philippe 1998 : Homotopy based classification
        self.play(frame.animate.shift(RIGHT * 2), ShowCreation(line_1998))
        self.add(diamond_1996)
        self.play(FadeIn(year_1998), FadeIn(diamond_1998))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_1998, ref_1998, citation_1998]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_1998, ref_1998, citation_1998]])
        # 2004 : Baili
        self.play(frame.animate.shift(RIGHT * 6), ShowCreation(line_2004))
        self.add(diamond_1998)
        self.play(FadeIn(year_2004), FadeIn(diamond_2004))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2004, ref_2004, citation_2004]])
        self.wait(7)
        self.play(*[FadeOut(obj) for obj in [description_2004, ref_2004, citation_2004]])
        # 2004 : Corvez
        self.play(FadeIn(diamond_2004_corvez))
        self.play(*[FadeIn(obj) for obj in [description_2004_corvez, ref_2004_corvez, citation_2004_corvez]])
        self.wait(10)
        self.play(*[FadeOut(obj) for obj in [description_2004_corvez, ref_2004_corvez, citation_2004_corvez]])
        # 2004 : Wenger
        self.play(FadeIn(diamond_2004_wenger))
        self.play(*[FadeIn(obj) for obj in [description_2004_wenger, ref_2004_wenger, citation_2004_wenger]])
        self.wait(7)
        self.play(*[FadeOut(obj) for obj in [description_2004_wenger, ref_2004_wenger, citation_2004_wenger]])
        # 2006 : Zein
        self.play(frame.animate.shift(RIGHT * 2), ShowCreation(line_2006))
        # self.play(*[ShowCreation(obj) for obj in [line_2006]])
        self.add(diamond_2004)
        self.play(FadeIn(year_2006), FadeIn(diamond_2006))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2006, ref_2006, citation_2006]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_2006, ref_2006, citation_2006]])
        # 2008 : Paganelli
        self.play(frame.animate.shift(RIGHT * 2), ShowCreation(line_2008))
        self.add(diamond_2006)
        self.play(FadeIn(year_2008), FadeIn(diamond_2008))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2008, ref_2008, citation_2008]])
        self.wait(7)
        self.play(*[FadeOut(obj) for obj in [description_2008, ref_2008, citation_2008]])
        # 2011 : Thomas catastrophe theory
        self.play(frame.animate.shift(RIGHT * 3), ShowCreation(line_2011))
        self.add(diamond_2008)
        self.play(FadeIn(year_2011), FadeIn(diamond_2011))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2011, ref_2011, citation_2011]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_2011, ref_2011, citation_2011]])
        # 2016 : Thomas distance theory
        self.play(frame.animate.shift(RIGHT * 5), ShowCreation(line_2016))
        self.add(diamond_2011)
        self.play(FadeIn(year_2016), FadeIn(diamond_2016))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2016, ref_2016, citation_2016]])
        self.wait(5)
        self.play(*[FadeOut(obj) for obj in [description_2016, ref_2016, citation_2016]])
        # 2020 : Capco
        self.play(frame.animate.shift(RIGHT * 4), ShowCreation(line_2020))
        self.add(diamond_2016)
        self.play(FadeIn(year_2020), FadeIn(diamond_2020))
        self.wait(0.5)
        self.play(*[FadeIn(obj) for obj in [description_2020, citation_2020, ref_2020]])
        self.wait(7)
        self.play(*[FadeOut(obj) for obj in [description_2020, citation_2020, ref_2020]])
        #2023 : PhD
        text_ecarp = Text("Efficient Certified Algorithms for Robot Motion Planning", font_size=20, line_width=6).next_to(line_2023, UP).shift(RIGHT * 3)
        text_ecarp_short = Text("ECARP", line_width=6, color=YELLOW_D).next_to(line_2023, UP)
        for ii in [0, 9, 18, 31, 42]:
            text_ecarp[ii].set_color(YELLOW_D)
        self.play(frame.animate.shift(RIGHT * 3), ShowCreation(line_2023))
        self.add(diamond_2020)
        self.play(FadeIn(year_2023), FadeIn(diamond_2023), FadeIn(text_ecarp))
        self.wait(3)
        self.play(Transform(text_ecarp, text_ecarp_short))
        self.wait()
        self.play(*[FadeIn(obj) for obj in [description_2023]])
        self.wait(7)

        self.play(frame.animate.scale(3).shift(LEFT * 13), run_time=3)
        brief_history = Text("Brief history of serial cuspidal robots", font_size=30).fix_in_frame().shift(UP * 3)
        self.add(brief_history)
        self.wait()

        self.embed()

    def FadeInFadeOut(self, *in_obj, wait_time=3):
        self.play(*[FadeIn(item) for item in in_obj])
        self.wait(wait_time)
        self.play(*[FadeOut(item) for item in in_obj])

    def FadeIt(self, *in_obj):
        self.play(*[Transform(k2, k2.copy().set_opacity(0.2)) for k2 in in_obj])


    def draw_mirror(self, point1, point2, ee_point, l1, _add_obj=False):

        alpha = angle_between_vectors(ee_point - point1, point2 - point1)
        beta = angle_between_vectors(RIGHT, ee_point - point1)
        theta1_mirror = -(alpha - beta)
        mirror_point2 = point1 + np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        new_point1 = np.array([l1 * cos(theta1_mirror), l1 * sin(theta1_mirror), 0])
        theta2_mirror = angle_between_vectors(new_point1, ee_point - (point1 + new_point1))

        if not _add_obj:
            return [theta1_mirror, theta2_mirror]

        joint2 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(mirror_point2)
        mirror_link1 = DashedLine(point1, mirror_point2)
        mirror_link2 = DashedLine(mirror_point2, ee_point)
        joint1 = Circle(radius=0.2, fill_color=BLACK, fill_opacity=1).move_to(point1)

        self.play(*[FadeIn(obj) for obj in [mirror_link1, mirror_link2, joint2, joint1]])

def my_line(start, end, dashed=False):
    if dashed:
        return DashedLine([start - 1988, 0, 0], [end - 1988, 0, 0])
    else:
        return Line([start - 1988, 0, 0], [end - 1988, 0, 0])


class Test(Scene):

    def construct(self) -> None:
        text_ecarp = Text("Efficient Certified Algorithms for Robot Motion Planning", font_size=20, line_width=6)
        for ii in [0, 9, 18, 31, 42]:
            text_ecarp[ii].set_color(YELLOW_D)

        text_ecarp_short = Text("ECARP", line_width=6, color=YELLOW_D)
        self.add(text_ecarp)

        self.wait()
        self.play(Transform(text_ecarp, text_ecarp_short))
        self.embed()
