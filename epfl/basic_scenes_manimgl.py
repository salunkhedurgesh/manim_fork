from functions.phd_functions.functions_epfl import *
from manim_slides.slide import Slide

path_resources = "resources/"


class FrontPage(Scene):
    """
    Scene for initial page and its animation to start the presentation
    """

    def construct(self):
        # objects for front page (big titles and logos)
        title_talk = TexText(r"6R cuspidal robots: dangers in cobots \\ and how to avoid them", font_size=60,
                             color=YELLOW_D).shift(UP * 0.5)
        author_title = TexText("Durgesh Salunkhe").move_to(title_talk.get_bottom() + [0, -1, 0])
        seminar_title = TexText("Talk at: ").move_to(author_title.get_bottom() + [-1, -1, 0])
        date = TexText(r"$13^{th}$ Nov, $2023$", font_size=36).move_to(seminar_title.get_bottom() + [1, -0.5, 0])

        ls2n_logo_big = ImageMobject("resources/raster_images/ls2n_logo.png").scale(0.5).shift(np.array([5, 2.4, 0]))
        ecn_logo = ImageMobject("resources/raster_images/ecn_logo.jpg").scale(0.3).shift(LEFT * 5 + UP * 2.5)
        epfl_logo_big = ImageMobject("resources/raster_images/epfl_logo.png").scale(0.3).next_to(seminar_title, RIGHT)

        # objects for background page (small titles and logos)
        top_line = Line([-8, 3.4, 0], [8, 3.4, 0], stroke_width=0.8, opacity=1, color=WHITE)
        bottom_line = Line([-8, -3.7, 0], [8, -3.7, 0], stroke_width=0.3)
        ls2n_logo = ImageMobject("resources/raster_images/ls2n_logo.png").scale(0.15).to_corner(UR).shift([0.4, 0.4, 0])
        bottom_text = TexText("6R cuspidal robots: dangers in cobots and how to avoid them", font_size=24).to_edge(
            DOWN, buff=0.02)
        epfl_logo = ImageMobject("resources/raster_images/epfl_logo.png").scale(0.1).next_to(bottom_text, LEFT).shift(
            LEFT * 2.5)

        # Animations
        self.add(ls2n_logo_big, ecn_logo, epfl_logo_big)  # adds the objects for front page presentation
        self.add(title_talk, author_title, seminar_title, date)
        # self.embed()
        # self.remove(ecn_logo, author_title, seminar_title)
        # animations to transition from front page to background page
        # self.play(*[ReplacementTransform(ii, jj) for ii, jj in zip([ls2n_logo_big, epfl_logo_big, title_talk],
        #                                                            [ls2n_logo, epfl_logo, bottom_text])], run_time=2)
        # self.play(*[ShowCreation(ii) for ii in [bottom_line, top_line]])



