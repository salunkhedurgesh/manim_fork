# from manimlib import *
from manim import *

path_resources = "resources/"


class BackgroundEPFL(Scene):

    def construct(self) -> None:
        grid_background = NumberPlane((-11, 11), (-5, 5))
        top_line = Line([-8, 3.4, 0], [8, 3.4, 0], stroke_width=0.3)
        bottom_line = Line([-8, -3.5, 0], [8, -3.5, 0], stroke_width=0.3)
        title_text = Tex(r"Introduction", color=YELLOW_D).to_corner(UL, buff=0.2)
        # title_text.shift([0.3, 0, 0])
        ls2n_logo = ImageMobject(path_resources + "ls2n_logo.png").scale(0.1).to_corner(UR)
        ls2n_logo.shift([0.5, 0.6, 0])
        # title_text = TexText("Introduction")
        bottom_text = Tex("6R cuspidal robots: Dangers in cobots and how to avoid them", font_size=36).to_edge(
            DOWN, buff=0.02)
        self.add(ls2n_logo)
        self.add(top_line, bottom_line, title_text, bottom_text)

        # self.embed()


class FrontPage(Scene):
    """
    Scene for initial page and its animation to start the presentation
    """

    def construct(self):
        # objects for front page (big titles and logos)
        title_talk = Tex(r"6R cuspidal robots: dangers in cobots \\ and how to avoid them", font_size=60,
                             color=YELLOW_D).shift(UP * 0.5)
        author_title = Tex("Durgesh Salunkhe").move_to(title_talk.get_bottom() + [0, -1, 0])
        seminar_title = Tex("Talk at: ").move_to(author_title.get_bottom() + [-1, -1, 0])
        date = Tex(r"$13^{th}$ Nov, $2023$", font_size=36).move_to(seminar_title.get_bottom() + [1, -0.5, 0])

        ls2n_logo_big = ImageMobject("resources/raster_images/ls2n_logo.png").scale(0.5).shift(np.array([5, 2.4, 0]))
        ecn_logo = ImageMobject("resources/raster_images/ecn_logo.jpg").scale(0.3).shift(LEFT * 5 + UP * 2.5)
        epfl_logo_big = ImageMobject("resources/raster_images/epfl_logo.png").scale(0.3).next_to(seminar_title, RIGHT)

        # Animations
        self.add(ls2n_logo_big, ecn_logo, epfl_logo_big)  # adds the objects for front page presentation
        self.add(title_talk, author_title, seminar_title, date)


