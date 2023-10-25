from manimlib import *


def get_background(title: str = None):
    top_line = Line([-8, 3.4, 0], [8, 3.4, 0], stroke_width=0.8)
    bottom_line = Line([-8, -3.7, 0], [8, -3.7, 0], stroke_width=0.3)
    ls2n_logo = ImageMobject("resources/raster_images/ls2n_logo.png").scale(0.15).to_corner(UR).shift([0.4, 0.4, 0])
    bottom_text = TexText("6R cuspidal robots: dangers in cobots and how to avoid them", font_size=24).to_edge(
        DOWN, buff=0.02)
    epfl_logo = ImageMobject("resources/raster_images/epfl_logo.png").scale(0.1).next_to(bottom_text, LEFT).shift(
        LEFT * 2.5)
    if title is not None:
        title_text = TexText(title, color=YELLOW_D).to_corner(UL, buff=0.2)
        return Group(ls2n_logo, epfl_logo, bottom_text, bottom_line, top_line, title_text)
    else:
        return Group(ls2n_logo, epfl_logo, bottom_text, bottom_line, top_line)
