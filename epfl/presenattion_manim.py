
from manimlib import *
from manim_slides.slide import Slide


class BasicExample(Slide):
    def construct(self):
        circle = Circle(radius=3, stroke_color=BLUE)
        circle2 = Circle(radius=1.5, stroke_color=RED)
        dot = Dot()

        self.play(GrowFromCenter(circle))
        self.next_slide()  # Waits user to press continue to go to the next slide
        self.play(ShowCreation(circle2))
        self.next_slide()

        self.start_loop()  # Start loop
        self.play(MoveAlongPath(dot, circle), run_time=2, rate_func=linear)
        self.end_loop()  # This will loop until user inputs a key

        self.play(dot.animate.move_to(ORIGIN))
