import sys
from icra_supplementary import *


def get_Background(title=None, page=None):
    bottom_title = Tex(r"Trajectory planning problems in commercial cuspidal robots").scale(0.7).move_to(
        [0, -3.7, 0])
    bottom_line = Line(start=[-7, -3.5, 0], end=[7, -3.5, 0], stroke_width=0.5)
    top_line = Line(start=[-7, 3.2, 0], end=[7, 3.2, 0], stroke_width=0.5)

    icra_L2 = ImageMobject(
        "D:\\Work\\Projects\\PhD\\manim_durgesh\\PhD_thesis\\sixR\\resources\\images\\icra23_logo.png").scale(
        0.32).move_to([-6, 3.5, 0])
    ls2n = ImageMobject(
        "D:\\Work\\Projects\\PhD\\manim_durgesh\\PhD_thesis\\sixR\\resources\\images\\ls2n_logo.png").scale(
        0.06).move_to([6, 3.5, 0])
    top_title = None
    page_num = None

    if title is not None:
        top_title = Tex(title).scale(0.8).move_to(
            [0, 3.5, 0]).set_color(YELLOW)
    if page is not None:
        page = str(page)
        page_num = MathTex(page).move_to([6.5, -3.7, 0]).scale(0.8)

    if title is None and page is None:
        return bottom_title, bottom_line, top_line, icra_L2, ls2n
    elif title is None:
        return bottom_title, bottom_line, top_line, icra_L2, page_num, ls2n
    elif page is None:
        return bottom_title, bottom_line, top_line, icra_L2, top_title, ls2n
    else:
        return bottom_title, bottom_line, top_line, icra_L2, page_num, top_title, ls2n


def mirror_position(object, direction='V'):
    center_c = object.get_center()
    if direction == 'V':
        return [-center_c[0], -center_c[1], center_c[2]]
    else:
        return [center_c[0], -center_c[1], center_c[2]]


def get_ActiveLink(p1, p2, link_color=None, radius=0.075, show_piston=False):
    if link_color is None:
        link_color = [DARK_BLUE, DARK_BLUE]
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link_length = la.norm(p2 - p1)
    if link_length < 1e-5:
        return 0

    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)
    ret_object = Group()

    link = Surface(
        lambda u, v: np.array([
            radius * np.cos(u),
            radius * np.sin(u),
            v]),
        v_range=[-link_length / 2 - radius * 0.5, link_length / 2 + radius * 0.5], u_range=[0, TAU],
        checkerboard_colors=link_color, stroke_width=0.0001
    )
    link.rotate(angle=angle, axis=axis, about_point=link.get_center()).move_to((p1 + p2) / 2)
    ret_object.add(link)

    if show_piston:
        piston = Surface(
            lambda u, v: np.array([
                radius * 1.5 * np.cos(u),
                radius * 1.5 * np.sin(u),
                v]),
            v_range=[-link_length / 4, link_length / 4], u_range=[0, TAU],
            checkerboard_colors=[PURPLE, PURPLE], stroke_width=0.0001
        )
        piston.rotate(angle=angle, axis=axis, about_point=piston.get_center()).move_to(p1 + (p2 - p1) / 4)
        ret_object.add(piston)

    return ret_object


def get_Box(p1, p2, box_color=None, radius=0.075, opacity=1, box_bre=None, box_h=None):
    if box_color is None:
        box_color = [DARK_BLUE, DARK_BLUE]
    if isinstance(p1, list) or isinstance(p2, list):
        p1 = np.array(p1)
        p2 = np.array(p2)

    link_length = la.norm(p2 - p1)
    box_len = link_length + 0.5
    box_bre = min(link_length / 3, 0.6) if box_bre is None else box_bre
    box_h = box_bre if box_h is None else box_h
    if link_length < 1e-5:
        return 0

    axis, angle = get_axis_angle(np.array([0, 0, 1]), p2 - p1)
    ret_object = Group()

    bottom_surface = Surface(
        lambda u, v: np.array([
            u,
            -box_h / 2,
            v
        ]),
        v_range=[-box_len / 2, box_len / 2], u_range=[-box_bre / 2, box_bre / 2],
        checkerboard_colors=box_color, stroke_width=0.0001, fill_opacity=opacity
    )

    top_surface = Surface(
        lambda u, v: np.array([
            u,
            box_h / 2,
            v
        ]),
        v_range=[-box_len / 2, box_len / 2], u_range=[-box_bre / 2, box_bre / 2],
        checkerboard_colors=box_color, stroke_width=0.0001, fill_opacity=opacity
    )

    side_surface1 = Surface(
        lambda u, v: np.array([
            -box_bre / 2,
            u,
            v
        ]),
        v_range=[-box_len / 2, box_len / 2], u_range=[-box_h / 2, box_h / 2],
        checkerboard_colors=box_color, stroke_width=0.0001
    )

    side_surface2 = Surface(
        lambda u, v: np.array([
            box_bre / 2,
            u,
            v
        ]),
        v_range=[-box_len / 2, box_len / 2], u_range=[-box_h / 2, box_h / 2],
        checkerboard_colors=box_color, stroke_width=0.0001
    )

    down_surface = Surface(
        lambda u, v: np.array([
            v,
            u,
            -box_len / 2
        ]),
        v_range=[-box_bre / 2, box_bre / 2], u_range=[-box_h / 2, box_h / 2],
        checkerboard_colors=box_color, stroke_width=0.0001
    )

    up_surface = Surface(
        lambda u, v: np.array([
            v,
            u,
            box_len / 2
        ]),
        v_range=[-box_bre / 2, box_bre / 2], u_range=[-box_h / 2, box_h / 2],
        checkerboard_colors=box_color, stroke_width=0.0001
    )

    ret_object.add(bottom_surface, top_surface, side_surface1, side_surface2, up_surface, down_surface)
    ret_object.rotate(angle=angle, axis=axis, about_point=ret_object.get_center()).move_to(p1 + (p2 - p1) / 2)

    return ret_object


def get_ParaRobot(parameters=None, offset=None, para_joint=None, para_ext=True):
    if parameters is None:
        parameters = [1, 0, 0, 1, 0, 0, 1.8, 0, 0.5, 1.8, 0, 0.5, 3.5]
    offset = -3 if offset is None else offset
    para_joint = -3 if para_joint is None else para_joint

    sph_joint = Sphere(radius=0.15, checkerboard_colors=[YELLOW, YELLOW], stroke_width=0.0001)
    a1, theta1, h1, a2, theta2, h2, b1, phi1, h3, b2, phi2, h4, t = parameters

    # parameters = [a1, theta1, h1, a2, theta2, h2, b1, phi1, h3, b2, phi2, h4, t]
    rigid_link = get_ActiveLink([0, 0, offset], [a1 * np.cos(theta1), a1 * np.sin(theta1), offset + h1], radius=0.1)
    rigid_link2 = get_ActiveLink([0, 0, offset], [a2 * np.sin(theta2), a2 * np.cos(theta2), offset + h2], radius=0.1)
    sph_joint1 = sph_joint.copy().move_to([a1 * np.cos(theta1), a1 * np.sin(theta1), offset + h1])
    sph_joint2 = sph_joint.copy().move_to([a2 * np.sin(theta2), a2 * np.cos(theta2), offset + h2])

    active_leg = get_ActiveLink([a1 * np.cos(theta1), a1 * np.sin(theta1), offset + h1],
                                [b1 * np.cos(phi1), b1 * np.sin(phi1), t + h3 + offset],
                                link_color=[RED, RED], show_piston=True)
    active_leg2 = get_ActiveLink([a2 * np.sin(theta2), a2 * np.cos(theta2), offset + h2],
                                 [b2 * np.sin(phi1), b2 * np.cos(phi1), t + h4 + offset],
                                 link_color=[RED, RED], show_piston=True)

    rigid_link3 = get_ActiveLink([0, 0, offset + t + h3], [b1 * np.cos(phi1), b1 * np.sin(phi1), t + h3 + offset],
                                 radius=0.1)
    rigid_link4 = get_ActiveLink([0, 0, offset + t + h4], [b2 * np.sin(phi1), b2 * np.cos(phi1), t + h4 + offset],
                                 radius=0.1)

    sph_joint3 = sph_joint.copy().move_to([b1 * np.cos(phi1), b1 * np.sin(phi1), t + h3 + offset])
    sph_joint4 = sph_joint.copy().move_to([b2 * np.sin(phi1), b2 * np.cos(phi1), t + h4 + offset])

    mcg1 = get_ActiveLink([-0.4, 0, offset + t], [0.4, 0, offset + t], radius=0.15, link_color=[RED_B, RED_B])
    mcg2 = get_ActiveLink([0, -0.4, offset + t], [0, 0.4, offset + t], radius=0.15, link_color=[GREEN_C, GREEN_C])
    vert_link = get_ActiveLink([0, 0, offset], [0, 0, offset + t + max(h3, h4)], radius=0.1,
                               link_color=[GOLD_D, GOLD_D])

    pj1 = get_ActiveLink([0, 0, offset + t + max(h3, h4)], [0, 0, offset + t + max(h3, h4) + 1], radius=0.05)
    pj2 = get_ActiveLink([0, 0, offset + t + max(h3, h4) + 1], [para_joint, 0, offset + t + max(h3, h4) + 1],
                         radius=0.05)
    pj3 = get_ActiveLink([0, 0, offset + t + max(h3, h4) + 0.5], [para_joint, 0, offset + t + max(h3, h4) + 0.5],
                         radius=0.05)
    tool = get_ActiveLink([para_joint, 0, offset + t + max(h3, h4) + 1], [para_joint, 0, offset + t], radius=0.05,
                          link_color=[LIGHT_PINK, LIGHT_PINK])

    robot = Group()
    robot.add(active_leg, active_leg2, rigid_link, rigid_link2, sph_joint1, sph_joint2)
    robot.add(rigid_link3, rigid_link4, sph_joint3, sph_joint4, mcg1, mcg2, vert_link)
    if para_ext:
        robot.add(pj1, pj2, pj3, tool)

    return robot


def get_implicitplot(paras=None, rdw=None):
    if paras is None:
        a, a_prime, h, hk, t = [1, 1, 0, 0, 1]
    else:
        a, a_prime, h, hk, t = paras
    ax = Axes(x_range=[-PI, PI], y_range=[-PI, PI], x_length=6, y_length=6, tips=False)

    para_sing = ax.plot_implicit_curve(
        lambda alpha, beta: np.cos(beta) ** 2 * np.sin(alpha) ** 2 * a * h * hk * t - np.cos(beta) ** 2 * np.sin(alpha)
                            * np.cos(alpha) * a_prime * h * t ** 2 + np.cos(beta) ** 2 * np.sin(alpha) * np.cos(alpha)
                            * h * hk * t ** 2 + np.sin(beta) * np.sin(alpha) * np.cos(alpha) * a * a_prime * h * t -
                            np.sin(beta) * np.sin(alpha) * np.cos(alpha) * a * h * hk * t - np.cos(beta) *
                            np.sin(beta) * np.cos(alpha) ** 2 * a * a_prime * hk * t + np.cos(beta) * np.sin(beta) *
                            np.sin(alpha) * a * a_prime * h * t + np.cos(beta) * np.sin(beta) * np.sin(alpha) * a *
                            h * hk * t - np.cos(beta) * np.sin(beta) * np.sin(alpha) ** 2 * a * a_prime * hk * t +
                            np.cos(beta) ** 2 * np.cos(alpha) * a ** 2 * h ** 2 + np.cos(beta) * np.cos(alpha) ** 2 *
                            a_prime ** 2 * t ** 2 - np.cos(beta) * np.cos(alpha) ** 2 * hk ** 2 * t ** 2 - np.sin(beta)
                            * np.sin(alpha) * a ** 2 * a_prime ** 2 - np.sin(beta) * np.cos(alpha) * a * a_prime ** 2
                            * t + np.sin(beta) * np.cos(alpha) ** 2 * a_prime * h * t ** 2 - np.sin(beta) *
                            np.cos(alpha) ** 2 * h * hk * t ** 2 + np.cos(beta) * np.sin(alpha) * a ** 2 * a_prime * h
                            - np.cos(beta) ** 2 * np.sin(alpha) * a * h ** 2 * t + np.sin(beta) ** 2 * np.cos(alpha) *
                            a ** 2 * a_prime * hk + np.cos(beta) ** 2 * np.cos(alpha) ** 2 * a * a_prime * h * t +
                            np.cos(beta) * np.sin(beta) * np.sin(alpha) ** 2 * a * h ** 2 * t + np.cos(beta) *
                            np.sin(beta) * np.cos(alpha) ** 2 * a * h ** 2 * t - np.sin(beta) ** 2 * np.sin(alpha) ** 2
                            * a * a_prime * h * t - np.sin(beta) ** 2 * np.sin(alpha) * np.cos(alpha) * a_prime * h
                            * t ** 2 + np.sin(beta) ** 2 * np.sin(alpha) * np.cos(alpha) * h * hk * t ** 2 -
                            np.sin(beta) ** 2 * np.cos(alpha) ** 2 * a * h * hk * t - np.cos(beta) * np.sin(beta) *
                            np.cos(alpha) * a ** 2 * a_prime * h - np.cos(beta) * np.sin(beta) * np.cos(alpha) *
                            a ** 2 * h * hk + np.cos(beta) * np.sin(alpha) * np.cos(alpha) * a * a_prime ** 2 * t -
                            np.cos(beta) * np.sin(alpha) * np.cos(alpha) * a * hk ** 2 * t - np.sin(beta) ** 2 *
                            np.sin(alpha) * a * a_prime * hk * t + np.cos(beta) * np.cos(alpha) * a * a_prime *
                            h * t,
        color=RED,
    )
    c = Rectangle(height=5.95, width=5.95, stroke_width=1.5, stroke_color=WHITE)
    alpha_sym = MathTex(r"\alpha").move_to(c.get_bottom() + DOWN * 0.5)
    beta_sym = MathTex(r"\beta").move_to(c.get_left() + LEFT * 0.5)
    pi_left = MathTex(r"-\pi").move_to([-3.2, -3.1, 0])
    pi_right = MathTex(r"\pi").move_to([3.1, -3.1, 0])
    if rdw == "circ":
        circ1 = Circle(radius=1, stroke_width=1, color=GREEN, fill_color=GREEN, fill_opacity=0.6).move_to(
            c.get_center())
        return Group(para_sing, c, alpha_sym, beta_sym, pi_left, pi_right), circ1
    else:
        return Group(para_sing, c, alpha_sym, beta_sym, pi_left, pi_right)


def get_item(input_point, fs=36):
    return Tex("\\begin{itemize} \\item " + input_point + "\\end{itemize}", font_size=fs)
