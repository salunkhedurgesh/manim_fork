from functions.phd_functions.robot_constants import *
from functions.phd_functions.maple_functions import *
from functions.phd_functions.robot_functions import *
from functions.maths_functions.maths_phd import *
from sympy import *

small_plot_size = 4


def get_small_plot(edge=None, label=False, xvalue=None, yvalue=None, xconfig=None, yconfig=None, only_labels=False, box_opacity=1, label_list=None, label_position="corner"):
    if xvalue is None:
        xvalue = (-3, 3)
    if yvalue is None:
        yvalue = (-3, 3)

    if xconfig is None:
        xconfig = dict(stroke_width=0.001, opacity=0, include_ticks=False, stroke_color=BLACK)
    if yconfig is None:
        yconfig = dict(stroke_width=0.001, opacity=0, include_ticks=False, stroke_color=BLACK)

    plot_to_return = Axes(
        x_range=xvalue,
        y_range=yvalue,
        y_axis_config=yconfig,
        x_axis_config=xconfig,
        width=small_plot_size,
        height=small_plot_size
    )
    width = height = small_plot_size
    shift_buff = 0.85
    if edge is not None:
        plot_to_return.to_edge(edge, buff=SMALL_BUFF * 2)
    box = SurroundingRectangle(plot_to_return, stroke_width=1, buff=0, stroke_opacity=box_opacity)
    if label_list is None:
        label_list = [r"""$(\pi, \pi)$""""", r"""(-$\pi$, -$\pi$)"""""]
    top_right = TexText(label_list[0]).move_to(plot_to_return.get_center()).scale(0.6)
    bottom_left = TexText(label_list[1]).move_to(plot_to_return.get_center()).scale(0.6)

    if label_position == "corner":
        horizontal_shift = [width * shift_buff / 2, -width * shift_buff / 2]
        vertical_shift = [height * 1.1 / 2, -height * 1.1 / 2]
    elif label_position == "center":
        horizontal_shift = [-width * 1.1 / 2, 0]
        vertical_shift = [0, -height * 1.1 / 2]
    else:
        horizontal_shift = [-width * 0.1 / 2, 0]
        vertical_shift = [0, -height * 1.1 / 2]

    top_right.shift(np.array([horizontal_shift[0], vertical_shift[0], 0]))
    bottom_left.shift(np.array([horizontal_shift[1], vertical_shift[1], 0]))
    second_element = Group(box, top_right, bottom_left) if label else box
    if only_labels: second_element = Group(top_right, bottom_left)


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
    plot_to_return.set_height(small_plot_size)
    plot_to_return.set_width(small_plot_size)
    if edge is not None:
        plot_to_return.to_edge(edge, buff=SMALL_BUFF * 2)
    box = SurroundingRectangle(plot_to_return, stroke_width=1, buff=0)

    return plot_to_return, box


def get_critv():
    return lambda rho, z: 65536 * rho ** 16 + 524288 * z ** 2 * rho ** 14 + 1835008 * z ** 4 * rho ** 12 + 3670016 * z ** 6 * rho ** 10 + 4587520 * z ** 8 * rho ** 8 + 3670016 * z ** 10 * rho ** 6 + 1835008 * z ** 12 * rho ** 4 + 524288 * z ** 14 * rho ** 2 + 65536 * z ** 16 - 2228224 * rho ** 14 - 16121856 * z ** 2 * rho ** 12 - 49938432 * z ** 4 * rho ** 10 - 85852160 * z ** 6 * rho ** 8 - 88473600 * z ** 8 * rho ** 6 - 54657024 * z ** 10 * rho ** 4 - 18743296 * z ** 12 * rho ** 2 - 2752512 * z ** 14 + 20692992 * rho ** 12 + 133332992 * z ** 2 * rho ** 10 + 369901568 * z ** 4 * rho ** 8 + 560136192 * z ** 6 * rho ** 6 + 483934208 * z ** 8 * rho ** 4 + 224559104 * z ** 10 * rho ** 2 + 43499520 * z ** 12 - 82698240 * rho ** 10 - 98885632 * z ** 2 * rho ** 8 - 253280256 * z ** 4 * rho ** 6 - 842514432 * z ** 6 * rho ** 4 - 907239424 * z ** 8 * rho ** 2 - 301817856 * z ** 10 + 305178112 * rho ** 8 - 2765285376 * z ** 2 * rho ** 6 - 3004079104 * z ** 4 * rho ** 4 + 480122880 * z ** 6 * rho ** 2 + 749282816 * z ** 8 - 714739200 * rho ** 6 + 7322492416 * z ** 2 * rho ** 4 - 6911564288 * z ** 4 * rho ** 2 + 160135680 * z ** 6 + 1117824960 * rho ** 4 + 964516736 * z ** 2 * rho ** 2 + 195134400 * z ** 4 - 1675600160 * rho ** 2 + 99657312 * z ** 2 + 12271009


def get_conic(k_R, k_z, d_list=None, a_list=None, alpha_list=None, robot_type=None, _sym=False):
    if d_list is None and robot_type is None:
        robot_type = "philippe"

    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    d1, d2, d3 = d_list
    a1, a2, a3 = a_list
    alpha1, alpha2, alpha3 = alpha_list
    ca1 = cos(alpha1)
    sa1 = sin(alpha1)
    ca2 = cos(alpha2)
    sa2 = sin(alpha2)
    ca3 = cos(alpha3)
    sa3 = sin(alpha3)

    if _sym:
        c3, s3 = var('c3, s3')
        return 2 / a1 ** 2 * a2 * a3 ** 2 * c3 * d2 * s3 * sa2 + 2 / a1 ** 2 * a3 * ca2 * d2 ** 2 * d3 * s3 * sa2 + 2 / a1 ** 2 * a2 * a3 * c3 * ca2 * d2 * d3 - k_R / 2 + 1 / a1 ** 2 * a2 ** 2 * a3 * d2 * s3 * sa2 + 1 / a1 ** 2 * a3 * d2 * d3 ** 2 * s3 * sa2 - 1 / a1 ** 2 * a3 * d2 * k_R * s3 * sa2 + 2 / sa1 ** 2 * a3 * ca1 ** 2 * d2 * s3 * sa2 - 2 / sa1 ** 2 * a3 * ca1 * k_z * s3 * sa2 + 2 * a3 * ca2 * d3 * s3 * sa2 + 2 / sa1 ** 2 * a3 * ca1 ** 2 * ca2 * d3 * s3 * sa2 + 1 / a1 ** 2 * a2 ** 2 * a3 ** 2 / 2 + 1 / a1 ** 2 * a2 ** 2 * d2 ** 2 / 2 + 1 / a1 ** 2 * a2 ** 2 * d3 ** 2 / 2 + 1 / a1 ** 2 * a3 ** 2 * d2 ** 2 / 2 + 1 / a1 ** 2 * a3 ** 2 * d3 ** 2 / 2 + 1 / a1 ** 2 * d2 ** 2 * d3 ** 2 / 2 - 1 / a1 ** 2 * a2 ** 2 * k_R / 2 - 1 / a1 ** 2 * a3 ** 2 * k_R / 2 - 1 / a1 ** 2 * d2 ** 2 * k_R / 2 - 1 / a1 ** 2 * d3 ** 2 * k_R / 2 + 1 / sa1 ** 2 * ca1 ** 2 * d2 ** 2 - a3 ** 2 * ca2 ** 2 * s3 ** 2 + 1 / a1 ** 2 * a2 ** 4 / 4 + 1 / a1 ** 2 * a3 ** 4 / 4 + 1 / a1 ** 2 * d2 ** 4 / 4 + 1 / a1 ** 2 * d3 ** 4 / 4 + 1 / a1 ** 2 * k_R ** 2 / 4 + 1 / sa1 ** 2 * k_z ** 2 - d3 ** 2 * sa2 ** 2 - a3 ** 2 * c3 ** 2 + d2 ** 2 / 2 + d3 ** 2 / 2 - a2 ** 2 / 2 + a3 ** 2 / 2 + a1 ** 2 / 4 + 1 / a1 ** 2 * a2 ** 2 * a3 ** 2 * c3 ** 2 + 1 / a1 ** 2 * ca2 ** 2 * d2 ** 2 * d3 ** 2 + 1 / a1 ** 2 * a2 ** 3 * a3 * c3 + 1 / a1 ** 2 * a2 * a3 ** 3 * c3 + 1 / a1 ** 2 * ca2 * d2 ** 3 * d3 + 1 / a1 ** 2 * ca2 * d2 * d3 ** 3 + 1 / sa1 ** 2 * ca1 ** 2 * ca2 ** 2 * d3 ** 2 - 2 / sa1 ** 2 * ca1 * d2 * k_z + ca2 * d2 * d3 - a2 * a3 * c3 + 1 / a1 ** 2 * a3 ** 2 * d2 ** 2 * s3 ** 2 * sa2 ** 2 + 1 / a1 ** 2 * a3 ** 3 * d2 * s3 * sa2 + 1 / a1 ** 2 * a3 * d2 ** 3 * s3 * sa2 + 1 / a1 ** 2 * a2 ** 2 * ca2 * d2 * d3 + 1 / a1 ** 2 * a2 * a3 * c3 * d2 ** 2 + 1 / a1 ** 2 * a2 * a3 * c3 * d3 ** 2 + 1 / a1 ** 2 * a3 ** 2 * ca2 * d2 * d3 - 1 / a1 ** 2 * a2 * a3 * c3 * k_R - 1 / a1 ** 2 * ca2 * d2 * d3 * k_R + 1 / sa1 ** 2 * a3 ** 2 * ca1 ** 2 * s3 ** 2 * sa2 ** 2 + 2 / sa1 ** 2 * ca1 ** 2 * ca2 * d2 * d3 - 2 / sa1 ** 2 * ca1 * ca2 * d3 * k_z + a3 * d2 * s3 * sa2
    else:
        return lambda x, y: 2 / a1 ** 2 * a2 * a3 ** 2 * x * d2 * y * sa2 + 2 / a1 ** 2 * a3 * ca2 * d2 ** 2 * d3 * y * sa2 + 2 / a1 ** 2 * a2 * a3 * x * ca2 * d2 * d3 - k_R / 2 + 1 / a1 ** 2 * a2 ** 2 * a3 * d2 * y * sa2 + 1 / a1 ** 2 * a3 * d2 * d3 ** 2 * y * sa2 - 1 / a1 ** 2 * a3 * d2 * k_R * y * sa2 + 2 / sa1 ** 2 * a3 * ca1 ** 2 * d2 * y * sa2 - 2 / sa1 ** 2 * a3 * ca1 * k_z * y * sa2 + 2 * a3 * ca2 * d3 * y * sa2 + 2 / sa1 ** 2 * a3 * ca1 ** 2 * ca2 * d3 * y * sa2 + 1 / a1 ** 2 * a2 ** 2 * a3 ** 2 / 2 + 1 / a1 ** 2 * a2 ** 2 * d2 ** 2 / 2 + 1 / a1 ** 2 * a2 ** 2 * d3 ** 2 / 2 + 1 / a1 ** 2 * a3 ** 2 * d2 ** 2 / 2 + 1 / a1 ** 2 * a3 ** 2 * d3 ** 2 / 2 + 1 / a1 ** 2 * d2 ** 2 * d3 ** 2 / 2 - 1 / a1 ** 2 * a2 ** 2 * k_R / 2 - 1 / a1 ** 2 * a3 ** 2 * k_R / 2 - 1 / a1 ** 2 * d2 ** 2 * k_R / 2 - 1 / a1 ** 2 * d3 ** 2 * k_R / 2 + 1 / sa1 ** 2 * ca1 ** 2 * d2 ** 2 - a3 ** 2 * ca2 ** 2 * y ** 2 + 1 / a1 ** 2 * a2 ** 4 / 4 + 1 / a1 ** 2 * a3 ** 4 / 4 + 1 / a1 ** 2 * d2 ** 4 / 4 + 1 / a1 ** 2 * d3 ** 4 / 4 + 1 / a1 ** 2 * k_R ** 2 / 4 + 1 / sa1 ** 2 * k_z ** 2 - d3 ** 2 * sa2 ** 2 - a3 ** 2 * x ** 2 + d2 ** 2 / 2 + d3 ** 2 / 2 - a2 ** 2 / 2 + a3 ** 2 / 2 + a1 ** 2 / 4 + 1 / a1 ** 2 * a2 ** 2 * a3 ** 2 * x ** 2 + 1 / a1 ** 2 * ca2 ** 2 * d2 ** 2 * d3 ** 2 + 1 / a1 ** 2 * a2 ** 3 * a3 * x + 1 / a1 ** 2 * a2 * a3 ** 3 * x + 1 / a1 ** 2 * ca2 * d2 ** 3 * d3 + 1 / a1 ** 2 * ca2 * d2 * d3 ** 3 + 1 / sa1 ** 2 * ca1 ** 2 * ca2 ** 2 * d3 ** 2 - 2 / sa1 ** 2 * ca1 * d2 * k_z + ca2 * d2 * d3 - a2 * a3 * x + 1 / a1 ** 2 * a3 ** 2 * d2 ** 2 * y ** 2 * sa2 ** 2 + 1 / a1 ** 2 * a3 ** 3 * d2 * y * sa2 + 1 / a1 ** 2 * a3 * d2 ** 3 * y * sa2 + 1 / a1 ** 2 * a2 ** 2 * ca2 * d2 * d3 + 1 / a1 ** 2 * a2 * a3 * x * d2 ** 2 + 1 / a1 ** 2 * a2 * a3 * x * d3 ** 2 + 1 / a1 ** 2 * a3 ** 2 * ca2 * d2 * d3 - 1 / a1 ** 2 * a2 * a3 * x * k_R - 1 / a1 ** 2 * ca2 * d2 * d3 * k_R + 1 / sa1 ** 2 * a3 ** 2 * ca1 ** 2 * y ** 2 * sa2 ** 2 + 2 / sa1 ** 2 * ca1 ** 2 * ca2 * d2 * d3 - 2 / sa1 ** 2 * ca1 * ca2 * d3 * k_z + a3 * d2 * y * sa2


def get_fkin(theta_list, d_list=None, a_list=None, alpha_list=None, robot_type=None):
    if d_list is None and robot_type is None:
        robot_type = "philippe"

    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    if len(theta_list) == 2:
        theta_list = [1, theta_list[0], theta_list[1]]

    M1 = np.eye(4, 4)
    for index_i in range(len(d_list)):
        # print(index_i)
        M1 = np.matmul(np.matmul(M1, z_rotation_matrix(theta_list[index_i])), z_translation_matrix(d_list[index_i]))
        M1 = np.matmul(np.matmul(M1, x_rotation_matrix(alpha_list[index_i])), x_translation_matrix(a_list[index_i]))

    return M1[0:3, 3]


def get_det(theta_list=None, d_list=None, a_list=None, alpha_list=None, robot_type=None, _val=False):
    if d_list is None and robot_type is None:
        robot_type = "philippe"

    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    d1, d2, d3 = d_list
    a1, a2, a3 = a_list
    alpha1, alpha2, alpha3 = alpha_list

    if _val:
        if len(theta_list) == 3:
            theta_list = theta_list[1:3]

        t2, t3 = theta_list[0], theta_list[1]
        return a3 * (((math.cos(t2) * math.sin(alpha2) * d2 - math.sin(t2) * a2 * math.cos(alpha2)) * math.sin(alpha1) + a1 * math.sin(t2) * math.cos(alpha1) * math.sin(alpha2)) * a3 * math.cos(t3) ** 2 + (((a1 * math.sin(t3) * a3 - d2 * d3 * math.sin(t2)) * math.cos(alpha2) ** 2 - math.sin(alpha2) * (math.sin(t2) * math.sin(t3) * a3 * d2 + a1 * d3) * math.cos(alpha2) + math.cos(t2) * a2 * d2 * math.sin(alpha2) + d2 * d3 * math.sin(t2) - math.sin(t3) * a3 * (math.cos(t2) * a2 + a1)) * math.sin(alpha1) + a1 * math.cos(alpha1) * (math.cos(t2) * math.sin(t3) * math.cos(alpha2) * math.sin(alpha2) * a3 + math.cos(t2) * d3 * math.cos(alpha2) ** 2 + math.sin(t2) * math.sin(alpha2) * a2 - math.cos(t2) * d3)) * math.cos(t3) - (-math.sin(t2) * math.cos(alpha2) * a3 + math.sin(t3) * (math.sin(t2) * math.sin(alpha2) * d3 + math.cos(t2) * a2 + a1)) * a2 * math.sin(alpha1))
    else:
        return lambda t2, t3: a3 * (((math.cos(t2) * math.sin(alpha2) * d2 - math.sin(t2) * a2 * math.cos(alpha2)) * math.sin(alpha1) + a1 * math.sin(t2) * math.cos(alpha1) * math.sin(alpha2)) * a3 * math.cos(t3) ** 2 + (((a1 * math.sin(t3) * a3 - d2 * d3 * math.sin(t2)) * math.cos(alpha2) ** 2 - math.sin(alpha2) * (math.sin(t2) * math.sin(t3) * a3 * d2 + a1 * d3) * math.cos(alpha2) + math.cos(t2) * a2 * d2 * math.sin(alpha2) + d2 * d3 * math.sin(t2) - math.sin(t3) * a3 * (math.cos(t2) * a2 + a1)) * math.sin(alpha1) + a1 * math.cos(alpha1) * (math.cos(t2) * math.sin(t3) * math.cos(alpha2) * math.sin(alpha2) * a3 + math.cos(t2) * d3 * math.cos(alpha2) ** 2 + math.sin(t2) * math.sin(alpha2) * a2 - math.cos(t2) * d3)) * math.cos(t3) - (-math.sin(t2) * math.cos(alpha2) * a3 + math.sin(t3) * (math.sin(t2) * math.sin(alpha2) * d3 + math.cos(t2) * a2 + a1)) * a2 * math.sin(alpha1))


def solve_for_t1t2(d_list, a_list, alpha_list, t3, rho, z, x=None):
    if x is None:
        x_val = 0
        y_val = rho
    else:
        x_val = x
        y_val = np.sqrt(rho ** 2 - x_val ** 2)

    d1, d2, d3 = d_list
    a1, a2, a3 = a_list
    alpha1, alpha2, alpha3 = alpha_list
    t2 = var('t2')

    expr1 = (((-math.cos(t3) ** 2 * a3 ** 2 + a3 ** 2 - d3 ** 2) * math.cos(alpha2) ** 2 - 2 * math.sin(t3) * math.sin(
        alpha2) * a3 * d3 * math.cos(alpha2) - math.cos(t3) ** 2 * a3 ** 2 - 2 * a2 * a3 * math.cos(
        t3) - a2 ** 2 + d3 ** 2) * cos(t2) ** 2 + 2 * sin(t2) * (
                         math.sin(t3) * math.cos(alpha2) * a3 - math.sin(alpha2) * d3) * (math.cos(t3) * a3 + a2) * cos(
        t2) + (-math.cos(t3) ** 2 * a3 ** 2 + a3 ** 2 - d3 ** 2) * math.cos(alpha2) ** 2 - 2 * d3 * (
                         math.sin(t3) * math.sin(alpha2) * a3 + d2) * math.cos(alpha2) + 2 * math.cos(
        t3) ** 2 * a3 ** 2 - 2 * math.sin(alpha2) * math.sin(t3) * a3 * d2 + 2 * a2 * a3 * math.cos(
        t3) + a2 ** 2 - a3 ** 2 - d2 ** 2) * math.cos(alpha1) ** 2 + 2 * ((-2 * math.sin(t3) * a3 * d3 * math.cos(
        alpha2) ** 2 + (math.cos(t3) ** 2 * math.sin(alpha2) * a3 ** 2 + (-a3 ** 2 + d3 ** 2) * math.sin(
        alpha2) - math.sin(t3) * a3 * d2) * math.cos(alpha2) + d3 * (math.sin(t3) * a3 + d2 * math.sin(alpha2))) * cos(
        t2) - sin(t2) * (math.sin(t3) * math.sin(alpha2) * a3 + d3 * math.cos(alpha2) + d2) * (math.cos(
        t3) * a3 + a2)) * math.sin(alpha1) * math.cos(alpha1) + (
                        (math.cos(t3) ** 2 * a3 ** 2 - a3 ** 2 + d3 ** 2) * math.cos(alpha2) ** 2 + 2 * math.sin(
                    t3) * math.sin(alpha2) * a3 * d3 * math.cos(alpha2) + (math.cos(t3) * a3 + a2 + d3) * (
                                    math.cos(t3) * a3 + a2 - d3)) * cos(t2) ** 2 + 2 * (math.cos(t3) * a3 + a2) * (
                        -sin(t2) * math.sin(t3) * a3 * math.cos(alpha2) + sin(t2) * math.sin(alpha2) * d3 + a1) * cos(
        t2) + (-2 * a1 * sin(t2) * math.sin(t3) * a3 + 2 * d2 * d3) * math.cos(alpha2) - math.cos(t3) ** 2 * a3 ** 2 + (
                        2 * a1 * sin(t2) * d3 + 2 * math.sin(t3) * a3 * d2) * math.sin(
        alpha2) + a1 ** 2 + a3 ** 2 + d2 ** 2 + d3 ** 2 - rho ** 2

    expr2 = math.sin(alpha1) * sin(t2) * math.cos(t3) * a3 + (
                math.sin(alpha1) * cos(t2) * math.cos(alpha2) + math.cos(alpha1) * math.sin(alpha2)) * math.sin(
        t3) * a3 + (-math.sin(alpha1) * cos(t2) * math.sin(alpha2) + math.cos(alpha1) * math.cos(
        alpha2)) * d3 + math.sin(alpha1) * sin(t2) * a2 + math.cos(alpha1) * d2 - z

    t2_set1 = solve(expr1)
    t2_set2 = solve(expr2)
    theta2_found = False

    for item in t2_set1:
        expre = expr2.subs(t2, item).evalf()
        if abs(expre) < 1e-5:
            theta2_found = True
            t2 = item
            break

    if not theta2_found:
        for item in t2_set2:
            expre = expr1.subs(t2, item).evalf()
            if abs(expre) < 1e-5:
                theta2_found = True
                t2 = item
                break

    if not theta2_found:
        raise ArithmeticError("No solution for theta2 found")
    print("Solved for theta2")

    t1 = var('t1')

    x_coord = (cos(t1) * math.cos(t2) - sin(t1) * math.cos(alpha1) * math.sin(t2)) * math.cos(t3) * a3 + (-cos(t1) * math.sin(t2) * math.cos(alpha2) - sin(t1) * math.cos(alpha1) * math.cos(t2) * math.cos(alpha2) + sin(t1) * math.sin(alpha1) * math.sin(alpha2)) * math.sin(t3) * a3 + (cos(t1) * math.sin(t2) * math.sin(alpha2) + sin(t1) * math.cos(alpha1) * math.cos(t2) * math.sin(alpha2) + sin(t1) * math.sin(alpha1) * math.cos(alpha2)) * d3 + cos(t1) * math.cos(t2) * a2 - sin(t1) * math.cos(alpha1) * math.sin(t2) * a2 + sin(t1) * math.sin(alpha1) * d2 + cos(t1) * a1 - x_val
    y_coord = (sin(t1) * math.cos(t2) + cos(t1) * math.cos(alpha1) * math.sin(t2)) * math.cos(t3) * a3 + (-sin(t1) * math.sin(t2) * math.cos(alpha2) + cos(t1) * math.cos(alpha1) * math.cos(t2) * math.cos(alpha2) - cos(t1) * math.sin(alpha1) * math.sin(alpha2)) * math.sin(t3) * a3 + (sin(t1) * math.sin(t2) * math.sin(alpha2) - cos(t1) * math.cos(alpha1) * math.cos(t2) * math.sin(alpha2) - cos(t1) * math.sin(alpha1) * math.cos(alpha2)) * d3 + sin(t1) * math.cos(t2) * a2 + cos(t1) * math.cos(alpha1) * math.sin(t2) * a2 - cos(t1) * math.sin(alpha1) * d2 + sin(t1) * a1 - y_val

    t1_set1 = solve(x_coord)
    t1_set2 = solve(y_coord)
    theta1_found = False

    for item in t1_set1:
        if abs(y_coord.subs(t1, item).evalf()) < 1e-5:
            theta1_found = True
            t1 = item
            break

    if not theta1_found:
        for item in t1_set2:
            if abs(x_coord.subs(t1, item).evalf()) < 1e-5:
                theta1_found = True
                t1 = item
                break
    if not theta1_found:
        raise ArithmeticError("No solution for theta1 found")

    print("Solved for theta1")
    return [t1, t2, t3]

def get_ikin(rho=None, z=None, theta_list=None, d_list=None, a_list=None, alpha_list=None, robot_type=None, _val=False, x=None):
    if d_list is None and robot_type is None:
        robot_type = "philippe"

    if robot_type is not None:
        d_list, a_list, alpha_list = get_dh_parameters(robot=robot_type)

    d1, d2, d3 = d_list
    a1, a2, a3 = a_list
    alpha1, alpha2, alpha3 = alpha_list
    ca1 = cos(alpha1)
    sa1 = sin(alpha1)
    ca2 = cos(alpha2)
    sa2 = sin(alpha2)
    ca3 = cos(alpha3)
    sa3 = sin(alpha3)

    if theta_list is not None:
        if len(theta_list) == 2:
            theta_list = [1, theta_list[0], theta_list[1]]
        ee = get_fkin(theta_list, d_list, a_list, alpha_list)
        rho = np.sqrt(ee[0] ** 2 + ee[1] ** 2)
        z = ee[2]

    c3, s3 = var('c3, s3')

    R = rho ** 2 + z ** 2
    conic_equation = get_conic(k_R=R, k_z=z, d_list=d_list, a_list=a_list, alpha_list=alpha_list, _sym=True)
    circle_eqn = c3 ** 2 + s3 ** 2 - 1

    # print(f"{rho} and {z}")

    iks = solve([conic_equation, circle_eqn], [c3, s3])
    vector_theta3 = []
    for sol in iks:
        first_sol_imag = sol[0].as_real_imag()
        second_sol_imag = sol[1].as_real_imag()
        if abs(first_sol_imag[1]) < 1e-10 and abs(second_sol_imag[1]) < 1e-10:
            vector_theta3.append(math.atan2(second_sol_imag[0], first_sol_imag[0]))

    # solving for theta2
    final_solutions = []
    for t3_val in vector_theta3:
        all_solutions = solve_for_t1t2(d_list, a_list, alpha_list, t3_val, rho, z, x)
        final_solutions.append(all_solutions)

    return final_solutions


def draw_robot2r(theta1=PI/4, theta2=PI/4, link_lengths=None, offset=None):
    if link_lengths is None:
        link_lengths = [0.5, 1]
    if offset is None:
        offset = np.array([-2, -1, 0])

    l1, l2 = link_lengths
    point1 = np.array([0, 0, 0])
    point2 = point1 + l1 * cos(theta1)
    ee_point = point2 + l2 * cos(theta1 + theta2)
    joint1 = Circle(radius=0.1).move_to(point1)
    joint2 = Circle(radius=0.1).move_to(point2)
    ee = Dot().move_to(ee_point)

def real_solutions(sol_set):
    vector_return = []
    for sol in sol_set:
        first_sol_imag = sol[0].as_real_imag()
        second_sol_imag = sol[1].as_real_imag()
        if abs(first_sol_imag[1]) < 1e-10 and abs(second_sol_imag[1]) < 1e-10:
            vector_return.append((first_sol_imag[0], second_sol_imag[0]))

    return vector_return


