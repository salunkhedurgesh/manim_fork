import numpy as np

from functions.maths_functions.motion_transformation import *
from functions.maths_functions.robot_functions import *
from manimlib import *
from numpy import sin, cos, sqrt, pi
import pandas as pd

dict_color_code = {0: BLACK, 2: BLUE_D, 4: YELLOW_D, 6: PURPLE_D, 8: GREEN_D, 10: PINK,
                   12: "#40E0D0", 14: "#9932cc", 16: RED_D}


class VisualizeSurface(ThreeDScene):
    """
        Scene to plot the surface in the workspace of the cobod robot
    """
    def construct(self) -> None:
        # setting frame for slice presentation
        frame = self.camera.frame
        frame.set_euler_angles(theta=PI/2 - 0.1, phi=PI/2, gamma=0)
        # previous_scale = 1
        # previous_shift = LEFT * UP
        # frame.scale(previous_scale)
        frame.move_to(np.array([-0.07568952, -0.7543715, 8.961137]))
        frame.scale(1.3)
        # frame.shift(previous_shift)
        robot_offset = 0.48

        # robot constants
        d_list = [0.8215, 0, 0, 1.079, 0, 0]
        a_list = [0, 1.045, 0.279, 0, 0.325, 0]
        alpha_list = [pi/2, 0, -pi/2, pi/2, pi/2, 0]

        # plotting the slice
        df = pd.read_excel("resources/data/all_iks_path_25Jun.xlsx", header=None)
        theta_instance = []
        record_row = True
        for ii in range(len(df)):
            if record_row:
                theta_instance.append(list(df.iloc[ii]))
                record_row = False
            elif list(df.iloc[ii])[1] == 0:
                record_row = True

        group_sphere = Group()
        print(f"length of dataframe is {len(df)}")
        for ii in range(0, len(theta_instance), 2):
            if ii % 100 == 0:
                print(f"done adding sphere at index {ii}")
            pose = get_transformation(d_list, theta_instance[ii], a_list, alpha_list, coord_num=6)
            position = pose[0:3, 3] * 10
            current_sphere = Sphere(radius=0.03).move_to(position).set_color(YELLOW_D)
            group_sphere.add(current_sphere)

        self.add(group_sphere)

        # reference frame
        ref_sphere = Sphere(radius=0.1).set_color(YELLOW)
        ref_sphere_x = Sphere(radius=0.1).set_color(RED).move_to(np.array([1, 0, 0]))
        ref_sphere_z = Sphere(radius=0.1).set_color(BLUE).move_to(np.array([0, 0, 1]))
        self.add(ref_sphere, ref_sphere_z, ref_sphere_x)
        self.embed()







