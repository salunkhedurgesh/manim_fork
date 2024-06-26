import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

df = pd.read_excel("resources/data/all_iks_path_25Jun.xlsx", header=None)


def get_theta_specific(df, theta_index):
    theta_list_return = []
    sub_list = []
    for iks_index in range(len(df)):
        if all(df.iloc[iks_index] == [0, 0, 0, 0, 0, 0]):
            theta_list_return.append(sub_list)
            sub_list = []
            continue

        sub_list.append(df.iloc[iks_index][theta_index - 1])

    return theta_list_return


def get_abscissa(in_list):
    counter = 1
    out_list = []
    for sub_list in in_list:
        out_list.append([counter] * len(sub_list))
        counter += 1

    return out_list


def get_theta_plot(df, index):
    theta_list = get_theta_specific(df, index)
    x_list = get_abscissa(theta_list)
    for ii in range(len(theta_list)):
        plt.plot(x_list[ii], theta_list[ii], 'o', markersize=0.1, markeredgecolor='red')

    savefile_name = "resources/images/theta" + str(index) + "_path.png"
    plt.xlabel("Progress of the trajectory")
    plt.ylabel("theta_" + str(index) +"(-Pi, Pi)")

    plt.savefig(savefile_name, dpi=1000)


def get_path(df, seed_index):
    candidate_iks = np.array(df.iloc[seed_index-1])
    complete_path = []
    current_norm = np.inf

    for iks_index in range(len(df)):
        if all(df.iloc[iks_index] == [0, 0, 0, 0, 0, 0]):
            complete_path.append(candidate_iks)
            current_norm = np.inf
            continue

        previous_iks = candidate_iks if len(complete_path) == 0 else complete_path[-1]
        if np.linalg.norm(previous_iks - np.array(df.iloc[iks_index])) < current_norm:
            current_norm = np.linalg.norm(previous_iks - np.array(df.iloc[iks_index]))
            candidate_iks = np.array(df.iloc[iks_index])

    return complete_path


def draw_path(df, seed_index, theta_index):
    chosen_path = get_path(df, seed_index)
    print(chosen_path)
    _path_list = []
    for ii in range(len(chosen_path)):
        _path_list.append(chosen_path[ii][theta_index - 1])

    return _path_list


# path_list = draw_path(df, 1, 4)
# plt.figure()
# plt.plot(np.arange(1, len(path_list) + 1), path_list)
# plt.show()


if __name__ == "__main__":
    for index in range(1, 7):
        plt.figure(index)
        get_theta_plot(df, index)
