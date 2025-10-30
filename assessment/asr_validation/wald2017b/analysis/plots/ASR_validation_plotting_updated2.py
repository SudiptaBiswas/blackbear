#!/usr/bin/env python3
# /*************************************************/
# /*           DO NOT MODIFY THIS HEADER           */
# /*                                               */
# /*                 BlackBear                     */
# /*                                               */
# /*    (c) 2020 Battelle Energy Alliance, LLC     */
# /*            ALL RIGHTS RESERVED                */
# /*                                               */
# /*   Prepared by Battelle Energy Alliance, LLC   */
# /*     Under Contract No. DE-AC07-05ID14517      */
# /*     With the U. S. Department of Energy       */
# /*                                               */
# /*     See COPYRIGHT for full restrictions       */
# /*************************************************/
"""Create all plots for asr_validation/wald2017b assessment case."""
from glob import glob
import itertools as it
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Plot Settings
# plt.style.use('../../../../../scripts/plot_style.mplstyle')

# Global Constants
AXES = ["x", "y", "z"]
# SEQ = range(1, 13)
SEQS = [range(1, 17), range(1,13), range(1,17)]
AXES2 = ["x", "z"]
SEQ2 = range(8, 17)
BOUNDARY = range(101,106)

TIME_DIV = 86400
MEAN_DIV = 0.48

# plotting constants
COLOR_LINES = ["r-", "b-", "g-"]
COLOR_LINES_DASH = ["r--", "b--", "g--"]
SUBSCRIPTS = ["xx", "yy", "zz"]
AXES_LIST = ['dispx', 'dispy', 'dispz']

def extract_name(fp):
    """Extract specimen name and axis from the name of the csv file."""
    clean = re.sub(r"\./figure[0-9]{2}/", "", fp)
    clean = clean.replace('.csv', '')
    split = clean.split('_')
    return (split[0], split[1].replace('p', ''))


def define_dfs(*args):
    """Return dataframe containing stacked data of all csv files."""
    df_settings = {'header': 0, 'names': ['x', 'y']}
    df_array = []
    for fp in args:
        df = pd.read_csv(fp, **df_settings)
        df = df.assign(**{'specimen': extract_name(fp)[0],
                          'axis': extract_name(fp)[1]})
        df_array.append(df)
    return pd.concat(df_array)

def define_dataframes(*args):
    """Return a list of pandas dataframes."""
    return [pd.read_csv(fp) for fp in args]


def sum_cols(df, col1, col2):
    """Compute absolute value of two columns from a dataframe."""
    return abs(df[col1]) + abs(df[col2])


def make_disp_sum_func(df):
    """Return a function initialized with a dataframe."""

    def sum_abs_cols(axis, num):
        """Sum the columns specified by axis and num."""
        return sum_cols(df, f"disp_{axis}_p{num}_pos", f"disp_{axis}_p{num}_neg")

    return sum_abs_cols


def disp_dict(func, sequences):
    all_dicts=[]
    for ax, seq in zip(AXES, sequences):
        all_dicts.append({f"disp{ax}_p{i}": func(ax, i) for i in seq})
    """Return dictionary of key:values for all axes and numbers."""
    return {**all_dicts[0], **all_dicts[1], **all_dicts[2]}


def compute_displacements(df):
    disp_func = make_disp_sum_func(df)
    new_df = df.assign(**disp_dict(disp_func, SEQS))
    return new_df


def disp_group(idx):
    """Utility function used by groupby to group disp{x,y,z} values."""
    return idx.split("_")[0]


def get_displacement_mean(df):
    """Return dataframe with each row being an axis of displacement."""
    cols_wanted_x = [f'dispx_p{i}' for i in range(1,17)]
    cols_wanted_y = [f'dispy_p{i}' for i in range(1,13)]
    cols_wanted_z = [f'dispz_p{i}' for i in range(1,17)]

    dat_x = df[cols_wanted_x].transpose()
    dat_y = df[cols_wanted_y].transpose()
    dat_z = df[cols_wanted_z].transpose()

    dat_mean_x = dat_x.groupby(by=disp_group).apply(lambda x: x.mean())
    dat_mean_y = dat_y.groupby(by=disp_group).apply(lambda x: x.mean())
    dat_mean_z = dat_z.groupby(by=disp_group).apply(lambda x: x.mean())
    dat_mean = pd.concat([dat_mean_x, dat_mean_y, dat_mean_z])

    return dat_mean

    #
    # def get_displacement_mean(df, ax, seq):
    #     """Return dataframe with each row being an axis of displacement."""
    #     # cols_wanted = [f'disp{i}_p{j}' for i,j in zip(AXES, sequences)]
    #     cols_wanted = [f'disp{ax}_p{i}' for i in seq]
    #     dat = df[cols_wanted].transpose()
    #     dat_mean = dat.groupby(by=disp_group).apply(lambda x: x.mean())
    #     return dat_mean


def row_to_list(df, identifier):
    """Return row of dataframe as list where index == identifier."""
    return df.filter(like=identifier, axis=0).squeeze()


def simplify(df, identifier):
    """Return a dataframe with two columns: [time, disp{x|y|z}]."""
    df_mean = get_displacement_mean(df)
    temp = {'time': df.time,
            identifier: row_to_list(df_mean, identifier)}

    return pd.DataFrame(temp)


def main():
    """Script Driver Function."""
    a3_102_L1_px, a3_102_L1_py, a3_102_L1_pz, a3_102_px, a3_102_py, a3_102_pz, a3_202_L2_px, a3_202_L2_py, a3_202_L2_pz, a3_202_L3_px, a3_202_L3_py, a3_202_L3_pz = define_dataframes(
        "./figure07/a3-102-L1_px.csv",
        "./figure07/a3-102-L1_py.csv",
        "./figure07/a3-102-L1_pz.csv",
        "./figure07/a3-102c_px.csv",
        "./figure07/a3-102c_py.csv",
        "./figure07/a3-102c_pz.csv",
        "./figure07/a3-202-L2_px.csv",
        "./figure07/a3-202-L2_py.csv",
        "./figure07/a3-202-L2_pz.csv",
        "./figure07/a3-202-L3_px.csv",
        "./figure07/a3-202-L3_py.csv",
        "./figure07/a3-202-L3_pz.csv",
    )

    # case_A3_102, case_A3_102b, case_A3_102_L1, case_A3_202_L2, case_A3_202_L3 = define_dataframes(
    #     "./data/asr_concrete_blockA3-102.csv",
    #     "./data/asr_concrete_blockA3-102b.csv",
    #     "./data/asr_concrete_blockA3-102L1.csv",
    #     "./data/asr_concrete_blockA3-202L2.csv",
    #     "./data/asr_concrete_blockA3-202L3.csv",
    # )

    # case_A3_102, case_A3_102_L1, case_A3_202_L2, case_A3_202_L3 = define_dataframes(
    #     "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/A3-102c/gold/A3-102c_out.csv",
    #     "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/A3-102-L1/gold/A3-102-L1_out.csv",
    #     "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/A3-202-L2/gold/A3-202-L2_out.csv",
    #     "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/A3-202-L3/gold/A3-202-L3_out.csv",
    # )

    case_A3_102, case_A3_102_L1, case_A3_202_L2, case_A3_202_L3 = define_dataframes(
        "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/outputs/A3-102c_out.csv",
        "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/outputs/A3-102-L1_out.csv",
        "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/outputs/A3-202-L2_out.csv",
        "~/Codes/blackbear/assessment/asr_validation/wald2017b/analysis/outputs/A3-202-L3_out.csv",
    )

    # Add new displacement columns
    case_A3_102 = compute_displacements(case_A3_102)
    # case_A3_102b = compute_displacements(case_A3_102b)
    case_A3_102_L1 = compute_displacements(case_A3_102_L1)
    case_A3_202_L2 = compute_displacements(case_A3_202_L2)
    case_A3_202_L3 = compute_displacements(case_A3_202_L3)


    case_A3_102_surf = pd.concat([(abs(case_A3_102['disp_x_101']) + abs(case_A3_102['disp_x_102'])),
                                  (abs(case_A3_102['disp_z_103']) + abs(case_A3_102['disp_z_104'])),
                                  (abs(case_A3_102['disp_y_105']) + abs(case_A3_102['disp_y_106']))])
    case_A3_102_surf_x = (abs(case_A3_102['disp_x_101']) + abs(case_A3_102['disp_x_102']))
    case_A3_102_surf_z = (abs(case_A3_102['disp_z_103']) + abs(case_A3_102['disp_z_104']))
    case_A3_102_surf_y = (abs(case_A3_102['disp_y_105']) + abs(case_A3_102['disp_y_106']))
    # print(case_A3_102_surf_x)
    ###################################################################
    #                             Plots                               #
    ###################################################################

    #############
    # Figure 01 #
    #############
    ax = plt.figure(14)
    # for ax, cl, sub in zip(AXES_LIST, COLOR_LINES, AXES):
    #     tmp_dat = simplify(case_A3_102, ax)
    #     plt.plot(
    #         (tmp_dat.time - tmp_dat.time[0]) / TIME_DIV + 27,
    #         tmp_dat[ax] / MEAN_DIV * 100,
    #         cl,
    #         label=f'Simulation-{sub}',
    #         markersize=5,
    #         linewidth=3
    #     )
    for ax, cl, sub in zip(AXES_LIST, COLOR_LINES, AXES):
        tmp_dat2 = simplify(case_A3_102, ax)
        plt.plot(
            (tmp_dat2.time - tmp_dat2.time[0]) / TIME_DIV + 27,
            tmp_dat2[ax] / MEAN_DIV * 100,
            cl,
            label=f'Simulation-{sub}',
            markersize=5,
            linewidth=3
        )

    # case_A3_102_time = (case_A3_102['time']-case_A3_102['time'][0])/TIME_DIV + 27
    # plt.plot(case_A3_102_time, case_A3_102_surf_x/MEAN_DIV*100, 'r.', label='surf-x', markersize = 5, linewidth=3)
    # plt.plot(case_A3_102_time, case_A3_102_surf_y/MEAN_DIV*100, 'b.', label='surf-y', markersize = 5, linewidth=3)
    # plt.plot(case_A3_102_time, case_A3_102_surf_z/MEAN_DIV*100, 'g.', label='surf-z', markersize = 5, linewidth=3)

    plt.plot(a3_102_px.x, a3_102_px.y, 'r*', label='Experiment-x', markersize=5)
    plt.plot(a3_102_py.x, a3_102_py.y, 'b*', label='Experiment-y', markersize=5)
    plt.plot(a3_102_pz.x, a3_102_pz.y, 'g*', label='Experiment-z', markersize=5)

    plt.xlabel('Time (days)')
    plt.ylabel('Axial Expansion (%)')
    # plt.legend(frameon=False)
    plt.savefig("A3-102c2_new.png", bbox_inches='tight')

    ax = plt.figure(15)
    for ax, cl, sub in zip(AXES_LIST, COLOR_LINES, AXES):
        tmp_dat = simplify(case_A3_102_L1, ax)
        plt.plot(
            (tmp_dat.time - tmp_dat.time[0]) / TIME_DIV + 27,
            tmp_dat[ax] / MEAN_DIV * 100,
            cl,
            label=f'Simulation-{sub}',
            markersize=5,
            linewidth=3
        )
    plt.plot(a3_102_L1_px.x, a3_102_L1_px.y, 'r*', label='Experiment-x', markersize=5)
    plt.plot(a3_102_L1_py.x, a3_102_L1_py.y, 'b*', label='Experiment-y', markersize=5)
    plt.plot(a3_102_L1_pz.x, a3_102_L1_pz.y, 'g*', label='Experiment-z', markersize=5)
    plt.xlabel('Time (days)')
    plt.ylabel('Axial Expansion (%)')
    # plt.legend(frameon=False)
    plt.savefig("A3-102-L12_new.png", bbox_inches='tight')

    ax = plt.figure(16)
    for ax, cl, sub in zip(AXES_LIST, COLOR_LINES, AXES):
        tmp_dat = simplify(case_A3_202_L2, ax)
        plt.plot(
            (tmp_dat.time - tmp_dat.time[0]) / TIME_DIV + 27,
            tmp_dat[ax] / MEAN_DIV * 100,
            cl,
            label=f'Simulation-{sub}',
            markersize=5,
            linewidth=3
        )
    plt.plot(a3_202_L2_px.x, a3_202_L2_px.y, 'r*', label='Experiment-x', markersize=5)
    plt.plot(a3_202_L2_py.x, a3_202_L2_py.y, 'b*', label='Experiment-y', markersize=5)
    plt.plot(a3_202_L2_pz.x, a3_202_L2_pz.y, 'g*', label='Experiment-z', markersize=5)
    plt.xlabel('Time (days)')
    plt.ylabel('Axial Expansion (%)')
    # plt.legend(frameon=False)
    plt.savefig("A3-202-L22_new.png", bbox_inches='tight')

    ax = plt.figure(17)
    for ax, cl, sub in zip(AXES_LIST, COLOR_LINES, AXES):
        tmp_dat = simplify(case_A3_202_L3, ax)
        plt.plot(
            (tmp_dat.time - tmp_dat.time[0]) / TIME_DIV + 27,
            tmp_dat[ax] / MEAN_DIV * 100,
            cl,
            label=f'Simulation-{sub}',
            markersize=5,
            linewidth=3
        )
    plt.plot(a3_202_L3_px.x, a3_202_L3_px.y, 'r*', label='Experiment-x', markersize=5)
    plt.plot(a3_202_L3_py.x, a3_202_L3_py.y, 'b*', label='Experiment-y', markersize=5)
    plt.plot(a3_202_L3_pz.x, a3_202_L3_pz.y, 'g*', label='Experiment-z', markersize=5)
    plt.xlabel('Time (days)')
    plt.ylabel('Axial Expansion (%)')
    plt.legend(frameon=False)
    plt.savefig("A3-202-L32_new.png", bbox_inches='tight')


    # # def grid_plot(df, nrow, ncol, file_name=None):
    #     """Create grid of plots."""
    #     # df_grouped = df.groupby(['specimen'])
    #     fig, subps = plt.subplots(2, 2)
    #     subps = list(it.chain(*subps))
    #
    #     for spec, datum in df_grouped:
    #         ax = subps[0]
    #         plot_dat = datum.groupby(['axis'])
    #         for axis, data in plot_dat:
    #             ax.plot(data.x, data.y, label=axis)
    #             ax.set_title(spec)
    #             ax.legend(frameon=False)
    #             # ax.set_xticks(np.arange(0, 600, step=100))
    #             # ax.set_yticks(np.arange(0, 1.40, step=0.2))
    #             # ax.set_xlim(0, 600, 100)
    #             # ax.set_ylim(0, 1.40, 0.2)
    #             # Expecting mostly (2,2) or (3,2) subplots
    #             if len(subps) % 2 == 0:
    #                 ax.set_ylabel('Axial Expansion [%]')
    #             if len(subps) <= 2:
    #                 ax.set_xlabel('Specimen Age [days]')
    #         subps.pop(0)
    #
    #     fig.subplots_adjust(hspace=.5)
    #     if nrow * ncol <= 4:
    #         fig.set_size_inches(8, 6)
    #     else:
    #         fig.set_size_inches(10, 10)
    #     if file_name is None:
    #         plt.show()
    #     else:
    #         plt.savefig(file_name, dpi=300)




if __name__ == "__main__":
    main()
