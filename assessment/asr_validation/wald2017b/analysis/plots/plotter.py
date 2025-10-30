#!/usr/bin/env python3

from glob import glob
import itertools as it
import re
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


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


def grid_plot(df, nrow, ncol, file_name=None):
    """Create grid of plots."""
    df_grouped = df.groupby(['specimen'])
    fig, subps = plt.subplots(nrow, ncol)
    subps = list(it.chain(*subps))

    for spec, datum in df_grouped:
        ax = subps[0]
        plot_dat = datum.groupby(['axis'])
        for axis, data in plot_dat:
            ax.plot(data.x, data.y, label=axis)
            ax.set_title(spec)
            ax.legend(frameon=False)
            # ax.set_xticks(np.arange(0, 600, step=100))
            # ax.set_yticks(np.arange(0, 1.40, step=0.2))
            # ax.set_xlim(0, 600, 100)
            # ax.set_ylim(0, 1.40, 0.2)
            # Expecting mostly (2,2) or (3,2) subplots
            if len(subps) % 2 == 0:
                ax.set_ylabel('Axial Expansion [%]')
            if len(subps) <= 2:
                ax.set_xlabel('Specimen Age [days]')
        subps.pop(0)

    fig.subplots_adjust(hspace=.5)
    if nrow * ncol <= 4:
        fig.set_size_inches(8, 6)
    else:
        fig.set_size_inches(10, 10)
    if file_name is None:
        plt.show()
    else:
        plt.savefig(file_name, dpi=300)


def main():
    """Script Driver Function."""
    figure04_fps = glob("./figure04/*.csv")
    figure04_dat = define_dfs(*figure04_fps)
    grid_plot(figure04_dat, 2, 2, "figure04.png")

    figure05_fps = glob("./figure05/*.csv")
    figure05_dat = define_dfs(*figure05_fps)
    grid_plot(figure05_dat, 3, 2, "figure05.png")

    figure06_fps = glob("./figure06/*.csv")
    figure06_dat = define_dfs(*figure06_fps)
    grid_plot(figure06_dat, 3, 2, "figure06.png")

    figure07_fps = glob("./figure07/*.csv")
    figure07_dat = define_dfs(*figure07_fps)
    grid_plot(figure07_dat, 2, 2, "figure07.png")




if __name__ == '__main__':
    main()
