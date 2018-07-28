import argparse
import os
import os.path as op
import glob
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import pandas as pd
import scrimmage.bindings as sc

parser = argparse.ArgumentParser()
parser.add_argument("base_dir", nargs='+')
parser.add_argument("out_dir")
parser.add_argument("--sensing", action="store_true")
parser.add_argument("--veh_id", type=int, default=1)
parser.add_argument("--width", type=float, default=6)
parser.add_argument("--height", type=float, default=4)
parser.add_argument("--save_figs", action="store_true")
parser.add_argument("--legend", action="store_true")
parser.add_argument("--prefix", default="",
                    help="what to add before filenames")
parser.add_argument(
    "--disable", nargs='*', choices=['actuator', 'path', 'zpath', 'min_d'])
args = parser.parse_args()
if not args.disable:
    args.disable = []

if args.save_figs:
    import texfig

import matplotlib.pyplot as plt

plt.rcParams['mathtext.fontset'] = 'dejavuserif'

MARKERS = ['--', '-', ':']

def _get_dirs(base_dirs):
    out = []
    for d in base_dirs:
        dirs = glob.glob(os.path.join(d, '*/'))
        dirs = [d.rstrip(os.path.sep) for d in dirs]
        dirs.sort()

        if not dirs:
            dirs = [d]
        out += dirs

    # a hack to make the colors consistent on the plots
    # In some plots the navigation function is not plotted
    # whereas in others it is so make sure it is put at the back
    if 'fw_nav_func_dec' in Path(out[0]).name:
        out.append(out[0])
        out.pop(0)
    return out


def _gen_fig(save_figs, width, height):
    if save_figs:
        import texfig
        fig = texfig.figure(width, height / width)
        return fig.gca()
    else:
        plt.figure(figsize=(width, height))
        return plt.gca()


def get_label(d):
    tree = ET.parse(op.join(d, 'mission.xml'))
    root = tree.getroot()
    param_common_node = root.find('param_common')
    calc_type = param_common_node.find('calc_type').text
    # heading_angle = \
    #     np.rad2deg(float(param_common_node.find('heading_angle').text))

    return r"$\gamma_{{{}}}$".format(calc_type.replace('_', ' '))


def plot_actuator(
        dirs, ax, veh_id, actuator_idx, enable_plot_lims,
        min_t=-np.inf, max_t=np.inf, MARKERS=MARKERS):

    is_turn_rate = actuator_idx == 1
    is_alt_rate = actuator_idx == 2
    if actuator_idx == 0:
        actuator_label = 'velocity'
    elif actuator_idx == 1:
        actuator_label = 'turn rate'
    elif actuator_idx == 2:
        actuator_label = ""
    elif actuator_idx == 3:
        actuator_label = r'$\epsilon$'

    data = []
    headers = ['t']

    dfs = []
    for mark, d in zip(MARKERS, dirs):
        if len(dirs) > 1:
            label = get_label(d)
        else:
            label = actuator_label
        headers.append(label)
        df = pd.read_pickle(op.join(d, 'data_controls.pickle'))
        df = df[(df['veh_id'] == veh_id) & (df['idx'] == actuator_idx)]
        df = df[(min_t <= df['t']) & (df['t'] <= max_t)]
        if is_turn_rate:
            df['u_safe'] = np.rad2deg(df['u_safe'])
        ax.plot(df['t'], df['u_safe'], mark, label=label)
        data.append(df['u_safe'].values.tolist())
        dfs.append(df)

    data.insert(0, df['t'].values.tolist())
    data = np.vstack(data).T

    if is_turn_rate:
        lims = (-13, 13)
        label = r'$\omega$ (deg/sec)'
    elif is_alt_rate:
        lims = (-3.9, 3.9)
        label = r'$\zeta$ (m/s)'
    else:
        lims = (15, 25)
        label = r'$v$ (m/s)'

    if enable_plot_lims:
        ones = np.ones(df['t'].shape[0])
        ax.plot(df['t'], ones * lims[0], 'k-')
        ax.plot(df['t'], ones * lims[1], 'k-')

    ax.set_ylabel(label)
    ax.set_xlabel(r'time (seconds)')

    return data, ','.join(headers)


def plot_min_dist(dirs, ax):

    des = 'distance'

    for mark, d in zip(MARKERS, dirs):
        if len(dirs) > 1:
            label = get_label(d)
        else:
            label = des
        df_safety = pd.read_pickle(op.join(d, 'data_safety.pickle'))
        min_d = df_safety['min_d'].groupby(df_safety['t']).min()
        ax.plot(min_d.index, min_d.values, mark, label=label)

    # assume they all have the same Ds and use the time axis from the last
    Ds = pd.read_csv(op.join(dirs[0], 'Ds.csv')).values[0][0]
    ax.plot(min_d.index, Ds * np.ones(len(min_d)), 'k-',
            label=r'$D_s = {}$'.format(Ds))
    ax.set_ylabel(r'vehicle dist (m)')
    ax.set_xlabel(r'time (seconds)')


def plot_paths(dirs, ax, veh_id):
    for mark, d in zip(MARKERS, dirs):
        label = get_label(d)
        df = sc.frames2pandas(os.path.join(d, 'frames.bin'))
        df = df[df['id'] == veh_id]
        ax.plot(df['x'], df['y'], mark, label=label)

    lims = ax.get_ylim()
    if abs(lims[0]) < 1e-5 and abs(lims[1]) < 1e-5:
        ax.set_ylim((-5, 5))

    ax.set_ylabel(r'$y$ (meters)')
    ax.set_xlabel(r'$x$ (meters)')


def plot_z_path(dirs, ax, veh_id):
    for mark, d in zip(MARKERS, dirs):
        label = get_label(d)
        df = sc.frames2pandas(os.path.join(d, 'frames.bin'))
        df = df[df['id'] == veh_id]
        ax.plot(df['time'], df['z'], mark, label=label)

    ax.set_ylabel(r'$z$ (meters)')
    ax.set_xlabel(r'$x$ (meters)')


def render(save_figs, fname, data = None, header = "None"):
    if save_figs:
        import texfig
        d, f = op.split(fname)
        out = args.prefix + "_" + f if args.prefix else f
        fname = op.join(d, out)
        texfig.savefig(fname)

        if data is not None:
            np.savetxt(fname + '.csv', data, delimiter=',', header=header)

        # not sure why negative numbers are not coming through on the graph
        # but this seems to fix it


    else:
        plt.show()

def plot_non_sensing(args):

    dirs = _get_dirs(args.base_dir)

    num_veh = len(pd.read_pickle(
       dirs[0] + '/data_controls.pickle')['veh_id'].unique())
    if not 'actuator' in args.disable:

        headers = ",".join(['t'] + dirs)
        print('generating actuator plots')
        ax_v = _gen_fig(args.save_figs, args.width, args.height)
        plot_actuator(dirs, ax_v, args.veh_id, 0, True)
        # ax_v.legend()
        fname = op.join(args.out_dir, '{}_veh{}_actuator{}'.format(
            num_veh, args.veh_id, 0))
        if args.legend:
            ax_v.legend()
        render(args.save_figs, fname)

        ax_w = _gen_fig(args.save_figs, args.width, args.height)
        plot_actuator(dirs, ax_w, args.veh_id, 1, True)
        fname = op.join(args.out_dir, '{}_veh{}_actuator{}'.format(
            num_veh, args.veh_id, 1))
        if args.legend:
            ax_w.legend()
        render(args.save_figs, fname)

        # ax_w_cropped = _gen_fig(args.save_figs, args.width, args.height)
        # plot_actuator(dirs, ax_w_cropped, args.veh_id, 1, 8, 9.5)
        # # ax_w_cropped.legend(loc='upper right')
        # fname = op.join(args.out_dir, '{}_veh{}_actuator{}_cropped'.format(
        #     num_veh, args.veh_id, 1))
        # render(args.save_figs, fname)

        ax_alt = _gen_fig(args.save_figs, args.width, args.height)
        data, header = plot_actuator(dirs, ax_alt, args.veh_id, 2, True)
        fname = op.join(args.out_dir, '{}_veh{}_actuator{}'.format(
            num_veh, args.veh_id, 2))
        # ax_alt.legend()
        render(args.save_figs, fname, data, header)

        # show that the dz control is continuous by zooming in on the figure
        ax_alt = _gen_fig(args.save_figs, args.width, args.height)
        plot_actuator(dirs, ax_alt, args.veh_id, 2, False,
                      min_t=8.1, max_t=8.25, MARKERS=['o', 'x', ':'])
        fname = op.join(args.out_dir, '{}_veh{}_actuator{}_zoom_in'.format(
            num_veh, args.veh_id, 2))
        if args.legend:
            ax_alt.legend()
        render(args.save_figs, fname)

    if not 'path' in args.disable:
        print('generating path plot')
        ax_paths = _gen_fig(args.save_figs, args.width, args.height)
        plot_paths(dirs, ax_paths, args.veh_id)
        fname = op.join(args.out_dir, '{}_veh{}_path'.format(
            num_veh, args.veh_id))
        if args.legend:
            ax_paths.legend()
        render(args.save_figs, fname)

    if not 'zpath' in args.disable:
        print('generating z path plot')
        ax_paths = _gen_fig(args.save_figs, args.width, args.height)
        plot_z_path(dirs, ax_paths, args.veh_id)

    if not 'min_d' in args.disable:
        print('generating min_d plot')
        ax_d = _gen_fig(args.save_figs, args.width, args.height)
        plot_min_dist(dirs, ax_d)
        # ax_d.legend(loc='upper right')
        fname = op.join(args.out_dir, '{}_veh_min_d'.format(num_veh))
        if args.legend:
            ax_d.legend()
        render(args.save_figs, fname)


def plot_sensing(args):
    dirs = _get_dirs(args.base_dir)

    def _get_type(d):
        tree = ET.parse(op.join(d, 'mission.xml'))
        root = tree.getroot()
        ent_nodes = root.findall('entity')
        has_fw = any((n for n in ent_nodes
                      if n.attrib['entity_common'] == 'fixed_wing'))
        has_si = any((n for n in ent_nodes
                      if n.attrib['entity_common'] == 'single_integrator'))
        return 'both' if has_fw and has_si else 'si' if has_si else 'fw'

    def _subdirs(_type):
        return [d for d in dirs if _get_type(d) == _type]

    def _get_min_dist(d):
        return pd.read_pickle(op.join(d, 'data_safety.pickle'))['min_d'].min()

    def _get_sensing_range(d):
        tree = ET.parse(op.join(d, 'mission.xml'))
        root = tree.getroot()
        return float(root.find('param_common').find('sensing_range').text)

    fw_dirs = _subdirs('fw')
    ax = _gen_fig(args.save_figs, args.width, args.height)

    data = np.array(
        [(_get_sensing_range(d), _get_min_dist(d)) for d in fw_dirs])
    ax.plot(data[:, 0], data[:, 1], MARKERS[0], label='Min Vehicle Dist')

    Ds = pd.read_csv(op.join(fw_dirs[-1], 'Ds.csv')).values[0][0]

    ax.plot(data[:, 0], Ds * np.ones(len(data[:, 0])), 'k-',
            label=r'$D_s = {}$'.format(Ds))
    ax.set_xlabel('$R$ (meters)')
    ax.set_ylabel('vehicle dist\n(meters)')

    min_R = 318.40880941045106
    min_val = Ds
    max_val = ax.get_ylim()[1]
    ax.plot([min_R, min_R], [min_val, max_val], ':g', label=r'Min Allowed $R$')

    ax.legend(loc='right', labelspacing=0.0)
    fname = op.join(args.out_dir, 'sensing')
    with open(op.join(args.out_dir, 'sensing_data_scalars.csv'), 'w') as f:
        f.write(f'min_R,Ds,max_val\n{min_R},{Ds},{max_val}')
    np.savetxt(op.join(args.out_dir, 'sensing_data.csv'), data,
               delimiter=',', header='R,min_dist', comments='')

    render(args.save_figs, fname)


def main() -> None:
    """Plot the following

    * actuator as a function of time
    * min dist as a function of time
    * path in x/y plane
    * sensing distance
    """
    if args.sensing:
        plot_sensing(args)
    else:
        plot_non_sensing(args)
    plt.close("all")


if __name__ == '__main__':
    main()
