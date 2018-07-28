import argparse
import shutil
import xml.etree.ElementTree as ET
import os.path as op
import subprocess as sp
from pathlib import Path
from typing import List


def get_run_out_dir(name: str) -> str:
    return f'/tmp/{name}'


def call_plot(name: str, plot_out_dir: str, disable: List[str]) -> None:
    # WIDTH = 3.4
    # HEIGHT = 1.3

    # paper dimensions
    WIDTH = 3.8
    HEIGHT = 1.6

    # thesis dimensions
    WIDTH = 5.5
    HEIGHT = 2.2

    veh_id = 1
    prefix = ''
    sensing = False

    if name == '2_veh_alt':
        prefix = 'alt'
    elif name == '2_veh_finite_sensing':
        sensing = True

    run_out_dir = get_run_out_dir(name)

    args = [
        'python', 'plotter.py', run_out_dir, plot_out_dir,
        '--veh_id', str(veh_id), '--width', str(WIDTH),
        '--height', str(HEIGHT)
    ]

    if disable:
        args += ['--disable'] + disable

    if prefix:
        args.append('--prefix')
        args.append(prefix)

    if sensing:
        args.append('--sensing')

    if plot_out_dir:
        args.append('--save_figs')

    sp.check_call(args)


def call_run(name: str, do_run: bool) -> None:
    # provide common defaults
    kwargs = {
        'radius': 200,
        'calc_type': 'all',
        'mission': '../missions/barrier_certs.xml',
        'sigma': 1.0,
        'heading_angle': 2,
        'start_z': 0,
        'dz': 0,
        'num_veh': 2,
        'is_3d': True,
        'sensing_range': -1,
        'centralized': False,
        'end_time': 20,
        'add_nav_func': False,
        'heading_randomization': False,
    }

    if name == '2_veh_alt':
        kwargs['start_z'] = -1
        kwargs['dz'] = 2
        kwargs['centralized'] = True

    elif name == '2_veh_inf_sensing':
        kwargs['centralized'] = True

    elif name == '20_veh_inf_sensing' \
            or name == '20_veh_inf_sensing_rand_heading':
        kwargs['decentralized'] = True
        kwargs['heading_angle'] = 25
        kwargs['num_veh'] = 20
        kwargs['add_nav_func'] = True

        if name == '20_veh_inf_sensing_rand_heading':
            kwargs['heading_randomization'] = 10

    elif name == '2_veh_finite_sensing':
        kwargs['calc_type'] = 'sensing'

    elif name == '20_veh_finite_sensing':
        kwargs['radius'] = 1500
        kwargs['calc_type'] = 'turn'
        kwargs['heading_angle'] = 0
        kwargs['num_veh'] = 20
        kwargs['sensing_range'] = 350
        kwargs['end_time'] = 120

    out_dir = get_run_out_dir(name)
    args = [
        'python', 'run.py',
        kwargs['mission'], str(kwargs['num_veh']), out_dir,
        '--start_radius', str(kwargs['radius']),
        '--end_radius', str(kwargs['radius']),
        '--calc_type', kwargs['calc_type'],
        '--sigma', str(kwargs['sigma']),
        '--heading_angle', str(kwargs['heading_angle']),
        '--start_z', str(kwargs['start_z']),
        '--dz', str(kwargs['dz']),
        '--end_time', str(kwargs['end_time']),
    ]

    if kwargs['is_3d']:
        args.append('--is_3d')

    if kwargs['centralized']:
        args.append('--centralized')

    if kwargs['add_nav_func']:
        args.append('--add_nav_func')

    if kwargs['heading_randomization']:
        args.append('--heading_randomization')
        args.append(str(kwargs['heading_randomization']))

    if do_run:
        args.append('--run')

    shutil.rmtree(out_dir, ignore_errors=True)
    sp.check_call(args)


def mfcbf_exp(plot_dir: str) -> None:

    repo_root = Path(__file__).resolve().parent.parent
    mission = repo_root / 'missions' / 'barrier_certs_start_on_right.xml'
    out_dir = '/tmp/mfcbf'

    def _run(_calc_type: str) -> None:
        tree = ET.parse(mission)
        root = tree.getroot()
        param_common = root.find('param_common')
        calc_type_node = param_common.find('calc_type')

        run_node = root.find('run')
        run_node.attrib['end'] = '20' if _calc_type == 'straight' else '30'
        calc_type_node.text = _calc_type
        log_dir = root.find('log_dir').text
        tree.write(mission)

        sp.check_call(['scrimmage', mission])
        src = op.expanduser(op.join(log_dir, 'latest'))
        dst = op.join(out_dir, f'fw_{_calc_type}_dec')
        shutil.copytree(src, dst, symlinks=False)

    _run('turn')
    _run('straight')

    call_plot('mfcbf', plot_dir, ['actuator', 'zpath', 'min_d'])


def main() -> None:
    parser = argparse.ArgumentParser()

    aiaa_paper_choices = [
        '2_veh_alt', '2_veh_inf_sensing', '20_veh_inf_sensing',
        ]
    icra_paper_choices = [
       '2_veh_finite_sensing', '20_veh_finite_sensing']
    choices = \
        ['aiaa', 'icra', 'mfcbf', '2_veh',
         '20_veh_inf_sensing_rand_heading'] + \
        aiaa_paper_choices + icra_paper_choices
    parser.add_argument('name', choices=choices)
    parser.add_argument('plot_out_dir', type=str)
    parser.add_argument('--no_run', action='store_true')
    parser.add_argument('--no_plot', action='store_true')
    args = parser.parse_args()

    if args.name == 'aiaa':
        names = aiaa_paper_choices
    elif args.name == 'icra':
        names = icra_paper_choices
    elif args.name == 'mfcbf':
        mfcbf_exp(args.plot_out_dir)
        return
    else:
        names = [args.name]

    for name in names:
        plot = not args.no_plot
        run = not args.no_run
        if plot:
            assert run, "set run to true to enable plots"

        print(f'performing {name} experiment')
        call_run(name, run)
        if plot:
            call_plot(name, args.plot_out_dir, [])


if __name__ == '__main__':
    main()
