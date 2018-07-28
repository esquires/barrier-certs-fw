import argparse
import xml.etree.ElementTree as ET
import os.path as op
import subprocess
import shutil

import numpy as np


def indent(elem, level=0):
    # from https://stackoverflow.com/a/33956544
    i = "\n" + level*"  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
        for e in elem:
            indent(e, level+1)
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i


def create_node(s, t):
    el = ET.Element(s)
    try:
        float(t)
        el.text = "{:.2f}".format(t)
    except ValueError:
        el.text = str(t)
    return el


def setup(args, heading_angle, offset_angle, calc_type, sigma,
          sensing_range, start_z, dz, is_3d,
          centralized, end_time, is_nav_func, heading_randomization):
    tree = ET.parse(args.mission_file)
    root = tree.getroot()

    run_node = root.find('run')
    run_node.attrib['end'] = str(end_time)
    entity_common_node = root.find('entity_common')
    for controller_node in entity_common_node.findall('controller'):
        entity_common_node.remove(controller_node)
    if is_nav_func:
        nd = create_node("controller", "NavigationFunction")
        nd.attrib['param_common'] = "params"
        entity_common_node.append(nd)
    else:
        nd = create_node("controller", "UnicycleControllerPoint")
        nd.attrib['param_common'] = "params"
        # allow nominal controller outside of actuation bounds
        nd.attrib['max_vel'] = "50"  
        entity_common_node.append(nd)

        nd = create_node("controller", "UnicycleControllerBarrier")
        nd.attrib['param_common'] = "params"
        entity_common_node.append(nd)

    motion_node = entity_common_node.find('motion_model')

    motion_node.text = "Unicycle"

    param_common_node = root.find('param_common')
    param_common_node.find('centralized').text = str(centralized)
    if sensing_range is not None:
        param_common_node.find('sensing_range').text = str(sensing_range)
    param_common_node.find('calc_type').text = calc_type
    param_common_node.find('heading_angle').text = str(heading_angle)
    param_common_node.find('sigma').text = str(sigma)
    param_common_node.find('is_3d').text = str(is_3d)

    for entity_node in root.findall('entity'):
        root.remove(entity_node)

    angles = np.pi + np.linspace(0, 2 * np.pi, args.num_agents + 1)[:-1]

    for i, angle in enumerate(angles):
        e = create_node('entity', '')
        e.attrib['entity_common'] = 'fixed_wing'

        start_x = args.start_radius * np.cos(angle + offset_angle)
        start_y = args.start_radius * np.sin(angle + offset_angle)
        z = start_z + i * dz if args.start_z else 0

        end_x = -args.end_radius * np.cos(angle)
        end_y = -args.end_radius * np.sin(angle)

        e.append(create_node('x', start_x))
        e.append(create_node('y', start_y))
        e.append(create_node('z', z))
        heading = (np.rad2deg(heading_angle + angle + np.pi) % 360) + \
            np.random.uniform(-heading_randomization, heading_randomization)
        e.append(create_node('heading', heading))
        autonomy_node = create_node('autonomy', "Wpt")
        autonomy_node.attrib['exit_on_reaching_wpt'] = 'false'
        autonomy_node.attrib['draw_wpt'] = 'false'
        autonomy_node.attrib['gain'] = '1'
        autonomy_node.attrib['wpt'] = "{},{},{}".format(end_x, end_y, 0)
        e.append(autonomy_node)

        root.append(e)

    indent(root)
    tree.write(args.mission_file)

    log_node = root.find('log_dir')
    return log_node.text


def do_run(args, heading_angle, offset_angle,
           calc_type, sigma, sensing_range, start_z, dz, is_3d,
           centralized, end_time, is_nav_func, heading_randomization):
    d = setup(args, heading_angle, offset_angle,
        calc_type, sigma, sensing_range, start_z, dz, is_3d, centralized,
        end_time, is_nav_func, heading_randomization)

    centralized_suffix = "cent" if centralized else "dec"
    if args.out_dir == "":
        args.out_dir = d

    if args.run:
        subprocess.call(['scrimmage', args.mission_file])
        src = op.expanduser(op.join(d, 'latest'))
        dst = op.join(args.out_dir, args.veh_types)
        if sensing_range is not None and sensing_range >= 0:
            dst += '_sensing_{0:.5f}'.format(sensing_range)
        dst += '_' + calc_type + "_" + centralized_suffix
        if heading_angle != 0:
            dst += str(int(heading_angle))

        if heading_randomization:
            dst += f"_rand_heading_{heading_randomization:.0f}"
        shutil.copytree(src, dst, symlinks=False)

def main():
    parser = argparse.ArgumentParser()
    add = parser.add_argument
    add('mission_file', help='the mission file')
    add('num_agents', type=int, help='how many agents to use')
    add('out_dir', help='where to put the scenario')
    add('--heading_angle', type=float, default=0,
        help=('normally vehicles point at the origin. '
              'This is an offset added to that angle (degrees)'
              'ignored when calc_type == "all"'))
    add('--offset_angle', type=float, default=0,
        help=('This is a position offset (degrees)'))
    add('--start_radius', type=float, default=10,
        help='how far from the origin to start')
    add('--end_radius', type=float, default=10,
        help='how far from the origin to end')
    add('--calc_type', default='turn',
        help='turn, straight, all, or sensing')
    add('--start_z', type=float, help='starting z position')
    add('--dz', type=float, help='how much to change z for each vehicle')
    add('--is_3d', action='store_true',
        help='whether to add z component to dynamics')
    add('--run', action='store_true', help='run the scenario')
    add('--veh_types', help='both, fw, si', default='fw')
    add('--sigma', default=1, type=float)
    add('--sensing_range', type=float, default=-1)
    add('--centralized', action='store_true')
    add('--end_time', type=float)
    add('--add_nav_func', action='store_true')
    add('--heading_randomization', default=0.0, type=float)

    args = parser.parse_args()

    heading_ang = np.deg2rad(args.heading_angle)
    offset_ang = np.deg2rad(args.offset_angle)

    if args.calc_type == 'all':
        for heading_angle, calc_type in \
                zip([0, args.heading_angle], ['turn', 'straight']):
            ang = np.deg2rad(heading_angle)
            do_run(args, ang, offset_ang, calc_type, args.sigma, args.sensing_range,
                   args.start_z, args.dz, args.is_3d, args.centralized, args.end_time,
                   False, args.heading_randomization)

        if args.add_nav_func:
            ang = np.deg2rad(0)
            do_run(args, ang, offset_ang, "nav_func", args.sigma, args.sensing_range,
                   args.start_z, args.dz, args.is_3d, args.centralized, args.end_time,
                   True, args.heading_randomization)
    elif args.calc_type == 'sensing':
        sigma = 1
        v_min = 15
        v_max = 25
        pct_offset = 0.1
        turn_rate_max = np.deg2rad(13)
        v = v_min + pct_offset * (v_max - v_min)
        w = (1 - pct_offset) * turn_rate_max
        r = v / w
        Ds = 5
        delta = 0.01

        old_R = ((4 * r)**2 + Ds**2 + 2 * delta)**0.5
        R = 2 * (r + r) + (Ds**2 - 4*delta)**0.5
        print('min R > ', R, old_R)
        R += 0.25
        # max_R = 314
        # R = 350
        print('r = ', r)

        # R above is the threshold but equality can't hold
        # R = -1
        for sensing_range in np.linspace(R, R + 10, 25):
            print('running sensing range ' + str(sensing_range))
            do_run(args, 0, offset_ang, 'turn', sigma, sensing_range,
                   args.start_z, args.dz, args.is_3d, args.centralized, args.end_time,
                   False, args.heading_randomization)

        # make sure 1.5 and 2 are there
        # do_run(args, 0, 'turn', 1.0)
        # do_run(args, 0, 'turn', 1.25)
        # do_run(args, 0, 'turn', 1.5)
    else:
        do_run(args, heading_ang, offset_ang, args.calc_type, args.sigma,
               args.sensing_range,
               args.start_z, args.dz, args.is_3d, args.centralized, args.end_time,
               False, args.heading_randomization)


if __name__ == '__main__':
    main()
