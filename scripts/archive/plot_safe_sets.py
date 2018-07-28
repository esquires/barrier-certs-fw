import numpy as np
import matplotlib.pyplot as plt

from test_dist_sq_fw_fw_turn import dist_sq_fw_fw_turn
from test_dist_sq_straight import dist_sq_fw_fw_straight


def main():
    w_max = np.deg2rad(13)
    v_max = 25
    v_min = 15
    Ds = 5

    sigma = 1
    delta = 0.01
    v = 0.9 * v_min + 0.1 * v_max
    w = 0.9 * w_max
    r = v / w
    v1 = v_min + 0.1 * (v_max - v_min)
    v2 = v_min + 0.2 * (v_max - v_min)

    def calc_data(_xx, _x20s, _y20s):
        h_straight = np.zeros_like(x20s)
        h_turn = np.zeros_like(x20s)

        for i in range(_x20s.shape[0]):
            for j in range(_x20s.shape[1]):
                xx[4] = x20s[i, j]
                xx[5] = y20s[i, j]

                temp = dist_sq_fw_fw_straight(xx, v1, v2)**0.5 - Ds
                h_straight[i, j] = 1 if temp >= 0 else 0

                temp = dist_sq_fw_fw_turn(xx, v, w, sigma, delta, True)
                if temp < 0:
                    # case where vehicles are too close
                    temp = 0
                h_turn[i, j] = 1 if temp**0.5 - Ds >= 0 else 0
                # h_turn[i, j] = temp**0.5 - Ds

        return h_straight, h_turn

    def make_plot(_ax, xx_data, yy_data, h_straight_data, h_turn_data, nm,
                  legend):
        turn_mask = h_turn_data.flatten() == 0
        straight_mask = h_straight_data.flatten() == 0
        xflat = xx_data.flatten()
        yflat = yy_data.flatten()

        _ax.scatter(xflat[~straight_mask], yflat[~straight_mask], c='w')
        _ax.scatter(xflat[~turn_mask], yflat[~turn_mask], c='w')
        _ax.scatter(xflat[straight_mask], yflat[straight_mask], label=r'$h_{straight}$')
        _ax.scatter(xflat[turn_mask], yflat[turn_mask], label=r'$h_{turn}$')

        # ax.set_xlim([-250, 400])
        # ax.set_ylim([-300, 350])
        # _ax.set_aspect('equal', 'box')
        _ax.axis('equal')
        if legend:
            _ax.legend(loc='lower right')
        plt.savefig(f'safe_set_{nm}.png', dpi=600)

    xx = np.zeros(8)

    n = 500
    x20s, y20s = np.meshgrid(
        np.linspace(-250, 400, n), np.linspace(-5*r, 5*r, n))

    figsize = (3, 3)
    print('working on left')
    xx[6] = np.pi
    h_straight, h_turn = calc_data(xx, x20s, y20s)
    ax_left = plt.subplots(1, 1, figsize=figsize)[1]
    make_plot(ax_left, x20s, y20s, h_straight, h_turn, 'left', True)

    print('working on right')
    xx[6] = 0
    h_straight, h_turn = calc_data(xx, x20s, y20s)
    ax_right = plt.subplots(1, 1, figsize=figsize)[1]
    make_plot(ax_right, x20s, y20s, h_straight, h_turn, 'right', False)

    print('working on up')
    xx[6] = np.pi / 2
    h_straight, h_turn = calc_data(xx, x20s, y20s)
    ax_up = plt.subplots(1, 1, figsize=figsize)[1]
    make_plot(ax_up, x20s, y20s, h_straight, h_turn, 'up', False)

    print('working on down')
    xx[6] = -np.pi / 2
    h_straight, h_turn = calc_data(xx, x20s, y20s)
    ax_down = plt.subplots(1, 1, figsize=figsize)[1]
    make_plot(ax_down, x20s, y20s, h_straight, h_turn, 'down', False)

    # cf = ax.contourf(x20s, y20s, h_turn, 1, colors=['tab:blue', 'white'])
    # cf = ax.contourf(x20s, y20s, h_straight, 1, colors=['tab:orange', 'white'])
    # cf = ax.contourf(x20s, y20s, h_turn, 100)
    # fig.colorbar(cf, ax=ax)
    # plt.show()
    


if __name__ == '__main__':
    main()
