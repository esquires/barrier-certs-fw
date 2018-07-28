import numpy as np
import matplotlib.pyplot as plt
import lvdb


def dist_sq_fw_fw_turn(x, v, w, sigma, delta, first_is_lower_id):
    x10, y10, th10, z10, x20, y20, th20, z20 = x
    is_3d = True
    th = th10 if first_is_lower_id else th20
    r = v / w

    b10 = x10 - sigma * r * np.sin(th10)
    b20 = x20 - r * np.sin(th20)
    db = b10 - b20

    c10 = y10 + sigma * r * np.cos(th10)
    c20 = y20 + r * np.cos(th20)
    dc = c10 - c20

    dz = z10 - z20 if is_3d else 0

    a1 = pow(db, 2) + pow(dc, 2) + pow(dz, 2) + \
        (1 + pow(sigma, 2)) * pow(r, 2) - \
        2 * sigma * pow(r, 2) * np.cos(th10 - th20) - \
        2 * delta

    def add(func):
        return 2 * r * (
            db * (sigma * func(th10 - np.pi / 2) - func(th20 - np.pi / 2)) +
            dc * (-sigma * func(th10) + func(th20))) + \
            delta * func(th10 - np.pi / 2) - delta * func(th10)

    re = add(np.cos) + delta * np.cos(th)
    im = add(np.sin) + delta * np.sin(th)
    a2 = (re**2 + im**2)**0.5
    return a1 - a2


def init_vars(x, v, w, sigma, delta, first_is_lower_id):
    x10, y10, th10, x20, y20, th20 = x
    th = th10 if first_is_lower_id else th20

    r = v / w

    b10 = x10 - sigma * r * np.sin(th10)
    b20 = x20 - r * np.sin(th20)
    db = b10 - b20

    c10 = y10 + sigma * r * np.cos(th10)
    c20 = y20 + r * np.cos(th20)
    dc = c10 - c20

    sa = np.sin(th10)
    ca = np.cos(th10)

    sb = np.sin(th20)
    cb = np.cos(th20)

    return th20, th, r, db, dc, sa, ca, sb, cb


def interp(x1, x2, pct):
    return (1 - pct) * x1 + pct * x2

def show_nondifferentiability():
    n = 10000
    div = 1e1
    data = np.empty((n, 3))
    x1 = np.array([205.066, -151.5, -3.02451, -0.000584902,
                  216.135, -254.793, 2.13433, -0.00178073])
    x2 = np.array([204.942, -151.515, -3.0245, -0.000584414,
                   216.06, -254.666, 2.07353, -0.00177926])
    dx = x2 - x1

    v = 16
    w = 0.204
    sigma = 1
    delta = 0.01

    dist2 = dist_sq_fw_fw_turn(x1, v, w, sigma, delta, True)
    h1 = dist2**0.5 - 5

    dist2 = dist_sq_fw_fw_turn(x2, v, w, sigma, delta, True)
    h2 = dist2**0.5 - 5

    pcts = np.linspace(-10, 10, n)
    for i, p in enumerate(pcts):
        x = interp(x1, x2, p)
        data[i, 0] = p
        data[i, 1] = dist_sq_fw_fw_turn(x, v, w, sigma, delta, True)**0.5 - 5

    plt.plot(data[:, 0], data[:, 1], 'b', label='$\delta$ = ' + str(delta))
    plt.plot([0, 1], [h1, h2], 'bo', label='$\delta$ = ' + str(delta))
    plt.legend()
    plt.show()

def main():
    # calc()
    show_nondifferentiability()

if __name__ == '__main__':
    main()
