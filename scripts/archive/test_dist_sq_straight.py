import numpy as np
import matplotlib.pyplot as plt


def dist_sq_fw_fw_straight(xx: np.ndarray, v1: float, v2: float) -> float:
    x10, y10, th10, z10, x20, y20, th20, z20 = xx

    db = x10 - x20
    dc = y10 - y20

    C = v1 * np.cos(th10) - v2 * np.cos(th20)
    S = v1 * np.sin(th10) - v2 * np.sin(th20)

    c = pow(db, 2) + pow(dc, 2)
    b = 2 * (db * C + dc * S)
    a = pow(C, 2) + pow(S, 2) + pow(z10 - z20, 2)

    min_t = max(0.0, -b / (2 * a))
    # print(x20, min_t)
    dist_squared = c + b * min_t + a * pow(min_t, 2)
    return dist_squared


def main():

    Ds = 5
    x10 = 5
    x20 = 0
    v1 = 15
    v2 = 16
    th10 = 0
    th20 = 0
    y10 = 0
    y20 = Ds
    z10 = 0
    z20 = 0

    xx = np.array([x10, y10, th10, z10, x20, y20, th20, z20], dtype=np.float32)

    n = 1000
    data = np.empty((n, 2))

    for i, x20 in enumerate(np.linspace(-0.0, 10.0, n)):
        # print(x20)
        data[i, 0] = x20
        xx[4] = x20
        data[i, 1] = dist_sq_fw_fw_straight(xx, v1, v2) - Ds**2

    plt.plot(data[:, 0], data[:, 1])
    plt.show()


if __name__ == '__main__':
    main()
