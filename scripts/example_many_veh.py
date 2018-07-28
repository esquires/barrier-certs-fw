import numpy as np

def main():
    Ds = 0.5
    r = 1
    f = 2 * r + Ds
    psi = np.arccos((Ds / 2 + 2*r) / (2 * r + Ds))
    s = np.sin(psi)
    c = np.cos(psi)

    print('x2 = ', [f * s, f * c - 2 * r, 180])
    print('x3 = ', [f * s, 2 * r - f * c, 180])

if __name__ == '__main__':
    main()
