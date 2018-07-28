import numpy as np
import matplotlib.pyplot as plt
import lvdb

def main():
    datas = []
    for i, files in enumerate(['temp.csv', 'temp2.csv']):
        data = np.loadtxt(files, delimiter=",", skiprows=1)
        datas.append(data)
        row = np.argmin(data[:, 0])
        plt.plot(data[:, 1], data[:, 2], 'b' if i == 1 else 'b--')
        plt.plot(data[:, 3], data[:, 4], 'b' if i == 1 else 'b--', label=i)
        plt.plot(data[row, 1], data[row, 2], 'ro')
        plt.plot(data[row, 3], data[row, 4], 'ro')
        plt.plot(data[0, 1], data[0, 2], 'ko')
        plt.plot(data[0, 3], data[0, 4], 'ko')
        plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    main()
