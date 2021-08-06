"""Displays 3D orientation being printed off of debug port"""

from telnetlib import Telnet
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


def main():
    t = Telnet("localhost", 2333)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    while True:
        if (result := t.read_until(b"\n")) :
            try:
                z, a, b = result.decode("utf-8").split("\t")
            except ValueError:
                continue

            soa = np.array([[0, 0, 0] + list(map(float, a.split(" ")))])
            ax.set_xlim([-1, 1])
            ax.set_ylim([-1, 1])
            ax.set_zlim([-1, 1])
            ax.quiver(*zip(*soa))
            plt.draw()
            plt.pause(0.001)
            ax.clear()


if __name__ == "__main__":
    main()
