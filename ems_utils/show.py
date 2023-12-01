from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import random
import matplotlib.pyplot as plt
import matplotlib


def cuboid_data2(o, size=(1, 1, 1)):
    X = [[[0, 1, 0], [0, 0, 0], [1, 0, 0], [1, 1, 0]],
         [[0, 0, 0], [0, 0, 1], [1, 0, 1], [1, 0, 0]],
         [[1, 0, 1], [1, 0, 0], [1, 1, 0], [1, 1, 1]],
         [[0, 0, 1], [0, 0, 0], [0, 1, 0], [0, 1, 1]],
         [[0, 1, 0], [0, 1, 1], [1, 1, 1], [1, 1, 0]],
         [[0, 1, 1], [0, 0, 1], [1, 0, 1], [1, 1, 1]]]
    X = np.array(X).astype(float)
    for i in range(3):
        X[:, :, i] *= size[i]
    X += np.array(o)
    return X


mpl_colors = matplotlib.colors.cnames
mpl_colors = list(mpl_colors.keys())


def get_color():
    return random.choice(mpl_colors)


class CubePlot:
    def __init__(self, size):
        self.size = size
        self.fig = plt.figure()
        #self.ax = self.fig.gca(projection='3d')
        self.ax = self.fig.add_subplot(projection='3d')
        self.ax.set_xlim([0, size[0]])
        self.ax.set_ylim([0, size[1]])
        self.ax.set_zlim([0, size[2]])
        plt.pause(0.1)

    def update(self, size, location, offset=None, color=None, alpha=1, delay=0.1):  #0.1
        if offset is not None:
            location[0] += offset
            location[1] += offset
            size[0] -= offset * 2
            size[1] -= offset * 2
        pc = self.plotCube(size, location, color, alpha)
        self.ax.add_collection3d(pc)
        plt.ion()
        plt.pause(delay)
        return pc

    @staticmethod
    def remove(pc: Poly3DCollection):
        pc.set_visible(False)

    def plotCube(self, size, location, color=None, alpha=1):
        raw_data = cuboid_data2(location, size)
        if color is None:
            color = np.repeat(get_color(), 6)
        else:
            color = np.repeat(color, 6)
        return Poly3DCollection(np.concatenate([raw_data]), facecolors=color, edgecolor="black", alpha=alpha)

    def clear(self):
        self.ax.cla()
        self.ax.set_xlim([0, self.size[0]])
        self.ax.set_ylim([0, self.size[1]])
        self.ax.set_zlim([0, self.size[2]])
        plt.pause(0.1)


if __name__ == '__main__':
    positions = [(0, 0, 0), (5, 5, 5)]
    sizes = [(5, 5, 5), (5, 5, 5)]
    ploter = CubePlot([10, 10, 10])
    ploter.update(sizes[0], positions[0])
    ploter.update(sizes[1], positions[1])





