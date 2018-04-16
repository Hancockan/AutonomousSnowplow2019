import numpy as np
from lidar_serial import lidar_handler, fake_lidar_handler
from math import cos, sin
import matplotlib.pyplot as plt
import random


class Node:
    def __init__(self, x, y):
        x = round(x, 2)
        y = round(y, 2)
        self.neighbors = []
        self.x = x
        self.y = y
        self.hits = 0

    def __str__(self):
        return "Point: " + str(self.x) + ", " + str(self.y)


def round_to(n, precision):
    correction = 0.5 if n >= 0 else -0.5
    return int(n / precision + correction) * precision


def polar_to_world(x, y, lidar_theta, scan_theta, scan_radius):
    # print(str(x) + " " + str(y) + " " + str(lidar_theta) + " " + str(scan_theta) + " " + str(scan_radius))

    local_trans_x = []
    local_trans_y = []
    for t, r in list(zip(scan_theta, scan_radius)):
        local_trans_x.append(round_to((x + (r * cos(lidar_theta + t))), .05))
        local_trans_y.append(round_to((y + (r * sin(lidar_theta + t))), .05))
    return local_trans_x, local_trans_y


def getPose():
    lidarX = float(input("Enter Lidar X location: "))
    lidarY = float(input("Enter Lidar Y location: "))
    lidarTheta = float(input("Enter Lidar Theta (CCW is positive): "))
    lidarTheta = (lidarTheta - 90) * np.pi / 180.0
    return lidarX, lidarY, lidarTheta


def getFakePose():
    fakeX = random.uniform(-4, 4)
    fakeY = random.uniform(-4, 4)
    fakeTheta = random.uniform(0, 360) * np.pi / 180.0
    return fakeX, fakeY, fakeTheta


def createWorld(resolution, xwidth, ywidth):
    xx, yy = np.meshgrid(np.arange(-xwidth / 2, xwidth / 2, resolution), np.arange(-ywidth / 2, ywidth / 2, resolution))
    my_grid_list = list(zip(xx.flatten(), yy.flatten()))

    my_grid_list = [(round(x, 2), round(y, 2)) for x, y in my_grid_list]

    my_node_list = [Node(point[0], point[1]) for point in my_grid_list]
    # for node in my_node_list:
    #     #print("Node: "+str(node))
    #     # print("Neighbors: ")
    #     # Add the 8-connected neighbors
    #     for test_node in my_node_list:
    #         if test_node.x == node.x and (test_node.y == node.y + resolution or test_node.y == node.y - resolution):
    #             node.neighbors.append(test_node)
    #         if test_node.y == node.y and (test_node.x == node.x + resolution or test_node.x == node.x - resolution):
    #             node.neighbors.append(test_node)
    #         if (test_node.x == node.x - resolution or test_node.x == node.x + resolution) and (
    #                 test_node.y == node.y + resolution or test_node.y == node.y - resolution):
    # node.neighbors.append(test_node)
    # my_node_dict = dict(zip(my_grid_list, my_node_list))

    return my_grid_list, my_node_list


if __name__ == "__main__":
    grid, nodes = createWorld(0.05, 10, 10)
    node_dict = dict(zip(grid, nodes))

    l = lidar_handler()

    fig = plt.figure()
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])

    while True:
        xlid, ylid, theta = getPose()
        ax.set_ylim(-5, 5)
        ax.set_xlim(-5, 5)
        hitSizeList = []

        angle, radius = l.get_scan()
        # print(list(zip(angle, radius)))

        trans_x, trans_y = polar_to_world(xlid, ylid, theta, angle, radius)
        # print(list(zip(trans_x, trans_y)))
        for x, y in list(zip(trans_x, trans_y)):
            try:
                cur_node = node_dict[(x, y)]
                # print(cur_node)
                cur_node.hits += .1
                hitSizeList.append(cur_node.hits)
            except:
                print('too noisy')
        ax.scatter(trans_x, trans_y, s=3, c='b')

        plt.pause(4)
        # plt.show()
        # plt.cla()
