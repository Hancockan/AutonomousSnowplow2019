import numpy as np
from lidar_serial import lidar_handler 
from math import cos, sin
import matplotlib.pyplot as plt

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

def polar_to_world(x, y, lidar_theta, scan_theta, scan_radius):
    
    #print(str(x) + " " + str(y) + " " + str(lidar_theta) + " " + str(scan_theta) + " " + str(scan_radius))

    local_trans_x = []
    local_trans_y = []
    for t, r in list(zip(scan_theta, scan_radius)):
        local_trans_x.append(round((x+(r*cos(lidar_theta + t))), 2))
        local_trans_y.append(round((y+(r*sin(lidar_theta + t))), 2))
    return local_trans_x, local_trans_y

def getPose():
    lidarX = int(input("Enter Lidar X location: "))
    lidarY = int(input("Enter Lidar Y location: "))
    lidarTheta = float(input("Enter Lidar Theta (CCW is positive): "))
    lidarTheta = lidarTheta * np.pi / 180.0
    return lidarX, lidarY, lidarTheta

def createWorld(resolution, xwidth, ywidth):
    xx, yy = np.meshgrid(np.arange(-xwidth/2, xwidth/2, resolution), np.arange(-ywidth/2, ywidth/2, resolution))
    my_grid_list = list(zip(xx.flatten(), yy.flatten()))
    
    my_grid_list = [(round(x, 2), round(y,2)) for x, y in my_grid_list]
    
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
                #node.neighbors.append(test_node)
    #my_node_dict = dict(zip(my_grid_list, my_node_list))

    return my_grid_list, my_node_list

if __name__ == "__main__":
    grid, nodes = createWorld(0.01, 9, 9)
    node_dict = dict(zip(grid, nodes))
   
    l = lidar_handler()

    fig = plt.figure()
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ax.set_ylim(-10, 10)
    ax.set_xlim(-10, 10)

    xlid, ylid, theta = getPose()

    while True:
        ax.set_ylim(-5, 5)
        ax.set_xlim(-5, 5)
        hitSizeList = []      

        angle, radius = l.get_scan()
        #print(list(zip(angle, radius)))

        trans_x, trans_y = polar_to_world(xlid, ylid, theta, angle, radius)
        #print(list(zip(trans_x, trans_y)))
        for x, y in list(zip(trans_x, trans_y)):
            try:
                cur_node = node_dict[(x, y)]
                # print(cur_node)
                cur_node.hits+=0.3
                hitSizeList.append(cur_node.hits)
            except:
                print('too noisy')
        ax.scatter(trans_x, trans_y, s = hitSizeList, c = 'b')
        
        plt.pause(0.0001)
        plt.cla()
