import numpy as np
from lidar_serial import lidar_handler, fake_lidar_handler
from math import cos, sin, atan2, sqrt
import matplotlib.pyplot as plt
import random
import cv2
import random


class World:
    def __init__(self, worldDict, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.dict = worldDict
        self.img = np.zeros((0, 0, 3), np.uint8)

    def getHitsAtNode(self, x, y):
        hits = self.dict[(x, y)].getHits()
        return hits

    def objectAtNode(self, x, y):
        return self.dict[(x, y)].objectPresent()

    def getNode(self, x, y):
        return self.dict[(x, y)]

    def increaseNodeHits(self, x, y):
        self.dict[(x, y)].hits += 1
        return True

    def createImg(self):
        self.img = np.zeros((int(self.height / self.resolution), int(self.width / self.resolution), 3), np.uint8)

    def getImg(self):
        return self.img

    def metersToPixels(self, meter):
        return meter / self.resolution


class Node:
    def __init__(self, x, y):
        x = round(x, 2)
        y = round(y, 2)
        self.neighbors = []
        self.x = x
        self.y = y
        self.hits = 0
        self.objectPresent = False

    def objectPresent(self):
        return self.objectPresent()

    def getHits(self):
        return self.hits

    def __str__(self):
        return "Point: " + str(self.x) + ", " + str(self.y)


class VehicleSimulator:
    def __init__(self):
        self.width = 1.2
        self.length = 1.75
        self.xSpeed = 0
        self.ySpeed = 0
        self.x = 0
        self.y = 0
        self.heading = 90
        self.headingDesired = 0
        self.yawRateMax = 30
        self.time = 0
        self.maxSpeed = 2
        self.timeStep = .1
        self.xDesired = 0
        self.yDesired = 0
        self.thetaTol = 6
        self.distTol = .2

    def getPose(self):  # Returns pose of the robot
        return self.x, self.y, self.heading

    def distToNextPoint(self):
        return sqrt((self.yDesired - self.y) ** 2 + (self.xDesired - self.x) ** 2)

    def updateHeading(self):  # Called when heading needs to be updated if not pointing in correct direction
        self.getDesiredHeading()
        smallestDif = self.headingDesired - self.heading
        smallestDif = (smallestDif + 180) % 360 - 180
        sign = 0
        if (smallestDif < 0):
            sign = -1
        else:
            sign = 1
        self.heading = self.heading + self.timeStep * self.yawRateMax * sign

    def updateDesiredLocation(self, xNew, yNew):  # Call once when moving vehicle to a waypoint.
        self.xDesired = xNew
        self.yDesired = yNew

    def updateVehiclePosition(self):  # Uses speed to update position over a fixed time step
        self.updateSpeed()
        self.x = self.x + self.xSpeed * self.timeStep
        self.y = self.y + self.ySpeed * self.timeStep

    def updateSpeed(self):  # Sets the speed of the vehicle in the x and y directions as vectors
        self.xSpeed = self.maxSpeed * cos(degToRad(self.heading))
        self.ySpeed = self.maxSpeed * sin(degToRad(self.heading))

    def getDesiredHeading(self):  # Gives the desired heading based on current point and waypoint
        headingDesired = atan2(self.yDesired - self.y, self.xDesired - self.x)
        headingDesired = radToDeg(headingDesired)
        self.headingDesired = headingDesired

    def updateVehicleFullState(self, world):
        # check if heading correct
        # if correct, update position
        # else, update heading
        self.getDesiredHeading()
        smallestDif = self.headingDesired - self.heading
        smallestDif = (smallestDif + 180) % 360 - 180
        atPoint = False
        if (self.distToNextPoint() < self.distTol):
            print("At desired point")
            atPoint = True
        else:
            if abs(smallestDif) < 6:  # If pointing in correct direction
                self.updateVehiclePosition()
            else:
                self.updateHeading()
            print(str(self.x) + "," + str(self.y) + ", Heading: " + str(self.heading))

        centerX = world.metersToPixels(self.x + self.width / 2 + world.width / 2)
        centerY = world.metersToPixels(world.height-(self.y + self.length / 2 + world.height / 2))
        print(str(centerX) + " " + str(centerY))
        pixWidth = world.metersToPixels(self.width)
        pixHeight = world.metersToPixels(self.length)
        boxAngle = self.heading
        rect = ((centerX, centerY), (pixWidth, pixHeight), -boxAngle+90)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        retimg = cv2.drawContours(world.img, [box], 0, (0+random.randint(0,50), random.randint(200,255), 0), -1)
        return retimg, atPoint


def round_to(n, precision):
    correction = 0.5 if n >= 0 else -0.5
    return int(n / precision + correction) * precision


def degToRad(deg):
    return deg * np.pi / 180.0


def radToDeg(rad):
    return rad * 180 / np.pi


def polar_to_world(x, y, lidar_theta, scan_theta, scan_radius, resolution):
    # print(str(x) + " " + str(y) + " " + str(lidar_theta) + " " + str(scan_theta) + " " + str(scan_radius))

    local_trans_x = []
    local_trans_y = []
    for t, r in list(zip(scan_theta, scan_radius)):
        local_trans_x.append(round_to((x + (r * cos(lidar_theta + t))), resolution))
        local_trans_y.append(round_to((y + (r * sin(lidar_theta + t))), resolution))
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
    resolution = .05
    sizeX = 20
    sizeY = 20
    grid, nodes = createWorld(resolution, sizeX, sizeY)
    node_dict = dict(zip(grid, nodes))

    world = World(node_dict, sizeX, sizeY, resolution)
    world.createImg()
    img = world.getImg()
    l = fake_lidar_handler()

    veh = VehicleSimulator()
    veh.updateDesiredLocation(-3, 3)

    fig = plt.figure()
    ax = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    xlid, ylid, theta = getPose()
    while True:

        ax.set_ylim(-sizeY / 2, sizeY / 2)
        ax.set_xlim(-sizeX / 2, sizeX / 2)
        hitSizeList = []

        angle, radius = l.get_scan()
        ylid = radius[135]
        # print(list(zip(angle, radius)))

        # trans_x, trans_y = polar_to_world(xlid, ylid, theta, angle, radius, resolution)
        # # print(list(zip(trans_x, trans_y)))
        # for x, y in list(zip(trans_x, trans_y)):
        #     try:
        #         cur_node = world.getNode(x, y)
        #         # print(cur_node)
        #         world.increaseNodeHits(x, y)
        #         hitSizeList.append(world.getHitsAtNode(x, y))
        #     except:
        #         pass
        myimg, atPoint = veh.updateVehicleFullState(world)
        if atPoint:
            veh.updateDesiredLocation(random.randint(-sizeX/2,sizeX/2),random.randint(-sizeY/2,sizeY/2))
        myimg = cv2.resize(myimg, (0,0), fx =2, fy = 2)
        cv2.imshow("img", myimg)
        cv2.waitKey(5)
        # ax.scatter(trans_x, trans_y, s=hitSizeList, c='b')
        #
        # plt.pause(.1)
        # plt.show()
        # plt.cla()
