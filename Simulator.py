import numpy as np
from lidar_serial import lidar_handler, fake_lidar_handler
from math import cos, sin, atan2, sqrt
import matplotlib.pyplot as plt
import random
import cv2
import random
import time


class World:
    def __init__(self, worldDict, width, height, resolution):
        self.width = width
        self.height = height
        self.resolution = resolution
        self.dict = worldDict
        self.img = np.zeros((0, 0, 3), np.uint8)
        self.obsList = []

    def getHitsAtNode(self, x, y):
        hits = self.dict[(x, y)].getHits()
        return hits

    def objectAtNode(self, x, y):
        return self.dict[(x, y)].objectPresent()

    def putObjectAtNode(self, x, y):
        self.dict[round(round_to(x, self.resolution),5), round(round_to(y, self.resolution),5)].objectPresent = True
        self.obsList.append(self.dict[round(round_to(x, self.resolution),5), round(round_to(y, self.resolution),5)])
        obs = self.dict[round_to(x, self.resolution), round_to(y, self.resolution)]
        xpix = self.metersToPixels(obs.x + self.width / 2)
        ypix = self.metersToPixels(-obs.y + self.height / 2)
        self.img[int(ypix), int(xpix)] = (0, 0, 255)
        #print(str(xpix) + " " + str(ypix))

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
        return self.objectPresent

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
        self.yawRateMax = 45
        self.time = 0
        self.maxSpeed = 2
        self.timeStep = .1  # This is due to the 10Hz update rate of the Decawave system.
        self.xDesired = 0
        self.yDesired = 0
        self.thetaTol = 3
        self.distTol = .2

    def getPose(self):  # Returns pose of the robot
        return self.x+random.uniform(-.1,.1), self.y+random.uniform(-.1,.1), self.heading+random.uniform(-1,1)

    def __distToNextPoint(self):
        return sqrt((self.yDesired - self.y) ** 2 + (self.xDesired - self.x) ** 2)

    def __updateHeading(self):  # Called when heading needs to be updated if not pointing in correct direction
        self.__getDesiredHeading()
        smallestDif = self.headingDesired - self.heading
        smallestDif = (smallestDif + 180) % 360 - 180
        sign = 0
        if (smallestDif < 0):
            sign = -1
        else:
            sign = 1
        self.heading = self.heading + self.timeStep * self.yawRateMax * sign
        self.heading = self.heading % 360

    def updateDesiredLocation(self, xNew, yNew):  # Call once when moving vehicle to a waypoint.
        self.xDesired = xNew
        self.yDesired = yNew

    def __updateVehiclePosition(self):  # Uses speed to update position over a fixed time step
        self.__updateSpeed()
        self.x = self.x + (self.xSpeed + random.uniform(-.25, .25)) * self.timeStep
        self.y = self.y + (self.ySpeed + random.uniform(-.25, .25)) * self.timeStep

    def __updateSpeed(self):  # Sets the speed of the vehicle in the x and y directions as vectors
        self.xSpeed = cos(degToRad(self.heading)) * self.maxSpeed
        self.ySpeed = sin(degToRad(self.heading)) * self.maxSpeed

    def __getDesiredHeading(self):  # Gives the desired heading based on current point and waypoint
        headingDesired = atan2(self.yDesired - self.y, self.xDesired - self.x)
        headingDesired = radToDeg(headingDesired)
        self.headingDesired = headingDesired

    def updateVehicleFullState(self, world):  # This is the spot where you would implement your obstacle avoidance algorithms
        self.__getDesiredHeading()
        smallestDif = self.headingDesired - self.heading
        smallestDif = (smallestDif + 180) % 360 - 180
        atPoint = False
        if (self.__distToNextPoint() < self.distTol):
            print("At desired point")
            atPoint = True
        else:
            if abs(smallestDif) < 6:  # If pointing in correct direction
                self.__updateVehiclePosition()
            else:
                self.__updateHeading()

        centerX = world.metersToPixels(self.x + world.width / 2)
        centerY = world.metersToPixels(world.height - (self.y + world.height / 2))
        pixWidth = world.metersToPixels(self.width)
        pixHeight = world.metersToPixels(self.length)
        boxAngle = self.heading
        rect = ((centerX, centerY), (pixWidth, pixHeight), -boxAngle + 90)
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        retimg = cv2.drawContours(world.img, [box], 0, (0 + random.randint(0, 255), random.randint(200, 255), 0), -1)
        return retimg, atPoint


class fake_world_lidar_handler:
    def __init__(self, world, vehicle):
        self.data = []
        self.world = world
        self.vehicle = vehicle
        self.startAngle = -5
        self.endAngle = 185
        self.maxRange = 70
        self.resolution = 1

    def get_scan(self):
        angle = []
        radius = []
        for node in self.world.obsList:
            curX = node.x
            curY = node.y
            vHeading = self.vehicle.heading
            a = round_to(radToDeg(atan2(curY - self.vehicle.y, curX - self.vehicle.x)) - vHeading, self.resolution)
            if (a > -(360-self.endAngle) - 90 and a < self.startAngle - 90) or (a > self.endAngle-90 and a < (360-self.startAngle)-90):
                continue
            distToObstacle = sqrt((curY - self.vehicle.y) ** 2 + (curX - self.vehicle.x) ** 2) + random.uniform(-.1, .1)
            if distToObstacle > self.maxRange:
                continue
            angle.append(a * .01745329251)
            radius.append(distToObstacle)
        return angle, radius


class Obstacle:
    obstacleList = []

    def __init__(self, x, y, radius, world):
        self.x = x
        self.y = y
        self.radius = radius
        self.obstacleList.append(self)
        for i in np.arange(self.x - self.radius, self.x + self.radius, .01):
            for j in np.arange(self.y - self.radius, self.y + self.radius, .01):
                world.putObjectAtNode(round(round_to(i, world.resolution), 5), round(round_to(j, world.resolution), 5))


def round_to(n, precision):
    correction = 0.5 if n >= 0 else -0.5
    return round(int(n / precision + correction) * precision,5)


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
    return my_grid_list, my_node_list


if __name__ == "__main__":
    resolution = .05
    sizeX = 20
    sizeY = 20
    plotToggle = True

    timeScale = 1  # Less than or equal to 1

    grid, nodes = createWorld(resolution, sizeX, sizeY)
    node_dict = dict(zip(grid, nodes))
    world = World(node_dict, sizeX, sizeY, resolution)
    world.createImg()
    img = world.getImg()

    veh = VehicleSimulator()
    numObstacles = 8
    simpleobslist = []
    for i in range(0, numObstacles):
        try:
            simpleobslist.append(Obstacle(random.uniform(-sizeX/2,sizeX/2), random.uniform(-sizeY/2,sizeY/2), random.uniform(.1,.3), world))
            print("Obstacle created")
        except:
            print("Exception: Obstacle not created")
            continue

    l = fake_world_lidar_handler(world, veh)

    veh.updateDesiredLocation(0, 7)  # This sets where we want the vehicle to go!

    plt.figure(1)

    while True:  # This would be the main loop we care about during competition
        startTime = time.time()
        hitSizeList = []

        angle, radius = l.get_scan()  # Get a scan
        vehX, vehY, vehHeading = veh.getPose()  # Get our vehicle's location
        # We'd probably also want to get an image/boolean indicating stop sign presence here!
        trans_x, trans_y = polar_to_world(vehX, vehY, degToRad(vehHeading), angle, radius, resolution)  # Translate the lidar scan to our map points
        for x, y in list(zip(trans_x, trans_y)):  # This for loop acts on the world nodes based on lidar scan points.
            try:
                cur_node = world.getNode(x, y)
                world.increaseNodeHits(x, y)
                hitSizeList.append(world.getHitsAtNode(x, y))
            except:
                pass
        myimg, atPoint = veh.updateVehicleFullState(world)  # This is the function in which path planning would occur.
        if atPoint:  # If we make it to a point, pick a new point.  Good for simulation, possibly competition?
            veh.updateDesiredLocation(random.randint(-sizeX / 2, sizeX / 2), random.randint(-sizeY / 2, sizeY / 2))

        # Show our image, which represents world, objects, vehicle.
        myimg = cv2.resize(myimg, (0, 0), fx=1, fy=1)
        cv2.imshow("img", myimg)
        cv2.waitKey(int(1/timeScale))

        if plotToggle:  # This draws the lidar scan relative to world & lidar itself
            modAngle = [a + np.pi / 2 for a in angle]
            ax1 = plt.subplot(211, projection="polar")
            ax1.cla()
            ax1.set_ylim([0, 20])
            ax1.scatter(modAngle, radius)
            ax2 = plt.subplot(212)
            ax2.set_xlim([-sizeX / 2, sizeX / 2])
            ax2.set_ylim([-sizeY / 2, sizeY / 2])
            ax2.set_aspect('equal')
            ax2.set_autoscale_on(False)
            ax2.scatter(trans_x, trans_y, s=1, c='b')
            ax2.scatter(list([vehX]), list([vehY]), s=1, c='r')
            plt.pause(.05/timeScale)
            #plt.cla()
        endTime = time.time()
        print("Time taken: " + str(endTime-startTime))
