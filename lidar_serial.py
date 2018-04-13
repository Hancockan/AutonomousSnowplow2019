import socket
from timeit import default_timer as timer
import numpy as np
import matplotlib.pyplot as plt

#constants and commands 
status = bytearray([0x02, 0x73, 0x052, 0x4E, 0x20, 0x4C, 0x43, 0x4D, 0x73, 0x74, 0x61, 0x74, 0x65, 0x03])
operating_hours = bytearray([0x02, 0x73, 0x52, 0x4E, 0x20, 0x4F, 0x44, 0x6F, 0x70, 0x72, 0x68, 0x03])
last_scan = bytearray([0x02, 0x73, 0x52, 0x4E, 0x20, 0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61, 0x03])
# ScanFreq1 = bytearray([0x02, 0x73, 0x4D, 0x4E, 0x20, 0x6D, 0x4C, 0x4D, 0x50, 0x73, 0x65, 0x74, 0x73,
# 0x63, 0x61, 0x6E, 0x63, 0x66, 0x67, 0x20, 0x00, 0x00, 0x27, 0x10, 0x20, 0x00, 0x01, 0x20, ])

TCP_IP = '169.254.144.5'
TCP_PORT = 2112
BUFFER_SIZE = 8000 #arbitrary value, but big enough for all data in one scan

fig = plt.figure()
ax = fig.add_axes([0.1,0.1,0.8,0.8], polar = True)
ax.set_ylim(0, 5)
ax.set_yticks(np.arange(0, 3, 1))

class lidar_detected_object:
	def __init__(self, radius, angle):
		self.radius = radius
		self.angle = angle

	def print_info(self):
		print('radius: ' + str(self.radius) + ' angle: ' + str(self.angle * 180.0/np.pi))

while 1:
	
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect((TCP_IP, TCP_PORT))
	start = timer() #testing performance
	s.send(last_scan) #send data over socket
	data = s.recv(BUFFER_SIZE) #receive back data from socket
	data_array = data.split() #splits data by space
	#print(data_array)
	data_points = int(data_array[25], 16)
	#print('number of data points: ' + str(data_points))

	angle = []
	radius = []

	cur_angle = -5.0 #starting angle of the lidar


	try:
		for x in range(0, data_points):
			# distance_mm = int(data_array[26 + x], 16)
			# distance_m = distance_mm/1000.0
			# #print('Angle: ' + str(cur_angle) + ' Distance: ' + str(distance_m) + 'm iteration: ' + str(x))
			# ax.scatter((cur_angle*np.pi/180.0), distance_m, c = 'b', s = 0.25)
			angle.append((cur_angle * 0.01745329251))
			# angle.append(np.deg2rad(cur_angle))
			radius.append((int(data_array[26 + x], 16)/1000.0))
			cur_angle+=.1667

		end = timer()

		#ax.scatter(angle, radius, c = 'b', s = 0.25)

		#print('Speed: ' + str(1.0/(end - start)) + 'Hz ' + str(radius[94]))
		#ax.set_title("lidar readings")
		#print(radius[570])
		objects_list = []
		object_threshold = 2.0
		object_detected = False
		objects = 0
		first_edge = 0
		last_edge = 0
		for x in range(0, data_points):
			if (x == 0):
				if (radius[x] < object_threshold):
					object_detected = True
					first_edge = 0
				continue
			if ((object_detected == True) and (x != 0)):
				if radius[x] < object_threshold:
					object_detected = True
				else:
					object_detected = False
					last_edge = x
					ob = lidar_detected_object(radius[x-1], angle[x-1])
					objects_list.append(ob)
					objects+=1
				continue
			if ((object_detected == False) and (x != 0)):
				if radius[x] < object_threshold:
					object_detected = True
					first_edge = x
				continue
		print('objects:')
		for x in range(0, len(objects_list)):
			objects_list[x].print_info()
	except:
		print('Error. Total data points: ' + str(data_points) + ' data array size: ' + str(len(data_array)))
	s.close()
plt.show()