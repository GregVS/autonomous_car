from breezyslam.algorithms import RMHC_SLAM
from breezyslam.components import Laser
import numpy as np
from math import cos, sin, radians
from shapely.geometry import Point

class MyLidar(Laser):
	def __init__(self):
		Laser.__init__(self, 360, 3, 360, 6000, 0, 0)

MAP_SIZE = 10 #meters
MAP_PIXELS = 600
MM_PER_PIXEL = MAP_SIZE * 1000 / MAP_PIXELS

class Slam():
	def __init__(self):
		lidar = MyLidar()
		self.slam = RMHC_SLAM(lidar, MAP_PIXELS, MAP_SIZE, random_seed=9999, hole_width_mm=800)
		self.pos = self.slam.getpos()
		self.mapbytes = bytearray(MAP_PIXELS*MAP_PIXELS)
		self.data = []

	def calc_velocity(self, points, velocity):
		#create data array from lidar points
		self.data = [0 for i in range(360)]
		for point in points:
			self.data[int(point[1])] = int(point[0])
		self.data = [self.data[i-180] for i in range(360)]

		#run slam with odometry
		self.slam.update(self.data, velocity)
		diff = np.subtract(self.slam.getpos(), self.pos)
		self.pos = self.slam.getpos()
		self.slam.getmap(self.mapbytes)

		#convert mm to feet
		diff[0] *= 0.003280839895
		diff[1] *= 0.003280839895
		return diff

	def processed_lidar_points(self, car_pos):
		points = []
		for index, pixel in enumerate(self.mapbytes):
			if pixel == 127: continue
			if pixel > 0 and pixel < 220: points.append(((index % MAP_PIXELS * MM_PER_PIXEL) - MAP_SIZE*500, (index // MAP_PIXELS * MM_PER_PIXEL)- MAP_SIZE*500))
		print('done')
		return points