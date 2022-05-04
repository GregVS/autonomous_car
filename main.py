from car import CarPosition
import pid
from shapely.geometry.polygon import Polygon
import traj as trajectory
from arduino import SerialThread
from slam import Slam
from queue import Queue
from queue import Empty as QueueEmpty
from math import atan2, degrees
from robot_gui import RobotGUI
from datetime import datetime
import pos_calculator as pos_sim
import sys

MAX_DEGREES = 19
SLAM_RATE = 25 #in milliseconds
RECALC_RATE = 50
SAMPL_SIZE = 370
FPS = 0.5 #feet per second

def connect_to(robot, btn, port):
	if robot.connect(port):
		print('connected')
		btn.config(state='disabled')
		robot.start_loops()
	else: print('failed')

def main():
	gui = RobotGUI(title='Driverless Car')
	robot = Robot(gui)
	gui.set_close_callback(robot.close)
	gui.set_pause_callback(robot.pause)
	gui.set_reload_waypoints_callback(robot.reload_goals)
	gui.set_connect_callback(lambda btn, port: connect_to(robot, btn, port))
	gui.mainloop()

class Robot:
	def __init__(self, gui):
		self.gui = gui
		self.rxqueue = Queue()
		self.txqueue = []
		self.car_pos = CarPosition(x=0, y=0, rot=90, goals=[trajectory.Waypoint(-2, 9.5)])
		self.slam = Slam()
		self.serThread = SerialThread(self.rxqueue, self.txqueue)
		self.points = []
		self.last_time = datetime.now()
		self.paused = False

	def reload_goals(self,goals):
		self.car_pos.goals = goals

	def pause(self, force_pause=False):
		if not self.paused or force_pause: 
			# self.txqueue.put(254) # stop command on arduino
			self.paused = True
			self.car_pos.current_speed = 0
			self.gui.pause_btn.config(text = 'Resume')
		else:
			self.paused = False
			self.gui.pause_btn.config(text = 'Pause')

	def connect(self, port): # returns if successful
		return self.serThread.open(port)

	def close(self, root):
		root.destroy()
		self.serThread.stop()

	def start_loops(self):
		self.serThread.start() #begin listening
		self.update_slam() # init = True
		self.update_car()

	#collect data and run slam
	def update_slam(self, init=True):
		if not self.paused and self.rxqueue.qsize() > SAMPL_SIZE * (2 if init else 1):
			self._collect_scans(init=init)
			self._run_slam()
			self.gui.update_slam_render(self.slam.mapbytes)
			if init: init = False
		
		self.gui.schedule(SLAM_RATE, lambda: self.update_slam(init=init))

	def _recalc_car(self):
		if self.car_pos.finished:
			self.pause(force_pause=True)
			return
		if not trajectory.update(RECALC_RATE/1000, self.car_pos, self.slam.processed_lidar_points(self.car_pos)): 
			print('all paths blocked', trajectory.PATH_DIST)
			self.car_pos.current_speed = 0
			return #if no possible path, wait
		self.gui.update_path_canvas(trajectory.traj_index)

 		#calculate steering
		steering = pid.update(RECALC_RATE/1000, self.car_pos)
		if steering > MAX_DEGREES: steering = MAX_DEGREES
		elif steering < -MAX_DEGREES: steering = -MAX_DEGREES
		servo_degrees = int(90 + (90 * (steering / MAX_DEGREES)))
		self.car_pos.current_speed = FPS
		self.txqueue.append(servo_degrees)
		self.txqueue.append(int(trajectory.in_reverse))

		self.car_pos.front_axle.rot = steering

	#recalc trajectory and steering data
	def update_car(self):
		#update components
		if not self.paused:
			self._recalc_car()
		self.gui.schedule(RECALC_RATE, self.update_car)
		

	#execute slam and get pos
	def _run_slam(self):
		curr = datetime.now()
		dist = FPS * ((curr - self.last_time).microseconds / 1000000)
		self.last_time = datetime.now()
		#run slam
		vel = pos_sim.calc_velocity(self.car_pos, dist)
		v = self.slam.calc_velocity(self.points, vel)
		self.car_pos.x += v[0] #x is reversed
		self.car_pos.y -= v[1]
		self.car_pos.rot = (self.car_pos.rot - v[2] + 360) % 360
		#log to gui
		self.gui.update_pos(self.car_pos.x, self.car_pos.y, self.car_pos.rot)

	#gather waiting scan data
	def _collect_scans(self, init=False):
		self.points = []
		while True:
			try: data = self.rxqueue.get_nowait()
			except QueueEmpty: return
			else: self.points.append(data)
		if init: self._collect_scans() #clears the old data

#BEGIN PROGRAM
main()