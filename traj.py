from math import *
from matplotlib import path
from PIL import Image, ImageDraw
import sys
from itertools import filterfalse
from car import CarPosition
from slam import MM_PER_PIXEL

class Waypoint:
	def __init__(self, x, y):
		self.x = x
		self.y = y

class AStarNode:
	def __init__(self, g, h, x, y, rot, parent):
		self.x = x
		self.y = y
		self.rot = rot
		self.g = g
		self.h = h
		self.parent = parent

PATH_WIDTH = 0.6
MAX_DEGREES = 40 #max steering
ASTAR_GRID_SIZE = 0.5

PATH_DIST = 0.5
NUM_PATHS = 20

traj_index = NUM_PATHS
path_polygon = None
in_reverse = False

#PUBLIC METHODS
def update(delta, car_pos, lidar_points):
	if (0.6 >= ((car_pos.goals[car_pos.goal_index].y - car_pos.y)**2 + (car_pos.goals[car_pos.goal_index].x - car_pos.x)**2)**(0.5)):
		if len(car_pos.goals) > car_pos.goal_index + 1: 
			car_pos.goal_index += 1
		else: car_pos.finished = True
	return _update_route(car_pos, lidar_points)
	
#PRIVATE METHODS - DO NOT CALL
def _update_route(car_pos, lidar_points):
	if _should_replan(lidar_points):
		path = _best_path(car_pos, lidar_points)
		if path is None:
			return False
		car_pos.path = path
		car_pos.path_index = 0
		_update_direction(car_pos)
	else:
		dx, dy, rx, ry = car_pos.path[car_pos.path_index+1].x - car_pos.path[car_pos.path_index].x, car_pos.path[car_pos.path_index+1].y - car_pos.path[car_pos.path_index].y, car_pos.x - car_pos.path[car_pos.path_index].x, car_pos.y - car_pos.path[car_pos.path_index].y
		if (rx * dx + ry * dy) / (dx * dx + dy * dy) > 1: 
			car_pos.path_index += 1
			_update_direction(car_pos)
	return True

def _update_direction(car_pos):
	global in_reverse
	path_dir = degrees(atan2(car_pos.path[car_pos.path_index].y - car_pos.path[car_pos.path_index+1].y, car_pos.path[car_pos.path_index].x - car_pos.path[car_pos.path_index+1].x))
	delta = (path_dir - car_pos.rot + 180) % 360 - 180
	if delta < 90 and delta > -90: in_reverse = True
	else: in_reverse = False

def _should_replan(lidar_points):
	print(len(lidar_points))
	if path_polygon is None: return True
	for p in lidar_points:
		if path_polygon.contains_points([p])[0]: return True
	return False

def _gen_paths(car_pos, num_paths, reverse=False):
	dist = -MAX_DEGREES * 2 / num_paths
	trajectories = []
	axledist = -0.5 if reverse else 0.5
	pathdist = -PATH_DIST if reverse else PATH_DIST
	for x in range(num_paths+1):
		angle = (car_pos.rot + MAX_DEGREES) + dist * x
		car_front = (car_pos.x + cos(radians(car_pos.rot)) * axledist, car_pos.y + sin(radians(car_pos.rot)) * axledist)
		trajectories.append([Waypoint(car_front[0], car_front[1]), Waypoint(car_front[0] + cos(radians(angle)) * pathdist, car_front[1] + sin(radians(angle)) * pathdist)])
	return trajectories


def _score_path(trajectory, goal):
	return ((goal.y - trajectory[1].y)**2 + (goal.x - trajectory[1].x)**2)**(0.5)

def _is_safe(trajectory, lidar_points):
	poly = path.Path(_gen_polygon(PATH_WIDTH, trajectory))
	for p in lidar_points:
		if poly.contains_points([p])[0]: return False
	return True

def _retrace(end_node):
	current_node = end_node
	path = [Waypoint(current_node.x, current_node.y)]
	while current_node.parent:
		path.append(Waypoint(current_node.parent.x, current_node.parent.y))
		current_node = current_node.parent

	out = list(reversed(path))
	for o in out:
		print(o.x, o.y)
	print('\n')
	return out

def _a_star(car_pos, lidar_points):
	print('start')
	print(len(lidar_points))
	openset = set()
	closedset = set()
	nodes = {}

	goal = (car_pos.goals[car_pos.goal_index].x // ASTAR_GRID_SIZE, car_pos.goals[car_pos.goal_index].y // ASTAR_GRID_SIZE)
	tmp_pos = CarPosition(car_pos.x, car_pos.y, car_pos.rot)
	openset.add((tmp_pos.x // ASTAR_GRID_SIZE, tmp_pos.y // ASTAR_GRID_SIZE))
	nodes[(tmp_pos.x // ASTAR_GRID_SIZE, tmp_pos.y // ASTAR_GRID_SIZE)] = AStarNode(0, 0, tmp_pos.x, tmp_pos.y, tmp_pos.rot, None)
	while len(openset) > 0:
		current_cell = min(nodes, key=lambda entry: nodes[entry].g + nodes[entry].h)
		current_node = nodes[current_cell]
		if current_cell == goal: 
			print('end')
			return _retrace(current_node)

		tmp_pos.x = current_node.x
		tmp_pos.y = current_node.y
		tmp_pos.rot = current_node.rot

		closedset.add(current_cell)
		openset.remove(current_cell)

		trajectories = _gen_paths(tmp_pos, NUM_PATHS) + _gen_paths(tmp_pos, NUM_PATHS, reverse=True)
		for index, traj in enumerate(trajectories):
			g_cost = 0 if index < NUM_PATHS else 2 #2 penalty for reverse
			cell = (traj[1].x // ASTAR_GRID_SIZE, traj[1].y // ASTAR_GRID_SIZE)
			if not _is_safe(traj, lidar_points) or cell in closedset: continue
			if cell not in openset:
				rot = degrees(atan2(traj[1].y - traj[0].y, traj[1].x - traj[0].x))
				nodes[cell] = AStarNode(g_cost, _score_path(traj, car_pos.goals[car_pos.goal_index]), traj[1].x, traj[1].y, rot, current_node)
				openset.add(cell)
			else:
				if g_cost < nodes[cell].g: 
					nodes[cell].g = g_cost
					nodes[cell].parent = current_node
		del nodes[current_cell]
	print('nope')
	return None


def _best_path(car_pos, lidar_points):
	global path_polygon
	p = _a_star(car_pos, lidar_points)
	if p is not None: path_polygon = path.Path(_gen_polygon(PATH_WIDTH, p))
	return p

def _gen_polygon(width, trajectory):
	polygon = []
	rot = degrees(atan2(trajectory[1].y - trajectory[0].y, trajectory[1].x - trajectory[0].x)) - 90#default for first waypoint
	polygon.append((trajectory[0].x + cos(radians(rot)) * -width, trajectory[0].y + sin(radians(rot)) * -width))
	polygon.append((trajectory[0].x + cos(radians(rot)) * width, trajectory[0].y + sin(radians(rot)) * width))

	for i in range(1, len(trajectory)-1, 1):
		fa = (degrees(atan2(trajectory[i-1].y - trajectory[i].y, trajectory[i-1].x - trajectory[i].x)) + 180) % 360
		sa = (degrees(atan2(trajectory[i+1].y - trajectory[i].y, trajectory[i+1].x - trajectory[i].x)) + 180) % 360
		bisect_angle = (fa + sa) / 2
		polygon.append((trajectory[i].x + cos(radians(bisect_angle)) * -width, trajectory[i].y + sin(radians(bisect_angle)) * -width))

	rot = degrees(atan2(trajectory[-2].y - trajectory[-1].y, trajectory[-2].x - trajectory[-1].x)) + 90
	polygon.append((trajectory[-1].x + cos(radians(rot)) * width, trajectory[-1].y + sin(radians(rot)) * width))
	polygon.append((trajectory[-1].x + cos(radians(rot)) * -width, trajectory[-1].y + sin(radians(rot)) * -width))

	for i in range(len(trajectory)-2, 0, -1):
		fa = (degrees(atan2(trajectory[i-1].y - trajectory[i].y, trajectory[i-1].x - trajectory[i].x)) + 180) % 360
		sa = (degrees(atan2(trajectory[i+1].y - trajectory[i].y, trajectory[i+1].x - trajectory[i].x)) + 180) % 360
		bisect_angle = (fa + sa) / 2
		polygon.append((trajectory[i].x + cos(radians(bisect_angle)) * width, trajectory[i].y + sin(radians(bisect_angle)) * width))
	return polygon
