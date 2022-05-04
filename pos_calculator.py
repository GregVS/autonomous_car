from math import radians, cos, sin, atan2, degrees
import numpy as np

class Line:
	def __init__(self, x1, y1, x2, y2):
		self.x1 = x1
		self.y1 = y1
		self.x2 = x2
		self.y2 = y2

def calc_velocity(car_pos, dist):
	if car_pos.front_axle.rot == 0:
		return (car_pos.x + dist * cos(radians(car_pos.rot)), car_pos.y + dist * sin(radians(car_pos.rot)), car_pos.rot)
	back_perp_line = _perpendicular_line(car_pos, car_pos.back_axle)
	front_perp_line = _perpendicular_line(car_pos, car_pos.front_axle)
	origin = _intersection(back_perp_line, front_perp_line)
	return _calc_pos(dist, back_perp_line, front_perp_line, origin, car_pos)

def _calc_pos(dist, back_line, front_line, origin, car_pos):
	radius = ((back_line.x1 - origin[0])**2 + (back_line.y1 - origin[1])**2)**(0.5)
	angle_on_circle = (degrees(atan2(back_line.y1 - origin[1],back_line.x1 - origin[0])) + 360) % 360 #relative to back wheel
	car_angle = 0
	if car_pos.front_axle.rot > 0: 
		angle_on_circle += (degrees(dist / radius) + 360) % 360
		car_angle = angle_on_circle + 90
	else: 
		angle_on_circle -= (degrees(dist / radius) + 360) % 360
		car_angle = angle_on_circle - 90
	return (origin[0] + radius * cos(radians(angle_on_circle)) - car_pos.back_axle.yoffset * cos(radians(car_angle)),
		origin[1] + radius * sin(radians(angle_on_circle)) - car_pos.back_axle.yoffset * sin(radians(car_angle)), car_angle)

def _intersection(l1, l2):
	d = (l2.y2 - l2.y1) * (l1.x2 - l1.x1) - (l2.x2 - l2.x1) * (l1.y2 - l1.y1)
	if d == 0: return None
	ua = ((l2.x2 - l2.x1) * (l1.y1 - l2.y1) - (l2.y2 - l2.y1) * (l1.x1 - l2.x1)) / d
	return (l1.x1 + (l1.x2 - l1.x1) * ua, l1.y1 + (l1.y2 - l1.y1) * ua)

def _perpendicular_line(car_pos, axle):
	rot = axle.rot + car_pos.rot + 90
	x = car_pos.x + axle.yoffset * cos(radians(car_pos.rot))
	y = car_pos.y + axle.yoffset * sin(radians(car_pos.rot))
	return Line(x, y, x + 100 * cos(radians(rot)), y + 100 * sin(radians(rot)))
