class Axle:
	def __init__(self, yoffset):
		self.rot = 0
		self.yoffset = yoffset

class CarPosition:
	def __init__(self, x, y, rot, goals=[]):
		self.x = x
		self.y = y
		self.rot = rot
		self.path = None
		self.path_index = 0
		self.goals = goals
		self.goal_index = 0
		self.back_axle = Axle(-0.3)
		self.front_axle = Axle(0.3)
		self.finished = False
		self.current_speed = 0