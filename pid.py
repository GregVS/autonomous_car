import math

K = 0.3

def update(delta, car_pos):
	try: seg = (car_pos.path[car_pos.path_index], car_pos.path[car_pos.path_index+1])
	except: return 0
	return _calc(delta, seg, car_pos)

def _calc(delta, seg, car_pos):
	dx, dy, rx, ry = seg[1].x - seg[0].x, seg[1].y - seg[0].y, car_pos.x - seg[0].x, car_pos.y - seg[0].y
	cte = (ry * dx - rx * dy) / ((dx * dx + dy * dy)**(0.5))
	seg_rot = (math.degrees(math.atan2(seg[1].y - seg[0].y, seg[1].x - seg[0].x)) + 360) % 360
	return (seg_rot - car_pos.rot + math.degrees(-math.atan(K * cte)) + 180) % 360 - 180