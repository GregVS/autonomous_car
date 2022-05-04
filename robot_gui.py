from tkinter import *
from tkinter.scrolledtext import ScrolledText
from tkinter import ttk
import tkinter.simpledialog, tkinter.filedialog, tkinter.messagebox
import glob
from PIL import Image, ImageTk
from math import sin, cos, radians
from traj import MAX_DEGREES, NUM_PATHS, PATH_DIST
import os
import traj as trajectory
from slam import MAP_PIXELS as MAP_SIZE_PIXELS

class RobotGUI:

	def __init__(self, title='Untitled'):
		self.root = Tk()
		self.root.title(title)

		s=ttk.Style()
		s.theme_use('clam')

		tab_notebook = ttk.Notebook(self.root)
		self.live_frame = Frame(tab_notebook)
		self.waypoint_frame = Frame(tab_notebook)
		tab_notebook.add(self.live_frame, text='Live Feed')
		tab_notebook.add(self.waypoint_frame, text='Waypoints')
		tab_notebook.pack()

		self.menubar = Menu(self.root)
		self.filemenu = Menu(self.menubar, tearoff=0)
		self.filemenu.add_command(label='Save', command=self._save)
		self.filemenu.add_command(label='Open', command=self._load)
		self.menubar.add_cascade(label="File", menu=self.filemenu)
		self.root.config(menu=self.menubar)

		self.frame = Frame(self.live_frame)
		self.frame.pack(side=RIGHT)

		self.pos_label = ScrolledText(self.live_frame, width=55, height=40)
		self.pos_label.insert(END, 'Position Data:\n')
		self.pos_label.config(state=DISABLED)
		self.pos_label.pack(side=LEFT, fill="both", expand=True)

		self.devices_list = Listbox(self.frame, width=25, height=4)
		self.devices_list.config(selectmode=SINGLE)
		self.devices_list.grid(row=0, padx=5)
		for name in list(glob.glob('/dev/tty.*')):
			self.devices_list.insert(END, name)
		self.devices_list.selection_set(END)

		self.connect_btn = Button(self.frame, text='Connect')
		self.connect_btn.grid(row=1)
		self.pause_btn = Button(self.frame, text='Pause')
		self.pause_btn.grid(row=2)

		self.slam_canvas = Canvas(self.frame, width=MAP_SIZE_PIXELS/2, height=MAP_SIZE_PIXELS/2, bg="#000000")
		self.slam_canvas.grid(row=4)

		self.path_canvas = Canvas(self.live_frame, width=400, height=600, bg='#000000')
		self.path_canvas.pack(side=RIGHT, fill="both", expand=True)
		self.update_path_canvas(NUM_PATHS/2)

		top = Frame(self.waypoint_frame)
		top.pack(side=TOP)
		self.add_waypoint_x = Entry(self.waypoint_frame, width=3)
		self.add_waypoint_y = Entry(self.waypoint_frame, width=3)
		self.add_waypoint_x.insert(END, 'X')
		self.add_waypoint_y.insert(END, 'Y')
		self.add_waypoint_x.pack(in_=top, side=LEFT)
		self.add_waypoint_y.pack(in_=top, side=RIGHT)
		self.add_waypoint_btn = Button(self.waypoint_frame, text='Add Waypoint', command=self._add_waypoint_btn_click)
		self.add_waypoint_btn.pack()

		self.waypoint_listbox = Listbox(self.waypoint_frame, width=25, height=6)
		self.waypoint_listbox.config(selectmode=SINGLE)
		self.waypoint_listbox.pack()
		self.del_waypoint_btn = Button(self.waypoint_frame, text='Delete Waypoint', command=self._del_waypoint_click)
		self.del_waypoint_btn.pack()
		self.waypoints = []
		self.reload_waypoint_func = None

		self.mapbytes = None

	def _del_waypoint_click(self):
		try: del self.waypoints[self.waypoint_listbox.curselection()[0]]
		except: return
		self.waypoint_listbox.delete(self.waypoint_listbox.curselection()[0])
		self.reload_waypoint_func()

	def set_reload_waypoints_callback(self, callback):
		self.reload_waypoint_func = lambda: callback(self.waypoints)

	def _add_waypoint_btn_click(self):
		try: wayp = trajectory.Waypoint(float(self.add_waypoint_x.get()), float(self.add_waypoint_y.get()))
		except: return
		self.add_waypoint_x.delete(0, END)
		self.add_waypoint_y.delete(0, END)
		self.waypoints.append(wayp)
		self.waypoint_listbox.insert(END, '{0}, {1}'.format(wayp.x, wayp.y))
		self.reload_waypoint_func()

	def set_connect_callback(self, callback):
		self.connect_btn.config(command=lambda: callback(self.connect_btn, self.devices_list.get(self.devices_list.curselection())))

	def set_pause_callback(self, callback):
		self.pause_btn.config(command=callback)

	def set_close_callback(self, callback):
		self.root.protocol('WM_DELETE_WINDOW', lambda: callback(self.root))

	def update_path_canvas(self, index):
		self.path_canvas.delete('all')
		self.path_canvas.create_rectangle(170, 300, 230, 400, fill='red')
		dist = MAX_DEGREES * 2/ NUM_PATHS
		for x in range(NUM_PATHS+1):
			angle = (270 - MAX_DEGREES) + dist * x
			if x == index:
				self.path_canvas.create_line(200, 300, 200+cos(radians(angle)) * PATH_DIST * 100, 300+sin(radians(angle)) * PATH_DIST * 100, fill='green')
			else:
				self.path_canvas.create_line(200, 300, 200+cos(radians(angle)) * PATH_DIST * 100, 300+sin(radians(angle)) * PATH_DIST * 100, fill='white')
		self.path_canvas.update()


	def update_slam_render(self, mapbytes):
		self.slam_canvas.delete('all')
		self.mapbytes = mapbytes
		self.image = Image.frombuffer('L', (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS), mapbytes, 'raw', 'L', 0, 1)
		self.image.thumbnail((MAP_SIZE_PIXELS/2, MAP_SIZE_PIXELS/2), Image.ANTIALIAS)
		self.photo = ImageTk.PhotoImage(image=self.image)
		self.slam_canvas.create_image(MAP_SIZE_PIXELS/4,MAP_SIZE_PIXELS/4, image=self.photo)
		self.slam_canvas.update()

	def schedule(self, millis, func):
		self.root.after(millis, func)

	def update_pos(self, x, y, rot):
		self.pos_label.config(state=NORMAL)
		self.pos_label.insert(END, "x -> {0}\ty -> {1}\t r -> {2}\n".format(round(x, 4), round(y, 4), round(rot, 4)))
		self.pos_label.config(state=DISABLED)
		self.pos_label.see(END)

	def _load(self):
		foldername = tkinter.filedialog.askdirectory()
		pos_file = foldername + '/' + 'pos_file.log'
		try:
			with open(pos_file, 'r') as f_file:
				self.pos_label.config(state=NORMAL)
				self.pos_label.delete(1.0, END)
				for line in f_file:
					self.pos_label.insert(END, line)
				self.pos_label.config(state=DISABLED)
			map_file = foldername + '/' + 'map_file.imgd'
			with open(map_file, 'rb') as f_file:
				self.update_slam_render(bytearray(f_file.read()))
		except: tkinter.messagebox.showwarning('Folder Error', 'Could not load from directory')

	def _save(self):
		try: foldername = 'logs/' + tkinter.simpledialog.askstring('Save File', 'Name file')
		except: return
		if not os.path.exists(foldername):
			os.makedirs(foldername)
		pos_file = foldername + '/' + 'pos_file.log'
		with open(pos_file, 'w+') as f_file:
			pos_data = self.pos_label.get(1.0,END)
			f_file.write(pos_data)
		map_file = foldername + '/' + 'map_file.imgd'
		with open(map_file, 'wb') as f_file:
			f_file.write(self.mapbytes)

	def mainloop(self):
		self.root.focus_force()
		self.root.mainloop()