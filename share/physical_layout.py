import math
import numpy as np

class PhysicalLayout:

	def __init__(self):
		self.layout = {}
		self.layout_centers = {}
		self.cell_data = {}
		self.regions = {}
		self.region_colors = {}
		self.region_cell_data = {}
		self.grid = [0,0]
		self.initialize(0,0)
		self.layout_image_details = ""

	def initialize(self, size_x, size_y):
		self.grid[0] = size_x
		self.grid[1] = size_y
		self.layout = {}
		self.layout_centers = {}
		self.cell_data = {}
		self.regions = {}
		self.region_colors = {}
		self.region_cell_data = {}
		self.initialize_grid()

	def get_grid_size(self):
		return self.grid[0]*self.grid[1]

	def initialize_grid(self):
		for i in range(0,self.grid[0]):
			for j in range(0,self.grid[1]):
				self.cell_data[(i,j)] = []
				self.region_cell_data[(i,j)] = []

	def populate_cell(self, x, y, item):
		if item not in self.layout:
			self.layout[item] = []

		# ensure everything is within bounds
		if x < self.grid[0] and y < self.grid[1] and x >= 0 and y >= 0:
			tup = (x,y)
			self.layout[item].append(tup)
			self.cell_data[tup].append(item)

	def populate_region_cell(self, x, y, region):
		if region not in self.regions:
			self.regions[region] = []

		# ensure everything is within bounds
		if x < self.grid[0] and y < self.grid[1] and x >= 0 and y >= 0:
			tup = (x,y)
			self.regions[region].append(tup)
			self.region_cell_data[tup] = region

	def assign_region_color(self,region,r,g,b):
		self.region_colors[region] = (r,g,b)

	def calculate_centers(self):
		for layout in [self.layout,self.regions]:
			for item in layout:
				x_points = []
				y_points = []
				for grid_cell in layout[item]:
					x_points.append(grid_cell[0])
					y_points.append(grid_cell[1])
				center_x = round(np.average(x_points))
				center_y = round(np.average(y_points))
				self.layout_centers[item] = (center_x,center_y)

	def get_objects_close_to_robot(self, robot_x, robot_y):
		close_objects = {}
		for i in range(max(robot_x-9,0),min(robot_x+9,self.grid[0]-1)):
			for j in range(max(robot_y-9,0),min(robot_y+9,self.grid[1]-1)):
				for item in self.cell_data[(i,j)]:
					if item not in close_objects:
						close_objects[item] = (i,j)
					else:  # determine if the current i,j is closer than the new i,j
						curr_distance = self.get_distance(robot_x,robot_y,close_objects[item][0],close_objects[item][1])
						new_distance = self.get_distance(robot_x,robot_y,i,j)
						if new_distance < curr_distance:
							close_objects[item] = (i,j)
		return close_objects

	def get_region_of_robot(self, robot_x, robot_y):
		if (robot_x,robot_y) in self.region_cell_data and len(self.region_cell_data[(robot_x,robot_y)]) > 0:
			return self.region_cell_data[(robot_x,robot_y)]
		return None

	def determine_if_objects_close(self, x1,y1,x2,y2):
		'''
		Grid values, not pixel values
		'''
		distance = self.get_distance(x1,y1,x2,y2)
		return (distance <= 15)

	def get_distance(self,x1,y1,x2,y2):
		xdiff = x2 - x1
		ydiff = y2 - y1

		xprod = xdiff**2
		yprod = ydiff**2

		summ = float(xprod + yprod)

		distance = math.sqrt(summ)
		return distance