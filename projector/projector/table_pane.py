from PyQt5.QtWidgets import QLabel, QPushButton
from PyQt5.QtGui import QPainter, QBrush, QPen, QFont, QColor, QCursor, QPixmap, QIcon, QPolygon
from PyQt5.QtCore import Qt, QPoint

import math
import random
import numpy as np
import time
import threading
from pynput.mouse import Button, Controller

from figaro_msgs.msg import PathComponent

class NotificationLabel(QLabel):

	def __init__(self,parent,text):
		super().__init__(parent=parent)
		self.itr = 0
		self.text = text
		print("initialized notification")

	def increment(self):
		self.itr += 1

	def renew(self):
		if self.itr > 500:
			self.itr = 500

class TablePane(QLabel):

	def __init__(self,gui,physical_layout, test_ui, parent, offset_x=0, offset_y=0):
		super().__init__(parent=parent)
		
		self.setGeometry(0,0,gui.width,gui.height)

		# not whether we are in test_ui
		self.test_ui = True

		# set cursor
		if not test_ui:
			self.setCursor(QCursor(Qt.BlankCursor))
			self.test_ui = False

		# set params
		self.xdim = 72
		self.ydim = 54

		self.recording = False
		self.gui = gui

		self.robot_x = 50
		self.robot_y = 50
		self.robot_prev_x = 50
		self.robot_prev_y = 50
		self.orientation_lock = threading.Lock()
		self.robot_orientation = AgentOrientation([[0.0,0.0],[100.0,0.0],[100.0,100.0],[0.0,100.0]],self.orientation_lock)


		self.human_coords = {}
		self.human_orientation = {}
		self.human_dragged = {}

		self.robot_dragged = False
		self.human_coords["h1"] = NamedCoords("h1",50,500)
		self.human_orientation["h1"] = AgentOrientation([[0.0,450.0],[100.0,450.0],[100.0,550.0],[0.0,550.0]], self.orientation_lock)
		self.human_dragged["h1"] = False

		self.updated_externally = False

		self.physical_layout = physical_layout

		self.content_label = QLabel(parent=self)
		self.content_label.setStyleSheet(""" QLabel {border: 2px solid grey; border-radius: 8px; background-color: rgba(255, 255, 255, 0.9); color: grey; font: 24px 'Monaco';} """)
		# content coordinates are relative to the specified agent
		self.content_x = 0 
		self.content_y = 0
		self.content_timer_val = 0
		self.content_agent = "robot"
		self.content_label.setAlignment(Qt.AlignTop)
		self.content_label.hide()
		self.content_id = 0
		self.highlighting_robot_say = []
		self.highlighting_human_say = []
		self.highlighting_robot_do = []
		self.highlighting_human_do = []

		self.content_flag = QPushButton(parent=self.content_label)
		self.content_flag.setCheckable(True)
		#self.content_flag.toggle()
		self.content_flag.clicked.connect(self.flag_content)
		self.content_flag.setGeometry(50, 10, 50, 50)
		pixmap = QPixmap("icons/flag-black-18dp/2x/outline_flag_black_18dp.png")
		flag_icon = QIcon(pixmap)
		self.content_flag.setIcon(flag_icon);
		self.content_flag.setIconSize(pixmap.rect().size())
		self.content_flag.setStyleSheet(""" QPushButton {border: 3px solid grey; border-radius: 24px; background-color: rgba(255, 255, 255, 1.0);} 
											QPushButton:checked {border: 3px solid grey; border-radius: 24px; background-color: rgba(100, 100, 100, 1.0);}""")
		self.content_delete = QPushButton(parent=self.content_label)
		self.content_delete.setCheckable(True)
		#self.content_delete.toggle()
		self.content_delete.clicked.connect(self.delete_content)
		self.content_delete.setGeometry(50, 10, 50, 50)
		pixmap = QPixmap("icons/delete-black-18dp/2x/outline_delete_black_18dp.png")
		delete_icon = QIcon(pixmap)
		self.content_delete.setIcon(delete_icon);
		self.content_delete.setIconSize(pixmap.rect().size())
		self.content_delete.setStyleSheet(""" QPushButton {border: 3px solid grey; border-radius: 24px; background-color: rgba(255, 255, 255, 1.0);} 
											QPushButton:checked {border: 3px solid grey; border-radius: 24px; background-color: rgba(100, 100, 100, 1.0);}""")

		# handle pausing and playing
		self.play_pause_label = QLabel(parent=self)
		self.play_pause_label.setGeometry(self.width() - 110,10,100,100)
		self.play_pause_button = QPushButton(parent=self.play_pause_label)
		self.play_pause_button.setGeometry(0,0,100,100)
		pixmap = QPixmap("icons/pause.png")
		pause_icon = QIcon(pixmap)
		self.play_pause_button.setIcon(pause_icon);
		self.play_pause_button.setIconSize(pixmap.rect().size())
		self.play_pause_button.setStyleSheet(""" QPushButton {border: 0px; background-color: rgba(255, 255, 255, 0.0);}""")
		self.play_pause_button.clicked.connect(self.toggle_pause)
		self.human_display_path = []
		self.robot_display_path = []
		self.rewind_idx = -1
		self.display_path_lock = threading.Lock()
		self.paused = False
		self.play_pause_label.hide()

		# handling notifications
		#self.notification_label = NotificationLabel(self,None)
		#self.notification_label.setGeometry(10,10,200,100)
		#self.notification_label.setStyleSheet(""" QLabel {background-color: rgba(200, 200, 200, 0.7);}""")
		#self.notification_label.hide()

		# position history
		self.position_queue = {"r":[],"h1":[]}
		self.position_color = {"r":(150,156,158),"h1":(74,96,161)}

		# assets
		self.proximal_pic = QPixmap("icons/agents_proximal.png")
		self.warning_pic = QPixmap("icons/notify/notify.png")

		# mouse emulator
		self.mouse = Controller()
		self.active_clicks = {}
		self.offset_x = offset_x
		self.offset_y = offset_y

		# calibrator
		self.calibrator = Calibrator(self)
		self.static_calib_position = None

		# thread the content label timer
		thread = threading.Thread(target=self.ui_update_timer)
		thread.daemon = True							# Daemonize thread
		thread.start()

		# locks
		self.click_lock = threading.Lock()
		self.point_lock = threading.Lock()
		self.wait_lock = threading.Lock()
		self.nod_lock = threading.Lock()
		self.layout_lock = threading.Lock()
		#self.notification_label_lock = threading.Lock()

		# pre-determining label placements
		self.region_label_placements = {}

		# pointing, waiting, nodding
		self.point = None
		self.wait = False
		pixmap = QPixmap("icons/baseline_hourglass_empty_black_18dp.png")
		pixmap = pixmap.scaledToWidth(100)
		pixmap = pixmap.scaledToHeight(100)
		self.wait_label = QLabel(self)
		self.wait_label.setGeometry(0,0,100,100)
		self.wait_label.setPixmap(pixmap)
		self.wait_label.hide()
		self.nod = None

	def set_grid_size(self,x,y):
		self.xdim = x
		self.ydim = y

	def begin_calibration(self):
		self.calibrator.begin_calibration()
		self.calibrator.increment()

	def enable_recording(self):
		self.recording = True
		paused = False
		self.play_pause_label.show()
		self.update()

	def disable_recording(self):
		self.recording = False
		self.paused = False
		pixmap = QPixmap("icons/pause.png")
		pause_icon = QIcon(pixmap)
		self.play_pause_button.setIcon(pause_icon);
		self.play_pause_button.setIconSize(pixmap.rect().size())
		self.human_display_path = []
		self.robot_display_path = []
		self.play_pause_label.hide()
		self.update()

	def toggle_pause(self):
		if self.paused:
			self.paused = False
		else:
			self.paused = True

		if self.paused:
			pixmap = QPixmap("icons/play.png")
			play_icon = QIcon(pixmap)
			self.play_pause_button.setIcon(play_icon);
			self.play_pause_button.setIconSize(pixmap.rect().size())
		else:
			pixmap = QPixmap("icons/pause.png")
			pause_icon = QIcon(pixmap)
			self.play_pause_button.setIcon(pause_icon);
			self.play_pause_button.setIconSize(pixmap.rect().size())
			self.human_display_path = []
			self.robot_display_path = []

		# send message to gui requesting to pause
		if self.paused:
			self.gui.request_pause(self.paused,rewind=None)
		else:
			self.gui.request_pause(self.paused,rewind=self.rewind_idx)

	def extract_paths(self,path_array):
		print("received path")
		print(path_array)
		self.display_path_lock.acquire()

		for component in path_array:
			pix_x = int(round(component.x*self.width()))
			pix_y = int(round(component.y*self.height()))
			agent = component.agent
			if agent == "robot":
				self.robot_display_path.append((pix_x,pix_y))
			else:
				self.human_display_path.append((pix_x,pix_y))

		self.display_path_lock.release()

	def update_content_position(self):
		# udpate the content
		if self.content_agent == "robot":
			self.content_label.move(self.robot_x+self.content_x,self.robot_y+self.content_y)
		else:
			self.content_label.move(self.human_coords["h1"].x+self.content_x,self.human_coords["h1"].y+self.content_y)

	'''
	def add_notification(self,typ):
		if self.notification_label.text is None or self.notification_label.text != text:
			self.notification_label.text = typ
			self.notification_label.itr = 0
			self.notification_label.setText(typ)
			self.notification_label.show()
		elif self.notification_label.text == text:
			self.notification_label.renew()
	'''

	def update_position(self, ident, x, y, corners):
		#print("{} - {}x{}".format(ident,x,y))
		if ident == "r":
			self.robot_x = x
			self.robot_y = y

		else:
			self.human_coords[ident].x = x
			self.human_coords[ident].y = y
		#print("{} - {}".format(self.robot_x,self.robot_y))

		# update the orientation
		if corners is not None and not self.is_moving(ident):
			if ident == "r":
				robot_faces = self.convert_corners_to_faces(corners,self.robot_x,self.robot_y)
				self.robot_orientation.orientation = robot_faces
				self.robot_orientation.add_to_translate([[robot_faces[0][0]-self.robot_x,robot_faces[0][1]-self.robot_y],
											   		   [robot_faces[1][0]-self.robot_x,robot_faces[1][1]-self.robot_y],
											   		   [robot_faces[2][0]-self.robot_x,robot_faces[2][1]-self.robot_y],
											   		   [robot_faces[3][0]-self.robot_x,robot_faces[3][1]-self.robot_y]]
														)

			elif ident == "h":
				human_faces = self.convert_corners_to_faces(corners,self.human_coords["h1"].x,self.human_coords["h1"].y)
				self.human_orientation["h1"].orientation = human_faces
				self.human_orientation["h1"].add_to_translate([[human_faces[0][0]-self.human_coords[ident].x,human_faces[0][1]-self.human_coords[ident].y],
										   				  [human_faces[1][0]-self.human_coords[ident].x,human_faces[1][1]-self.human_coords[ident].y],
										   				  [human_faces[2][0]-self.human_coords[ident].x,human_faces[2][1]-self.human_coords[ident].y],
										   				  [human_faces[3][0]-self.human_coords[ident].x,human_faces[3][1]-self.human_coords[ident].y]]
										   				  )

		self.update_content_position()

		self.updated_externally = True
		self.update()

	def mousePressEvent(self, event):
		x = event.x()
		y = event.y()
		if x > self.robot_x - 20 and x < self.robot_x + 20:
			if y > self.robot_y - 20 and y < self.robot_y + 20:
				self.robot_x = event.x()
				self.robot_y = event.y()
				self.update()
				self.robot_dragged = True
		for ident,coords in self.human_coords.items():
			currx = coords.x
			curry = coords.y
			if x > currx - 20 and x < currx + 20:
				if y > curry - 20 and y < curry + 20:
					coords.x = x
					coords.y = y
					self.update()
					self.human_dragged[ident] = True

		self.update_content_position()

		# if the calibrator is on
		if self.test_ui:
			if self.calibrator.is_calibrating:
				self.calibrator.increment()

	def mouseMoveEvent(self,event):
		if self.robot_dragged:
			self.robot_x = event.x()
			self.robot_y = event.y()
			self.update()
			if self.recording:
				self.gui.add_to_mom_signal.emit(["movement","True"])
		for ident,coords in self.human_coords.items():
			if self.human_dragged[ident]:
				coords.x = event.x()
				coords.y = event.y()
				self.update()
				if self.recording:
					self.gui.add_to_mom_signal.emit(["{}_movement".format(ident),"True"])

		self.update_content_position()

	def mouseReleaseEvent(self, event):
		if self.robot_dragged:
			self.robot_x = event.x()
			self.robot_y = event.y()
			self.update()
			self.robot_dragged = False
		for ident,coords in self.human_coords.items():
			if self.human_dragged[ident]:
				coords.x = event.x()
				coords.y = event.y()
				self.update()
				self.human_dragged[ident] = False

		self.update_content_position()

	def update_region_label_placements(self):
		for region in self.physical_layout.regions:
			width = math.ceil(len(region)*1.2)
			height = 4
			start_x,start_y = self.determine_region_label_placement(width+2,height+1,region)
			start_x_pix,start_y_pix = self.get_coords_from_grid_cell_origins(start_x+1,start_y-1)
			self.region_label_placements[region] = (start_x_pix,start_y_pix)

	'''
	CODE FOR HEAD NODDING, POINTING, and WAITING
	'''
	def begin_nod(self, direction):
		print("tilting")
		self.nod_lock.acquire()
		self.nod = direction
		self.nod_lock.release()

	def end_nod(self):
		self.nod_lock.acquire()
		self.nod = None
		self.nod_lock.release()

	def begin_point(self, angle):
		print("pointing: {}".format(angle))
		self.point_lock.acquire()
		self.point = angle
		self.point_lock.release()

	def end_point(self):
		self.point_lock.acquire()
		self.point = None
		self.point_lock.release()

	def begin_wait(self):
		print("waiting")
		self.wait_lock.acquire()
		self.wait = True
		self.wait_label.show()
		self.wait_lock.release()

	def end_wait(self):
		self.wait_lock.acquire()
		self.wait = False
		self.wait_label.hide()
		self.wait_lock.release()

	def highlight_position(self, x, y):
		self.static_calib_position = (x,y)

	def unhighlight_position(self):
		self.static_calib_position = None

	def paintEvent(self, event):

		qp = QPainter()
		qp.begin(self)

		# block if physical layout is locked
		#self.layout_lock.acquire()

		def highlight_region(region):
			qp.setFont(QFont('Monospace', 36, QFont.Bold))
			r=self.physical_layout.region_colors[region][0]
			g=self.physical_layout.region_colors[region][1]
			b=self.physical_layout.region_colors[region][2]
			qp.setPen(QPen(QColor(r*0.6,g*0.6,b*0.6),  4, Qt.SolidLine))
			start_points = self.region_label_placements[region]
			qp.drawText(start_points[0],start_points[1],region)

		# notify regions that the robot is in
		# PRECALCULATED LOCATIONS
		region = None
		try:
			rx,ry = self.get_grid_cell_from_coords(self.robot_x,self.robot_y)
			region = self.physical_layout.get_region_of_robot(rx,ry)
			if region is not None:
				highlight_region(region)
		except:
			pass

		# notify regions that the robot is POINTING TO
		self.point_lock.acquire()
		if self.point is not None:

			# calculate angle
			top = self.robot_orientation.top()
			bottom = self.robot_orientation.bottom()
			theta = math.atan2(bottom[1] - top[1], bottom[0] - top[0])
			theta *= 180/math.pi
			theta = theta if theta >= 0 else 360 + theta
			theta -= 90
			theta = theta if theta >= 0 else 360 + theta
			theta = int(round(theta))
			angle = self.point
			if angle < 0:
				angle = 360 + angle
			angle = angle + theta
			if angle > 359:
				angle = angle-360
			#print("point: {}".format(self.point))
			#print("theta: {}".format(theta))
			#print("angle: {}".format(angle))

			# draw the arrow
			qp.setPen(QPen(QColor(0, 0, 0), 8, Qt.SolidLine))
			qp.setBrush(QBrush(QColor(0,0,0), Qt.SolidPattern))
			line_start_x = self.robot_x
			line_start_y = self.robot_y

			line_end_x = line_start_x + (150 * math.sin(math.radians(angle)))
			line_end_y = line_start_y + (-150 * math.cos(math.radians(angle)))

			arrow_left_x = line_start_x + (98 * math.sin(math.radians(angle-15)))
			arrow_left_y = line_start_y + (-98 * math.cos(math.radians(angle-15)))

			arrow_right_x = line_start_x + (98 * math.sin(math.radians(angle+15)))
			arrow_right_y = line_start_y + (-98 * math.cos(math.radians(angle+15)))

			qp.drawLine(line_start_x,line_start_y,line_end_x,line_end_y)
			polygon = [QPoint(line_end_x,line_end_y),QPoint(arrow_left_x,arrow_left_y),QPoint(arrow_right_x,arrow_right_y)]
			polygon = QPolygon(polygon)
			qp.drawPolygon(polygon)
			
			# calculate region being pointed to
			# cast a ray
			distance = 10
			curr_x = self.robot_x 
			curr_y = self.robot_y
			curr_region = region
			region_cast = None

			while (curr_x < self.gui.width and curr_x >= 0 and curr_y < self.gui.height and curr_y >= 0):
				new_y = -1* math.cos(math.radians(angle))*distance + curr_y
				new_x = math.sin(math.radians(angle))*distance + curr_x
				cast_rx,cast_ry = self.get_grid_cell_from_coords(new_x,new_y)
				region_cast = self.physical_layout.get_region_of_robot(cast_rx,cast_ry)
				if region_cast is not None and region_cast != curr_region:
					highlight_region(region_cast)

					# send a region!
					if self.recording:
						self.gui.figurine_point_signal.emit(region_cast)

					break
				curr_x = new_x
				curr_y = new_y
		self.point_lock.release()

		self.nod_lock.acquire()
		if self.nod is not None:
			top = self.robot_orientation.top()
			bottom = self.robot_orientation.bottom()
			theta = math.atan2(bottom[1] - top[1], bottom[0] - top[0])
			theta *= 180/math.pi
			theta = theta if theta >= 0 else 360 + theta
			theta -= 90
			theta = theta if theta >= 0 else 360 + theta
			front_theta = int(round(theta))
			back_theta = front_theta + 180

			line_start_x = self.robot_x
			line_start_y = self.robot_y

			front_line_end_x = line_start_x + (150 * math.sin(math.radians(front_theta)))
			front_line_end_y = line_start_y + (-150 * math.cos(math.radians(front_theta)))

			front_arrow_left_x = line_start_x + (151 * math.sin(math.radians(front_theta-15)))
			front_arrow_left_y = line_start_y + (-151 * math.cos(math.radians(front_theta-15)))

			front_arrow_right_x = line_start_x + (151 * math.sin(math.radians(front_theta+15)))
			front_arrow_right_y = line_start_y + (-151 * math.cos(math.radians(front_theta+15)))

			qp.setPen(QPen(QColor(0, 0, 0), 8, Qt.SolidLine))
			qp.setBrush(QBrush(QColor(0,0,0), Qt.SolidPattern))

			back_line_end_x = line_start_x + (150 * math.sin(math.radians(back_theta)))
			back_line_end_y = line_start_y + (-150 * math.cos(math.radians(back_theta)))

			back_arrow_left_x = line_start_x + (151 * math.sin(math.radians(back_theta-15)))
			back_arrow_left_y = line_start_y + (-151 * math.cos(math.radians(back_theta-15)))

			back_arrow_right_x = line_start_x + (151 * math.sin(math.radians(back_theta+15)))
			back_arrow_right_y = line_start_y + (-151 * math.cos(math.radians(back_theta+15)))

			if self.nod == "down":
				qp.setPen(QPen(QColor(0, 0, 0), 8, Qt.SolidLine))
				qp.setBrush(QBrush(QColor(0,0,0), Qt.SolidPattern))
			else:
				qp.setPen(QPen(QColor(0, 0, 0, 25), 8, Qt.SolidLine))
				qp.setBrush(QBrush(QColor(0,0,0, 25), Qt.SolidPattern))

			qp.drawLine(line_start_x,line_start_y,front_line_end_x,front_line_end_y)
			qp.drawLine(front_arrow_left_x,front_arrow_left_y,front_arrow_right_x,front_arrow_right_y)
			#polygon = [QPoint(front_line_end_x,front_line_end_y),QPoint(front_arrow_left_x,front_arrow_left_y),QPoint(front_arrow_right_x,front_arrow_right_y)]
			#polygon = QPolygon(polygon)
			#qp.drawPolygon(polygon)

			
			if self.nod == "down":
				qp.setPen(QPen(QColor(0, 0, 0, 25), 8, Qt.SolidLine))
				qp.setBrush(QBrush(QColor(0,0,0, 25), Qt.SolidPattern))
			else:
				qp.setPen(QPen(QColor(0, 0, 0), 8, Qt.SolidLine))
				qp.setBrush(QBrush(QColor(0,0,0), Qt.SolidPattern))

			qp.drawLine(line_start_x,line_start_y,back_line_end_x,back_line_end_y)
			qp.drawLine(back_arrow_left_x,back_arrow_left_y,back_arrow_right_x,back_arrow_right_y)
			
		self.nod_lock.release()

		self.wait_lock.acquire()
		if self.wait: # var is true or false
			self.wait_label.move(self.robot_x + 70, self.robot_y + 70)
		self.wait_lock.release()

		# notify close objects to the robot
		close_objects = self.physical_layout.get_objects_close_to_robot(rx,ry)
		qp.setPen(QPen(QColor(186, 31, 234, 150), 8, Qt.SolidLine))
		for obj in close_objects:
			obj_center = self.physical_layout.layout_centers[obj]
			xpix,ypix = self.get_coords_from_grid_cell_center(obj_center[0],obj_center[1])
			# draw line from robot to grid cell
			qp.drawLine(self.robot_x,self.robot_y,xpix,ypix)

		# notify if robot close to human
		x,y = self.get_grid_cell_from_coords(self.human_coords["h1"].x,self.human_coords["h1"].y)
		if self.physical_layout.determine_if_objects_close(rx,ry,x,y):
			qp.drawPixmap(self.robot_x-75,self.robot_y-75,150,150,self.proximal_pic)
			qp.drawPixmap(self.human_coords["h1"].x-75,self.human_coords["h1"].y-75,150,150,self.proximal_pic)

		# paint the agents' position history
		multiplier = 255*1.0/49
		for agent,position_queue in self.position_queue.items():
			for i in range(len(position_queue)-1):
				#alpha = (20-(i+1))*multiplier
				alpha = 255
				width = (50-(i+1))/3.0
				qp.setPen(QPen(QColor(self.position_color[agent][0],self.position_color[agent][1],self.position_color[agent][2],alpha),  width, Qt.SolidLine))
				pos1 = position_queue[i]
				pos2 = position_queue[i+1]
				if pos1 != pos2:
					qp.drawLine(pos1[0],pos1[1],pos2[0],pos2[1])

		# paint position of robot
		qp.setPen(QPen(QColor("#969c9e"),  8, Qt.SolidLine))
		qp.setBrush(QBrush(QColor("#969c9e"), Qt.SolidPattern))
		qp.drawEllipse(self.robot_x-20, self.robot_y-20, 40, 40)

		# paint position of all other people in scene
		qp.setPen(QPen(QColor("#4a60a1"),  8, Qt.SolidLine))
		qp.setBrush(QBrush(QColor("#4a60a1"), Qt.SolidPattern))
		for ident,coords in self.human_coords.items():
			qp.drawEllipse(coords.x-20, coords.y-20, 40, 40)

		'''
		HIGHLIGHING THE HUMAN/ROBOT WHEN THEY ARE SUPPOSED TO SAY/DO SOMETHING
		'''

		# highlight robot say if necessary
		qp.setPen(QPen(QColor(253, 11, 11, 150),  8, Qt.SolidLine))
		qp.setBrush(QBrush(QColor(253, 11, 11, 150), Qt.SolidPattern))
		if len(self.highlighting_robot_say) > 0:
			qp.drawEllipse(self.robot_x-80,self.robot_y-80,160,160)

		# highlight human say if necessary
		for ident,coords in self.human_coords.items():
			if len(self.highlighting_human_say) > 0:
				qp.drawEllipse(coords.x-80,coords.y-80,160,160)

		# highlighting robot do if necessary
		qp.setPen(QPen(QColor(186, 31, 234, 150), 8, Qt.SolidLine))
		qp.setBrush(QBrush(QColor(186, 31, 234, 150), Qt.SolidPattern))
		if len(self.highlighting_robot_do) > 0:
			qp.drawEllipse(self.robot_x-80,self.robot_y-80,160,160)

		# highlight human do if necessary
		for ident,coords in self.human_coords.items():
			if len(self.highlighting_human_do) > 0:
				qp.drawEllipse(coords.x-80,coords.y-80,160,160)

		# determine the orientation of both the human and the robot
		def translate_orientation(agent_x,agent_y,agent_orientation):
			# translate the current orientation points without rotating them
			#self.orientation_lock.acquire()
			translate = agent_orientation.translate
			#self.orientation_lock.release()
			agent_orientation.settop([agent_x+translate[0][0],agent_y+translate[0][1]])
			agent_orientation.setright([agent_x+translate[1][0],agent_y+translate[1][1]])
			agent_orientation.setbottom([agent_x+translate[2][0],agent_y+translate[2][1]])
			agent_orientation.setleft([agent_x+translate[3][0],agent_y+translate[3][1]])

		def calculate_orientation(ident,agent_orientation):
			# calculate new corners
			center_x = self.position_queue[ident][0][0]
			center_y = self.position_queue[ident][0][1]

			bottom_x_prime = self.position_queue[ident][10][0]
			bottom_y_prime = self.position_queue[ident][10][1]

			# target distance of each point from position
			d = 60.0

			# bottom side first
			d_prime = self.get_distance(center_x,bottom_x_prime,center_y,bottom_y_prime)
			bottom_x = center_x + (d/d_prime)*(bottom_x_prime - center_x)
			bottom_y = center_y + (d/d_prime)*(bottom_y_prime - center_y)

			# now left side
			left_x = center_x + (center_y - bottom_y)
			left_y = center_y + (bottom_x - center_x)

			# now top side
			top_x = center_x + (center_x - bottom_x)
			top_y = center_y + (center_y - bottom_y)

			# and finally right side
			right_x = center_x + (bottom_y - center_y)
			right_y = center_y + (center_x - bottom_x)

			agent_orientation.orientation = [[top_x,top_y],[right_x,right_y],[bottom_x,bottom_y],[left_x,left_y]]
			#self.orientation_lock.acquire()
			agent_orientation.add_to_translate([[top_x-center_x,top_y-center_y],
										   [right_x-center_x,right_y-center_y],
										   [bottom_x-center_x,bottom_y-center_y],
										   [left_x-center_x,left_y-center_y]])
			#self.orientation_lock.release()

		# case 1: we are not moving -- the orientation is the same as the stored orientation
		if not self.is_moving("r"):
			translate_orientation(self.robot_x,self.robot_y,self.robot_orientation)

		# case 2: we are moving
		# robot first:
		else:
			calculate_orientation("r",self.robot_orientation)

		# human next:
		if not self.is_moving("h1"):
			translate_orientation(self.human_coords["h1"].x,self.human_coords["h1"].y,self.human_orientation["h1"])
		else:
			calculate_orientation("h1",self.human_orientation["h1"])

		if self.recording:
			position_array = ["position"]
			for obj in close_objects:
				position_array.append(obj)
			if region is not None:
				position_array.append(region)

			for human,coords in self.human_coords.items():
				x,y = self.get_grid_cell_from_coords(coords.x,coords.y)
				h_position_array = ["{}_position".format(human)]
				close_objects = self.physical_layout.get_objects_close_to_robot(x,y)
				for obj in close_objects:
					h_position_array.append(obj)

				if self.physical_layout.determine_if_objects_close(rx,ry,x,y):
					r_orientation = self.get_other_orientation_with_respect_to_agent(coords.x,coords.y,self.robot_orientation)
					h_orientation = self.get_other_orientation_with_respect_to_agent(self.robot_x,self.robot_y,self.human_orientation[human])
					h_position_array.append("robot_{}".format(h_orientation))
					position_array.append("{}_{}".format(human,r_orientation))
				h_xpos_array = ["h_xpos",str(x)]
				h_ypos_array = ["h_ypos",str(y)]
				try:
					self.gui.add_to_mom_signal.emit(h_xpos_array)
					self.gui.add_to_mom_signal.emit(h_ypos_array)
					self.gui.add_to_mom_signal.emit(h_position_array)
					self.gui.record_mom_signal.emit(human,coords.x,coords.y,90)
				except:
					print("signal exception")
					exit(1)

			# send the orientation of the robots
			top = self.robot_orientation.top()
			bottom = self.robot_orientation.bottom()
			theta = math.atan2(bottom[1] - top[1], bottom[0] - top[0])
			theta *= 180/math.pi
			theta = theta if theta >= 0 else 360 + theta
			theta -= 90
			theta = theta if theta >= 0 else 360 + theta
			theta = int(round(theta))
			theta_array = ["rotation",str(theta)]
			xpos_array = ["xpos",str(rx)]
			ypos_array = ["ypos",str(ry)]

			try:
				self.gui.add_to_mom_signal.emit(position_array)
				self.gui.add_to_mom_signal.emit(theta_array)
				self.gui.add_to_mom_signal.emit(xpos_array)
				self.gui.add_to_mom_signal.emit(ypos_array)
				self.gui.record_mom_signal.emit("r",self.robot_x,self.robot_y,theta)
			except:
				print("signal exception")
				exit(1)

			'''
			self.gui.update_mode_stack_signal.emit("determine_state&&&position&&&none")
			#self.mode_stack.update(("determine_state","position",close_objects))
			self.gui.update_mode_stack_signal.emit("determine_moment&&&{}&&&{}".format(self.robot_x,self.robot_y))
			#self.mode_stack.update(("determine_moment",self.robot_x,self.robot_y))
			'''

		# post a warning that no physical environment has been loaded
		if self.physical_layout.get_grid_size() == 0:
			qp.drawPixmap((self.width()/2)-250,(self.height()/2)-200,500,500,self.warning_pic)
			qp.setFont(QFont('Arial', 50))
			qp.setPen(QPen(QColor(59, 148, 174,255)))
			qp.drawText((self.width()/2)-580,(self.height()/2)+300,"please edit or load physical environment")

		# handle clicks
		qp.setPen(QPen(QColor("#ff0000"),  4, Qt.SolidLine))
		qp.setBrush(QBrush(QColor(255,255,255,0), Qt.SolidPattern))
		for click_time, coords_dict in self.active_clicks.items():
			for coords_key,coords_val in coords_dict.items():
				qp.drawEllipse(coords_val[0]-30, coords_val[1]-30, 60, 60)

		# handle calibration
		if self.calibrator.is_calibrating:

			qp.setPen(QPen(QColor("#ff0000"),  8, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#ff0000"), Qt.SolidPattern))

			step = self.calibrator.step

			center_x = self.width()/2.0
			center_y = self.height()/2.0
			if step == 1:   # top left
				x = center_x/5.0
				y = center_y/5.0
				qp.drawEllipse(x-15, y-15, 30, 30)
			elif step == 2: # top right
				x = center_x + (center_x*4.0/5.0)
				y = center_y/5.0
				qp.drawEllipse(x-15, y-15, 30, 30)
			elif step == 3: # bottom right
				x = center_x + (center_x*4.0/5.0)
				y = center_y + (center_y*4.0/5.0)
				qp.drawEllipse(x-15, y-15, 30, 30)
			elif step == 4: # bottom left
				x = center_x/5.0
				y = center_y + (center_y*4.0/5.0)
				qp.drawEllipse(x-15, y-15, 30, 30)
			elif step == 5: # center
				qp.drawEllipse(center_x, center_y, 30, 30)

		if self.static_calib_position is not None:
			x = self.static_calib_position[0] - 25
			y = self.static_calib_position[1] - 25

			qp.drawEllipse(x, y, 50, 50)

		# display extra stuff if paused
		if self.paused:
			self.display_path_lock.acquire()

			# what point is the robot closest to on the path?
			robot_rewind_idx = -1
			if len(self.robot_display_path) > 0:
				rclosest_point_idx = len(self.robot_display_path)-1
				closest_point = self.robot_display_path[rclosest_point_idx]
				curr_closest_point_dist = self.get_distance(closest_point[0],self.robot_x,closest_point[1],self.robot_y)
				start_point_dist = curr_closest_point_dist
				for i in range(len(self.robot_display_path)):
					point = self.robot_display_path[i]
					dist = self.get_distance(point[0],self.robot_x,point[1],self.robot_y)
					if dist < curr_closest_point_dist and start_point_dist > 20:
						rclosest_point_idx = i
						curr_closest_point_dist = dist
						robot_rewind_idx = i

			# what point is the human closest to on the path?
			human_rewind_idx = -1
			if len(self.human_display_path) > 0:
				hclosest_point_idx = len(self.human_display_path)-1
				closest_point = self.human_display_path[hclosest_point_idx]
				curr_closest_point_dist = self.get_distance(closest_point[0],self.human_coords["h1"].x,closest_point[1],self.human_coords["h1"].y)
				start_point_dist = curr_closest_point_dist
				for i in range(len(self.human_display_path)):
					point = self.human_display_path[i]
					dist = self.get_distance(point[0],self.human_coords["h1"].x,point[1],self.human_coords["h1"].y)
					if dist < curr_closest_point_dist and start_point_dist > 20:
						hclosest_point_idx = i
						curr_closest_point_dist = dist
						human_rewind_idx = i

			if human_rewind_idx == -1:
				self.rewind_idx = robot_rewind_idx
			elif robot_rewind_idx == -1:
				self.rewind_idx = human_rewind_idx
			else:
				self.rewind_idx = min(robot_rewind_idx,human_rewind_idx)
			qp.setPen(QPen(QColor(self.position_color["r"][0],self.position_color["r"][1],self.position_color["r"][2],255), 8, Qt.SolidLine))
			for i in range(len(self.robot_display_path)-1):
				if i == rclosest_point_idx:
					qp.setPen(QPen(QColor(self.position_color["r"][0],self.position_color["r"][1],self.position_color["r"][2],100), 8, Qt.SolidLine))
				point_1 = self.robot_display_path[i]
				point_2 = self.robot_display_path[i+1]
				start_x = point_1[0]
				start_y = point_1[1]
				end_x = point_2[0]
				end_y = point_2[1]
				qp.drawLine(start_x,start_y,end_x,end_y)

			qp.setPen(QPen(QColor(self.position_color["h1"][0],self.position_color["h1"][1],self.position_color["h1"][2],255), 8, Qt.SolidLine))
			for i in range(len(self.human_display_path)-1):
				if i == hclosest_point_idx:
					qp.setPen(QPen(QColor(self.position_color["h1"][0],self.position_color["h1"][1],self.position_color["h1"][2],100), 8, Qt.SolidLine))
				point_1 = self.human_display_path[i]
				point_2 = self.human_display_path[i+1]
				start_x = point_1[0]
				start_y = point_1[1]
				end_x = point_2[0]
				end_y = point_2[1]
				qp.drawLine(start_x,start_y,end_x,end_y)

			self.display_path_lock.release()

		# purely for debugging -- draw robot orientation
		if self.test_ui:
			qp.setFont(QFont('Courier', 12))
			qp.setPen(QPen(QColor("#000000"),  4, Qt.SolidLine))
			qp.drawText(10,20,"TEST UI MODE ACTIVE")
			qp.drawText(10,40,"robot coords: ({},{})".format(rx,ry))
			qp.drawText(10,60,"robot xy crd: ({},{})".format(self.robot_x,self.robot_y))
			qp.drawText(10,80,"robot region: {}".format("--" if region is None else region))

			# draw a faint grid
			qp.setPen(QPen(QColor(220,220,220,50), 1, Qt.SolidLine))
			for i in range(self.xdim):
				start_x,start_y = self.get_coords_from_grid_cell_origins(i,0)
				end_x,end_y = self.get_coords_from_grid_cell_origins(i,self.ydim)
				qp.drawLine(start_x,start_y,end_x,end_y)
			for i in range(self.ydim):
				start_x,start_y = self.get_coords_from_grid_cell_origins(0,i)
				end_x,end_y = self.get_coords_from_grid_cell_origins(self.xdim,i)
				qp.drawLine(start_x,start_y,end_x,end_y)

			# draw the circumference of the puck
			# if test ui, calculate the height and width of the virtual pucks
			self.puck_width = self.puck_height = None
			if self.test_ui:
				self.puck_width = (1.5/24)*self.width()
				self.puck_height = (1.5/18)*self.height()
			qp.setPen(QPen(QColor(220,220,220,100),4,Qt.SolidLine))
			qp.setBrush(QBrush(QColor(220,220,220,100),Qt.SolidPattern))
			qp.drawEllipse(self.robot_x - self.puck_width/2.0,self.robot_y-self.puck_height/2.0,self.puck_width,self.puck_height)
			qp.drawEllipse(self.human_coords["h1"].x - self.puck_width/2.0,self.human_coords["h1"].y-self.puck_height/2.0,self.puck_width,self.puck_height)

			# draw angle of robot
			top = self.robot_orientation.top()
			bottom = self.robot_orientation.bottom()
			theta = math.atan2(bottom[1] - top[1], bottom[0] - top[0])
			theta *= 180/math.pi
			theta = theta if theta >= 0 else 360 + theta
			theta -= 90
			theta = theta if theta >= 0 else 360 + theta
			theta = int(round(theta))
			qp.setFont(QFont('Courier', 12))
			qp.setPen(QPen(QColor("#000000"),  4, Qt.SolidLine))
			qp.drawText(self.robot_x,self.robot_y,str(theta))

			top = self.robot_orientation.top()
			bottom = self.robot_orientation.bottom()
			right = self.robot_orientation.right()
			left = self.robot_orientation.left()
			qp.setPen(QPen(QColor("#ff0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#ff0000"), Qt.SolidPattern))
			qp.drawEllipse(top[0]-2,top[1]-2,4,4)
			qp.setPen(QPen(QColor("#f0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#f0000"), Qt.SolidPattern))
			qp.drawEllipse(bottom[0]-2,bottom[1]-2,4,4)
			qp.setPen(QPen(QColor("#f0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#f0000"), Qt.SolidPattern))
			qp.drawEllipse(left[0]-2,left[1]-2,4,4)
			qp.setPen(QPen(QColor("#00ff00"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#00ff00"), Qt.SolidPattern))
			qp.drawEllipse(right[0]-2,right[1]-2,4,4)

			# purely for debugging -- draw human orientation
			top = self.human_orientation["h1"].top()
			bottom = self.human_orientation["h1"].bottom()
			right = self.human_orientation["h1"].right()
			left = self.human_orientation["h1"].left()
			qp.setPen(QPen(QColor("#ff0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#ff0000"), Qt.SolidPattern))
			qp.drawEllipse(top[0]-2,top[1]-2,4,4)
			qp.setPen(QPen(QColor("#f0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#f0000"), Qt.SolidPattern))
			qp.drawEllipse(bottom[0]-2,bottom[1]-2,4,4)
			qp.setPen(QPen(QColor("#f0000"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#f0000"), Qt.SolidPattern))
			qp.drawEllipse(left[0]-2,left[1]-2,4,4)
			qp.setPen(QPen(QColor("#00ff00"),  4, Qt.SolidLine))
			qp.setBrush(QBrush(QColor("#00ff00"), Qt.SolidPattern))
			qp.drawEllipse(right[0]-2,right[1]-2,4,4)

		# unblock
		#self.layout_lock.release()

	def is_moving(self,agent):
		if len(self.position_queue[agent]) < 11:
			return False
		curr_pos = self.position_queue[agent][0]
		prev_pos = self.position_queue[agent][10]

		curr_x = curr_pos[0]
		curr_y = curr_pos[1]
		prev_x = prev_pos[0]
		prev_y = prev_pos[1]

		dist = self.get_distance(curr_x,prev_x,curr_y,prev_y)

		if dist > int(round(0.3*self.xdim)):  # previously just 2
		#if dist > int(round(0.06*self.xdim)):
			return True
		return False

	def get_distance(self,x1,x2,y1,y2):

		diff_x = x1 - x2
		diff_y = y1 - y2

		sq_x = diff_x**2
		sq_y = diff_y**2

		summa = sq_x + sq_y

		return math.sqrt(summa)

	def get_other_orientation_with_respect_to_agent(self,agent_x,agent_y,other_orientation):
		top = other_orientation.top()
		right = other_orientation.right()
		bottom = other_orientation.bottom()
		left = other_orientation.left()

		closest_orientation = "top"
		shortest_distance = self.get_distance(agent_x,top[0],agent_y,top[1])
		distance = self.get_distance(agent_x,right[0],agent_y,right[1])
		if distance < shortest_distance:
			shortest_distance = distance
			closest_orientation = "left"
		distance = self.get_distance(agent_x,bottom[0],agent_y,bottom[1])
		if distance < shortest_distance:
			shortest_distance = distance
			closest_orientation = "bottom"
		distance = self.get_distance(agent_x,left[0],agent_y,left[1])
		if distance < shortest_distance:
			shortest_distance = distance
			closest_orientation = "right"

		return closest_orientation

	def convert_corners_to_faces(self,corners,agent_x,agent_y):
		top_left = corners[0]
		top_right = corners[1]
		bottom_right = corners[2]
		bottom_left = corners[3]

		# translate corners so that center is agent
		leftmost_x = min(top_left[0],top_right[0],bottom_right[0],bottom_left[0])
		rightmost_x = max(top_left[0],top_right[0],bottom_right[0],bottom_left[0])
		corners_center_x = leftmost_x + (rightmost_x - leftmost_x)/2.0

		lower_y = min(top_left[1],top_right[1],bottom_right[1],bottom_left[1])
		upper_y = max(top_left[1],top_right[1],bottom_right[1],bottom_left[1])
		corners_center_y = lower_y + (upper_y - lower_y)/2.0

		trans_x = agent_x - corners_center_x
		trans_y = agent_y - corners_center_y

		top_left[0] += trans_x
		top_left[1] += trans_y
		top_right[0] += trans_x
		top_right[1] += trans_y
		bottom_right[0] += trans_x
		bottom_right[1] += trans_y
		bottom_left[0] += trans_x
		bottom_left[1] += trans_y

		# perform conversion
		top_x = top_left[0] + (top_right[0] - top_left[0])/2.0
		top_y = top_left[1] + (top_right[1] - top_left[1])/2.0

		right_x = top_right[0] + (bottom_right[0] - top_right[0])/2.0
		right_y = top_right[1] + (bottom_right[1] - top_right[1])/2.0

		bottom_x = bottom_left[0] + (bottom_right[0] - bottom_left[0])/2.0
		bottom_y = bottom_left[1] + (bottom_right[1] - bottom_left[1])/2.0

		left_x = top_left[0] + (bottom_left[0] - top_left[0])/2.0
		left_y = top_left[1] + (bottom_left[1] - top_left[1])/2.0

		# scale each face
		d = 60.0
		top_d_prime = self.get_distance(agent_x,top_x,agent_y,top_y)
		top_d_ratio = d/top_d_prime
		top_x_final = agent_x + top_d_ratio*(top_x - agent_x)
		top_y_final = agent_y + top_d_ratio*(top_y - agent_y)

		right_d_prime = self.get_distance(agent_x,right_x,agent_y,right_y)
		right_d_ratio = d/right_d_prime
		right_x_final = agent_x + right_d_ratio*(right_x - agent_x)
		right_y_final = agent_y + right_d_ratio*(right_y - agent_y)

		bottom_d_prime = self.get_distance(agent_x,bottom_x,agent_y,bottom_y)
		bottom_d_ratio = d/bottom_d_prime
		bottom_x_final = agent_x + bottom_d_ratio*(bottom_x - agent_x)
		bottom_y_final = agent_y + bottom_d_ratio*(bottom_y - agent_y)

		left_d_prime = self.get_distance(agent_x,left_x,agent_y,left_y)
		left_d_ratio = d/left_d_prime
		left_x_final = agent_x + left_d_ratio*(left_x - agent_x)
		left_y_final = agent_y + left_d_ratio*(left_y - agent_y)

		return [[top_x_final,top_y_final],[right_x_final,right_y_final],[bottom_x_final,bottom_y_final],[left_x_final,left_y_final]]

	def highlight_agent(self,agent,typ,content_id):
		if agent == "robot":
			if typ == "say":
				if content_id in self.highlighting_robot_say:
					self.highlighting_robot_say.remove(content_id)
				else:
					self.highlighting_robot_say.append(content_id)
			else:
				if content_id in self.highlighting_robot_do:
					self.highlighting_robot_do.remove(content_id)
				else:
					self.highlighting_robot_do.append(content_id)
		else:
			if typ == "say":
				if content_id in self.highlighting_human_say:
					self.highlighting_human_say.remove(content_id)
				else:
					self.highlighting_human_say.append(content_id)
			else:
				if content_id in self.highlighting_human_do:
					self.highlighting_human_do.remove(content_id)
				else:
					self.highlighting_human_do.append(content_id)

	def display_content(self,agent,speech_or_action,content,content_category,content_id,unfinished):

		'''
		3 cases
		 #1 unfinished content, where there previously was no content OR was different content (differentiated by content id)
		 	- choose new location
		 	- restart timer
		 #2 unfinished content, but the previous unfinished content had the same id
		 	- do NOT choose new location
		 	- restart timer
		 #3 finished content 
		 	- choose new location
		 	- restart timer
		'''

		# case 1
		choose_new_loc = False
		if unfinished and content_id != self.content_id:
			self.content_id = content_id
			choose_new_loc = True

		# case 2
		elif unfinished and content_id == self.content_id:
			pass # all we have to do is restart the timer

		# case 3
		elif not unfinished:
			self.content_id = content_id # just in case we never received content id from unfinished
			choose_new_loc = True

		# process content for actions/speech
		if speech_or_action == "speech":
			content = "\"" + content + "\""
		else:
			content = "(" + content + ")"

		# determine the height of the box
		num_lines = 1
		previous_space_idx = -1
		line_idx = 0
		line_break_idxs = []
		max_line_length = 0
		for i in range(len(content)):
			char = content[i]
			if char == " ":
				previous_space_idx = i

			if line_idx == 25:
				line_len = previous_space_idx - (line_break_idxs[-1] if len(line_break_idxs)>0 else 0)
				if line_len > max_line_length:
					max_line_length = line_len
				line_break_idxs.append(previous_space_idx)
				line_idx = i - previous_space_idx
				num_lines +=1
				previous_space_idx = -1

			line_idx += 1

		for idx in line_break_idxs:
			content = content[:idx] + "\n" + content[idx+1:]
		height = 32*num_lines + 10
		height += 80 # add the height for the buttons

		if max_line_length == 0:
			max_line_length = len(content)

		# determine width
		if max_line_length < 25:
			width = 420 * max(0.35,(max_line_length*1.0/25))
		else:
			width = 420
		#print(max_line_length)

		# add a category if present
		if content_category != "":
			content = "{}\n{}".format(content_category.upper(),content)
			height += 32

		if choose_new_loc:
			# determine placement of box using grid cells
			if agent == "robot":
				agentx = self.robot_x
				agenty = self.robot_y
				other_agentx = self.human_coords["h1"].x
				other_agenty = self.human_coords["h1"].y
				self.content_agent = "robot"
			elif agent == "human":
				agentx = self.human_coords["h1"].x
				agenty = self.human_coords["h1"].y
				other_agentx = self.robot_x
				other_agenty = self.robot_y
				self.content_agent = "human"

			# IF unfinished we must conservatively assume that the width will be MAX
			if unfinished:
				x,y = self.determine_box_placement(420,height,agentx,agenty,other_agentx,other_agenty)
			else:
				x,y = self.determine_box_placement(width,height,agentx,agenty,other_agentx,other_agenty)
			if agent == "robot":
				self.content_x = x - self.robot_x
				self.content_y = y - self.robot_y
			else:
				self.content_x = x - self.human_coords["h1"].x
				self.content_y = y - self.human_coords["h1"].y

		# determine the position of the box
		self.content_label.setText(content)
		if agent == "robot":
			x = self.content_x + self.robot_x
			y = self.content_y + self.robot_y
		else:
			x = self.content_x + self.human_coords["h1"].x
			y = self.content_y + self.human_coords["h1"].y
		self.content_label.setGeometry(x,y,width,height)
		if unfinished:
			self.content_flag.hide()
			self.content_delete.hide()
		else:
			self.content_flag.move(width - 120,height - 60)
			self.content_flag.show()
			self.content_delete.move(width - 60,height - 60)
			self.content_delete.show()
		self.content_label.show()
		self.content_timer_val = 0
		self.update()

	def determine_region_label_placement(self,grid_width,grid_height,region):

		def calculate_viability(gridx,gridy):
			if grid_x + grid_width > self.xdim or grid_y - grid_height < 0:
				return False
			return True

		def calculate_cost(gridx,gridy):

			cost = 0
			for x in range(gridx,gridx+grid_width):
				for y in range(gridy-grid_height,gridy):

					cell_region = self.physical_layout.region_cell_data[(x,y)]
					objects = self.physical_layout.cell_data[(x,y)]
					if cell_region == region:
						cost -= 1

					for obj in objects:
						cost += 1

			return cost

		grid_x = 0
		grid_y = 5
		best_grid_x = 0
		best_grid_y = 5

		best_cost = calculate_cost(grid_x,grid_y)
		for grid_x in range(self.xdim):
			for grid_y in range(5,self.ydim):
				is_viable = calculate_viability(grid_x,grid_y)
				if not is_viable:
					continue
				curr_cost = calculate_cost(grid_x,grid_y)
				if curr_cost < best_cost:
					best_cost = curr_cost
					best_grid_x = grid_x
					best_grid_y = grid_y

		return best_grid_x,best_grid_y

	def determine_box_placement(self,width,height,agentx,agenty,other_agentx,other_agenty):

		'''
		keeping track of the best grid placement score
		'''
		# intersecting with the robot is not allowed
		# intersecting with the edges of the screen is not allowed
		# intersecting self incurs a cost of 0.75
		# intersecting with the other agent incurs a cost of 0.75
		# intersecting with objects in the scene incurs a cost of 0.25

		# helper methods
		def calculate_viability(placement_x,placement_y):
			if placement_x + width > self.width() or placement_y + height > self.height():
				return False
			return True

		def calculate_cost(placement_x,placement_y):
			max_distance = math.sqrt(self.width()**2 + self.height()**2)
			new_center_x = int(round(placement_x + width/2.0))
			new_center_y = int(round(placement_y + height/2.0))
			sect_self = 0
			sect_other = 0
			is_above = 0
			# determine if intersecting self or other
			if agentx+50 > placement_x and agentx-50 < placement_x + width:
				if agenty+50 > placement_y and agenty-50 < placement_y + height:
					sect_self = 0.75
			if other_agentx+50 > placement_x and other_agentx-50 < placement_x + width:
				if other_agenty+50 > placement_y and other_agenty-50 < placement_y + height:
					sect_other = 0.75
			if placement_y < agenty:
				is_above = 0.3
			# get cost of new coordinate
			distance_x = (agentx - new_center_x)**2
			distance_y = (agenty - new_center_y)**2
			distance = math.sqrt(distance_x + distance_y)
			return distance/max_distance + sect_self + sect_other + is_above

		# starting placement is the best
		grid_x = 0
		grid_y = 0
		best_grid_x = 0
		best_grid_y = 0

		curr_pixel_x,curr_pixel_y = self.get_coords_from_grid_cell_origins(grid_x,grid_y)
		curr_cost = calculate_cost(curr_pixel_x,curr_pixel_y)
		best_cost = curr_cost

		for grid_x in range(self.xdim):
			for grid_y in range(self.ydim):
				curr_pixel_x,curr_pixel_y = self.get_coords_from_grid_cell_origins(grid_x,grid_y)
				is_viable = calculate_viability(curr_pixel_x,curr_pixel_y)
				if not is_viable:
					continue
				curr_cost = calculate_cost(curr_pixel_x,curr_pixel_y)
				if curr_cost < best_cost:
					best_cost = curr_cost
					best_grid_x = grid_x
					best_grid_y = grid_y


		best_pixel_x,best_pixel_y = self.get_coords_from_grid_cell_origins(best_grid_x,best_grid_y)
		return best_pixel_x,best_pixel_y

	def get_grid_cell_from_coords(self, x_abs, y_abs):
		x = int(math.floor((x_abs*self.xdim)/self.width()))
		y = int(math.floor((y_abs*self.ydim)/self.height()))
		x = min(max(0,x),self.xdim-1)
		y = min(max(0,y),self.ydim-1)
		return x,y

	def get_coords_from_grid_cell_center(self,x_cell,y_cell):
		pixels_per_xdim = self.width()*1.0/self.xdim
		pixels_per_ydim = self.height()*1.0/self.ydim
		return (x_cell+0.5)*pixels_per_xdim,(y_cell+0.5)*pixels_per_ydim

	def get_coords_from_grid_cell_origins(self,x_cell,y_cell):
		pixels_per_xdim = self.width()*1.0/self.xdim
		pixels_per_ydim = self.height()*1.0/self.ydim
		return x_cell*pixels_per_xdim,y_cell*pixels_per_ydim

	def calculate_has_neighbors(self, orig_cell, neighbor_cell):
		'''	
		Calculate whether the orig cell has a neighbor with different environment properties than itself.

		E.g. if the orig cell is a countertop and its left neighbor is a countertop and coffee maker, than return False
		'''
		orig_cell_items = self.physical_layout.cell_data[orig_cell]
		neighbor_cell_items = self.physical_layout.cell_data[neighbor_cell]

		if len(orig_cell_items) != len(neighbor_cell_items):
			return False

		is_identical = True
		for item in orig_cell_items:
			found_item = False
			for neigh_item in neighbor_cell_items:
				if item == neigh_item:
					found_item = True

			if not found_item:
				is_identical = False
				break

		return is_identical

	def ui_update_timer(self):

		while True:
			time.sleep(0.01)

			# handle content labels
			if self.content_timer_val < 5:
				self.content_timer_val += 0.01
			else:
				# reset and hide the content label
				# start with the buttons

				if self.content_delete.isChecked():
					self.content_delete.toggle()

					# must now throw away the content!
					print("throwing away content")
					text = self.content_label.text() # get rid of the intent label at the top
					text = text[text.index("\n")+1:] # get rid of the intent label at the top
					text = text[1:-1]   # get rid of the quotations and parentheses
					text = text.replace("\n", " ")
					self.gui.throwaway_speech_signal.emit(text)

				if self.content_flag.isChecked():
					self.content_flag.toggle()

					# must now throw away the content!
					print("flagging content")
					text = self.content_label.text() # get rid of the intent label at the top
					text = text[text.index("\n")+1:] # get rid of the intent label at the top
					text = text[1:-1]   # get rid of the quotations and parentheses
					text = text.replace("\n", " ")
					self.gui.flag_speech_signal.emit(text)

				self.content_label.hide()



			# handle robot position tracker
			for agent in ["r","h1"]:
				while len(self.position_queue[agent]) >= 50:
					del self.position_queue[agent][-1]
				if agent == "r":
					self.position_queue[agent].insert(0,(self.robot_x,self.robot_y))
				else:
					self.position_queue[agent].insert(0,(self.human_coords[agent].x,self.human_coords[agent].y))

			# handle notifications
			#self.notification_label_lock.acquire()
			#if self.notification_label.text is not None:
			#	self.notification_label.increment()
			#	if self.notification_label.itr > 700:
			#		self.notification_label.text = None
			#		self.notification_label.hide()
			#self.notification_label_lock.release()

			# update the UI
			self.update()

	def flag_content(self):
		print("flagging content")
		
		# if the delete button is toggled, untoggle it
		if self.content_delete.isChecked():
			self.content_delete.toggle()

	def delete_content(self):
		print("deleting content")
		if self.content_flag.isChecked():
			self.content_flag.toggle()

	def click(self, x, y, click_time):
		self.click_lock.acquire()
		click_x = x + self.offset_x
		click_y = y + self.offset_y
		self.active_clicks[click_time] = {}
		self.active_clicks[click_time][(x,y)] = (x,y)
		self.mouse.position = (click_x,click_y)
		print("clicking at {} - {} ({})".format(x,y,click_time))
		self.mouse.click(Button.left,1)
		self.click_lock.release()

		# if the calibrator is on
		if not self.test_ui:
			if self.calibrator.is_calibrating:
				self.calibrator.increment()

	def release_click(self,x,y,click_time):
		self.click_lock.acquire()
		if click_time in self.active_clicks and (x,y) in self.active_clicks[click_time]:
			print("removing click {}".format(click_time))
			del self.active_clicks[click_time][(x,y)]
			if len(self.active_clicks[click_time].keys()) == 0:
				del self.active_clicks[click_time]
		else:
			# TODO: possibly may have to record clicks that have been removed before being added
			# (I suppose this is possible?)
			pass
		self.click_lock.release()

class NamedCoords:

	def __init__(self,ident,x,y):
		self.ident = ident
		self.x = x
		self.y = y

class Calibrator:

	def __init__(self, pane):
		self.step = 0
		self.pane = pane
		self.is_calibrating = False

	def begin_calibration(self):
		self.is_calibrating = True

	def increment(self):
		self.step += 1
		self.pane.update()
		if self.step == 6:
			self.is_calibrating = False
			self.step = 0

class AgentOrientation:

	def __init__(self, initial_orientation, orientation_lock):
		self.orientation_lock = orientation_lock
		self.orientation = initial_orientation
		self.translate = [[0,0],[0,0],[0,0],[0,0]]
		self.translate_queue = []
		self.orientation_queue = []

	def add_to_translate(self, translate):
		self.orientation_lock.acquire()
		while len(self.translate_queue) >= 10:
			del self.translate_queue[-1]
		self.translate_queue.insert(0,translate)
		avex1 = np.average([t[0][0] for t in self.translate_queue])
		avey1 = np.average([t[0][1] for t in self.translate_queue])
		avex2 = np.average([t[1][0] for t in self.translate_queue])
		avey2 = np.average([t[1][1] for t in self.translate_queue])
		avex3 = np.average([t[2][0] for t in self.translate_queue])
		avey3 = np.average([t[2][1] for t in self.translate_queue])
		avex4 = np.average([t[3][0] for t in self.translate_queue])
		avey4 = np.average([t[3][1] for t in self.translate_queue])

		self.translate = [[avex1,avey1],[avex2,avey2],[avex3,avey3],[avex4,avey4]]
		self.orientation_lock.release()

	def add_to_orientation(self,faces):
		while len(self.orientation_queue) >= 10:
			del self.orientation_queue[-1]
		self.orientation_queue.insert(0,faces)
		avex1 = np.average([t[0][0] for t in self.orientation_queue])
		avey1 = np.average([t[0][1] for t in self.orientation_queue])
		avex2 = np.average([t[1][0] for t in self.orientation_queue])
		avey2 = np.average([t[1][1] for t in self.orientation_queue])
		avex3 = np.average([t[2][0] for t in self.orientation_queue])
		avey3 = np.average([t[2][1] for t in self.orientation_queue])
		avex4 = np.average([t[3][0] for t in self.orientation_queue])
		avey4 = np.average([t[3][1] for t in self.orientation_queue])

		self.orientation = [[avex1,avey1],[avex2,avey2],[avex3,avey3],[avex4,avey4]]

	def top(self):
		return self.orientation[0]

	def settop(self,top):
		self.orientation[0] = top

	def bottom(self):
		return self.orientation[2]

	def setbottom(self,bottom):
		self.orientation[2] = bottom

	def right(self):
		return self.orientation[1]

	def setright(self,right):
		self.orientation[1] = right

	def left(self):
		return self.orientation[3]

	def setleft(self,left):
		self.orientation[3] = left
