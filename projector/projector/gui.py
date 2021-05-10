from PyQt5.QtCore import Qt, pyqtSignal, QUrl
from PyQt5.QtWidgets import QApplication, QMainWindow, QDesktopWidget
from PyQt5.QtWebEngineWidgets import QWebEngineView

from table_pane import *

from std_msgs.msg import String
from figaro_msgs.msg import AgentPosition

import os
import sys
import signal

# import shared modules
cwd = os.getcwd()
idx = cwd.index("/projector/projector")
if idx + 20 == len(cwd):
	path =  cwd[0:idx]
	sys.path.append("{}/share".format(path))
from physical_layout import *

class GUI(QMainWindow):

	'''
	SIGNALS FROM TABLE TO ROS
	'''
	# subscriber signals
	record_subsc_signal = pyqtSignal(str)
	position_subsc_signal = pyqtSignal(str)

	# publisher signals
	add_to_mom_signal = pyqtSignal(list)
	record_mom_signal = pyqtSignal(str,int,int,int)
	update_mode_stack_signal = pyqtSignal(str)
	figurine_point_signal = pyqtSignal(str)

	# gui position updater
	update_gui_position_signal = pyqtSignal(AgentPosition)

	# content label buttons
	throwaway_speech_signal = pyqtSignal(str)
	flag_speech_signal = pyqtSignal(str)

	# pause 
	pause_request_signal = pyqtSignal(bool)
	resume_request_signal = pyqtSignal(int)

	# window resizing
	resized = pyqtSignal()

	'''
	SIGNALS FROM ROS TO TABLE
	'''
	external_gesture_signal = pyqtSignal(str,str,int,str)

	def __init__(self, controller, test_ui, update_lock, publishers):
		super().__init__()
		self.title = 'Figaro'
		self.update_lock = update_lock
		self.setWindowTitle(self.title)
		self.figurine_point_pub = publishers[0]
		screen_count = QDesktopWidget().screenCount()
		print(screen_count)
		if screen_count > 1:
			display_monitor = 1 # the number of the monitor you want to display your widget

			monitor = QDesktopWidget().screenGeometry(display_monitor)
			#widget.showFullScreen()
			self.width,self.height = monitor.width(),monitor.height()
			self.setGeometry(monitor.left(),monitor.top(),self.width,self.height)
			print("using display {}".format(display_monitor))
			offset_x = monitor.left()
			offset_y = monitor.top()# + 10
			
		else:
			display_monitor = 0 # the number of the monitor you want to display your widget

			monitor = QDesktopWidget().screenGeometry(display_monitor)
			self.move(monitor.left(), monitor.top())
			desktop = QApplication.desktop()
			screen_resolution = desktop.screenGeometry()
			#widget.showFullScreen()
			print("using display {}".format(display_monitor))
			self.width, self.height = screen_resolution.width(), screen_resolution.height()
			self.setGeometry(0,0,self.width,self.height)
			offset_x = 0
			offset_y = 0

		# link the controller
		self.controller = controller

		# TODO: replace with more permanent solution
		# initialize the physical layout
		self.physical_layout = PhysicalLayout()

		# setup any publishers
		self.add_to_mom_signal.connect(self.controller.publish_add_to_moment)
		self.record_mom_signal.connect(self.controller.publish_record_curr_moment)
		self.figurine_point_signal.connect(self.controller.publish_figurine_point)
		self.update_mode_stack_signal.connect(self.controller.publish_update_mode_stack_pub)
		self.throwaway_speech_signal.connect(self.controller.publish_throwaway_speech)
		self.flag_speech_signal.connect(self.controller.publish_flag_speech)
		self.pause_request_signal.connect(self.controller.publish_pause_request)
		self.resume_request_signal.connect(self.controller.publish_resume_request)
		self.controller.figurine_point_signal = self.figurine_point_signal
		self.controller.add_to_mom_signal = self.add_to_mom_signal
		self.controller.record_mom_signal = self.record_mom_signal
		self.controller.update_mode_stack_signal = self.update_mode_stack_signal
		self.controller.throwaway_speech_signal = self.throwaway_speech_signal
		self.controller.flag_speech_signal = self.flag_speech_signal
		self.controller.pause_request_signal = self.pause_request_signal
		self.controller.resume_request_signal = self.resume_request_signal

		# set up external signals
		self.external_gesture_signal.connect(self.begin_gesture)
		self.controller.external_gesture_signal = self.external_gesture_signal

		# setup any subscribers
		self.record_subsc_signal.connect(self.update_record)
		self.position_subsc_signal.connect(self.update_position)
		self.controller.record_subsc_signal = self.record_subsc_signal
		self.controller.position_subsc_signal = self.position_subsc_signal

		# setup gui position signal
		self.update_gui_position_signal.connect(self.update_position)
		self.controller.update_gui_position_signal = self.update_gui_position_signal

		# set up web view
		# immediately set the background to the webview
		self.webView = QWebEngineView(self)
		self.webView.setGeometry(0, 0, self.width, self.height)
		url = QUrl.fromLocalFile("{}/display.html".format(cwd))
		self.webView.load(url)

		# set up table pane
		self.table_pane = TablePane(self,self.physical_layout, test_ui, parent=self, offset_x=offset_x, offset_y=offset_y)
		#self.table_pane.setStyleSheet("QLabel { background-color: white; }")

		signal.signal(signal.SIGINT, signal.SIG_DFL)
		if display_monitor == 1:
			self.showFullScreen()
		else:
			self.show()

	def highlight_position(self, x, y):
		#self.update_lock.acquire()
		print("highlight position")
		if x == y == -1:
			self.table_pane.unhighlight_position()
		else:
			self.table_pane.highlight_position(x,y)
		#self.update_lock.release()

	def update_record(self, msg):
		#self.update_lock.acquire()
		print("update record")
		string = msg
		if string == "enable":
			self.table_pane.enable_recording()
		elif string == "disable":
			self.table_pane.disable_recording()
		#self.update_lock.release()

	def update_position(self, position_msg):
		ident = position_msg.identifier
		pos_x = position_msg.position.x
		pos_y = position_msg.position.y

		corners = None
		if position_msg.top_left.x != -1:
			corners = []
			corners.append([position_msg.top_left.x,position_msg.top_left.y])
			corners.append([position_msg.top_right.x,position_msg.top_right.y])
			corners.append([position_msg.bottom_right.x,position_msg.bottom_right.y])
			corners.append([position_msg.bottom_left.x,position_msg.bottom_left.y])
		#self.update_lock.acquire()
		self.table_pane.update_position(ident,pos_x,pos_y,corners)
		#self.update_lock.release()

	def get_projector_size(self):
		print("get_projector_size")
		'''
		Return the size of the table_pane in size=x,y
		'''
		#self.update_lock.acquire()
		x = self.table_pane.width()
		y = self.table_pane.height()
		#self.update_lock.release()
		print("projector size: {} - {}".format(x,y))
		return x,y

	def set_grid_size(self,x,y):
		#self.update_lock.acquire()
		print("get_grid_size")
		self.table_pane.set_grid_size(x,y)
		#self.update_lock.release()

	def display_content(self,agent,speech_or_action,content,content_category,content_id,unfinished):
		print("display_content")
		'''
		Display the content of the human/robot's speech/action
		'''
		#self.update_lock.acquire()
		self.table_pane.display_content(agent,speech_or_action,content,content_category,content_id,unfinished)
		#self.update_lock.release()

	def highlight_agent(self,agent,typ,content_id):
		print("highlight_agent")
		'''
		Display that the relevant agent hears/does not hear the wakeword
		'''
		#self.update_lock.acquire()
		self.table_pane.highlight_agent(agent,typ,content_id)
		#self.update_lock.release()

	def request_pause(self, pause_status, rewind=None):
		print("request pause")
		if pause_status:
			self.pause_request_signal.emit(pause_status)
		else:
			self.resume_request_signal.emit(rewind)

	def display_path_components(self, msg):
		#self.update_lock.acquire()
		print("display_path_components")
		self.table_pane.extract_paths(msg.array)
		#self.update_lock.release()

	def click(self, x, y, click_time):
		#self.update_lock.acquire()
		print("click")
		self.table_pane.click(x,y,click_time)
		#self.update_lock.release()

	def release(self,x,y,click_time):
		#self.update_lock.acquire()
		print("release")
		self.table_pane.release_click(x,y,click_time)
		#self.update_lock.release()

	def begin_calibration(self):
		#self.update_lock.acquire()
		print("begin calibration")
		self.table_pane.begin_calibration()
		#self.update_lock.release()

	def update_layout(self, layout_msg):
		print("UPDATING LAYOUT")
		print(layout_msg.object_data)
		print(layout_msg.region_data)

		# block updates to physical layout
		self.table_pane.layout_lock.acquire()

		# reinitialize the physical layout
		size_x = int(layout_msg.x)
		size_y = int(layout_msg.y)
		self.physical_layout.initialize(size_x,size_y)

		# populate the cells
		for item_loc in layout_msg.object_data:
			name = item_loc.name
			points = item_loc.data
			for point in points:
				x = int(point.x)
				y = int(point.y)
				self.physical_layout.populate_cell(x,y,name)

		# temporary code for handling regions
		for region_loc in layout_msg.region_data:
			name = region_loc.name
			points = region_loc.data
			for point in points:
				x = int(point.x)
				y = int(point.y)
				self.physical_layout.populate_region_cell(x,y,name)
			r = region_loc.r
			g = region_loc.g
			b = region_loc.b
			self.physical_layout.assign_region_color(name,r,g,b)
		self.physical_layout.calculate_centers()

		print(self.physical_layout.regions)

		# store the json with the specific layout details
		self.physical_layout.layout_image_details = layout_msg.detailed_json

		# update the region label placements in the gui
		self.table_pane.update_region_label_placements()

		# block updates to physical layout
		self.table_pane.layout_lock.release()

	def begin_gesture(self, typ, beg_end, angle, direction):
		if typ == "tilt":
			if beg_end == "beginning":
				self.begin_nod(direction)
			else:
				self.end_nod()
		elif typ == "point":
			if beg_end == "beginning":
				self.begin_point(angle)
			else:
				self.end_point()
		elif typ == "wait":
			if beg_end == "beginning":
				self.begin_wait()
			else:
				self.end_wait()

	def begin_nod(self, direction):
		#self.update_lock.acquire()
		self.table_pane.begin_nod(direction)
		#self.update_lock.release()

	def end_nod(self):
		#self.update_lock.acquire()
		self.table_pane.end_nod()
		#self.update_lock.release()

	def begin_point(self, angle):
		#self.update_lock.acquire()
		self.table_pane.begin_point(angle)
		#self.update_lock.release()

	def end_point(self):
		#self.update_lock.acquire()
		self.table_pane.end_point()
		#self.update_lock.release()

	def begin_wait(self):
		#self.update_lock.acquire()
		self.table_pane.begin_wait()
		#self.update_lock.release()

	def end_wait(self):
		#self.update_lock.acquire()
		self.table_pane.end_wait()
		#self.update_lock.release()
