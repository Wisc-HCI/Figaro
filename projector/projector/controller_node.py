import sys
import time
import os
import threading

from gui import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from figaro_msgs.msg import AgentPosition
from figaro_msgs.msg import StringArray
from figaro_msgs.msg import DisplayContent
from figaro_msgs.msg import Mouse
from figaro_msgs.msg import PhysicalLayout
from figaro_msgs.msg import ItemLocations
from figaro_msgs.msg import RegionLocations
from figaro_msgs.msg import PathComponentArray
from figaro_msgs.msg import Figurines
from std_msgs.msg import Int32

class Controller(Node):

	rclpy.init()

	def __init__(self):
		super().__init__('projector_node')

		# set GUI to none initially
		self.gui = None

		# setup publishers and subscribers
		self.add_to_moment_pub = self.create_publisher(StringArray, 'tracker_pub_ctrl_add_to_moment', 10)
		self.record_curr_moment_pub = self.create_publisher(AgentPosition, 'tracker_pub_ctrl_record_moment', 10)

		self.record_curr_position_sub = self.create_subscription(AgentPosition, 'tracker_pub_proj_position', self.position_subsc_callback, 10)
		self.update_mode_stack_pub = self.create_publisher(String, 'tracker_pub_ctrl_mode_stack', 10)

		self.record_subsc = self.create_subscription(String, 'ctrl_pub_tracker_record', self.record_subsc_callback, 10)
		self.position_subsc = self.create_subscription(String,'ctrl_pub_tracker_position', self.position_subsc_callback, 10)

		self.figurine_sub = self.create_subscription(Figurines, 'ctrl_figurines_projector_pub',self.figurine_activity_callback, 10)
		self.figurine_point_pub = self.create_publisher(String, 'projector_figurine_point_pub', 10)

		# receive messages from the touch screen (simulating a mouse!)
		self.mouse_subsc = self.create_subscription(Mouse, 'tracker_pub_projector_mouse', self.touch_mouse_callback, 10)

		# calibration requests
		self.calibration_request_receiver = self.create_subscription(String, 'touch_tracker_pub_projector_calibration', self.calibration_request_callback, 10)
		self.calibration_request_ack = self.create_publisher(String, 'projector_pub_touch_tracker_calibration_ack', 10)

		# send projector pixel size and receive acknowledgement from figure tracking
		self.projector_size_pub = self.create_publisher(Point, 'projector_pub_everyone_pixelsize', 10)
		self.projector_size_track_ack = self.create_subscription(String, 'tracker_pub_projector_ack_pixelsize', self.projector_size_ack_subsc_callback, 10)
		self.projector_size_ctrl_ack = self.create_subscription(String, 'ctrl_pub_projector_ack_pixelsize', self.projector_size_ack_subsc_callback, 10)
		self.projector_size_touch_ack = self.create_subscription(String, 'touch_tracker_pub_projector_ack_pixelsize', self.projector_size_ack_subsc_callback, 10)
		self.projector_size_tablet_ack = self.create_subscription(String, 'tablet_pub_projector_ack_pixelsize', self.projector_size_ack_subsc_callback, 10)
		self.projector_size_received = {"figure_tracker":False,"touch_tracker":False,"ctrl":False, "tablet":False}   # change to true when figure_tracking acknowledges the size of the projector pixels

		self.grid_size_subscriber = self.create_subscription(Point,'ctrl_pub_projector_grid_size', self.grid_size_callback, 10)

		# receive new layouts
		self.layout_sub = self.create_subscription(PhysicalLayout, 'ctrl_layout_pub', self.layout_sub_callback, 10)
		# receive requests for and send back stored layouts
		self.layout_request = self.create_subscription(String, 'ctrl_layout_request_pub', self.layout_request_sub_callback, 10)
		self.layout_pub = self.create_publisher(PhysicalLayout, 'layout_pub_ctrl', 10)

		self.audio_display_sub = self.create_subscription(DisplayContent, 'ctrl_pub_audio_display', self.display_audio_callback, 10)
		self.highlight_agent_sub = self.create_subscription(DisplayContent, 'ctrl_pub_projector_highlight_agent', self.highlight_agent_callback, 10)

		# handling content label buttons
		self.throwaway_speech_pub = self.create_publisher(String, 'projector_pub_ctrl_throwaway_speech', 10)
		self.flag_speech_pub = self.create_publisher(String, 'projector_pub_ctrl_flag_speech', 10)

		# handle pausing
		self.request_pause_pub = self.create_publisher(String, 'projector_pub_ctrl_pause', 10)
		self.receive_pause_path = self.create_subscription(PathComponentArray, 'ctrl_pub_projector_path', self.display_path_callback, 10)
		self.request_resume_pub = self.create_publisher(Int32, 'projector_pub_ctrl_resume', 10)

		# handle figurine calibration
		self.highlight_position_sub = self.create_subscription(Point, 'tracker_pub_projector_highlight_pos', self.highlight_position_callback, 10)

		# signals
		self.record_subsc_signal = None
		self.position_subsc_signal = None

		self.add_to_mom_signal = None
		self.record_mom_signal = None
		self.update_mode_stack_signal = None
		self.figurine_point_signal = None

		self.throwaway_speech_signal = None
		self.flag_speech_signal = None

		self.vis_signal = None

		self.update_gui_position_signal = None

		self.pause_request_signal = None
		self.resume_request_signal = None

		self.external_gesture_signal = None

		# send message to ctrl node
		self.linker_pub = self.create_publisher(String, "node_link_ctrl", 10)
		self.linker_sub = self.create_subscription(String, "ctrl_link_node", self.respond_to_ping, 10)
		name = String()
		name.data = "projector"
		self.linker_pub.publish(name)

		self.gui_update_lock = threading.Lock()
		self.gui_creation_lock = threading.Lock()
		self.gui_creation_lock.acquire()

		# thread ros
		thread = threading.Thread(target=self.run_ros)
		thread.daemon = True							# Daemonize thread
		thread.start()

		# thread sending the projector size
		proj_thread = threading.Thread(target=self.send_projector_size)
		proj_thread.daemon = True

		# determine whether to visualize the mouse cursor
		test_ui = False
		if len(sys.argv) < 2:
			print("Note: cursor has been deactivated for projected interface.\nTo reactivate the cursor, add the \"test_ui\" argument.")
			print("      E.g. \"ros2 run projector controller_node test_ui\"")
		elif sys.argv[1] == "test_ui":
			test_ui = True
		else:
			self.usage()

		# package publishers to give to the GUI
		pubs = [self.figurine_point_pub]

		# setup the visualization
		self.app = QApplication(sys.argv)
		self.gui = GUI(self, test_ui, self.gui_update_lock, pubs)
		proj_thread.start()
		print("initialized gui")
		self.gui_creation_lock.release()
		sys.exit(self.app.exec_())

	def run_ros(self):
		rclpy.spin(self)

	def highlight_position_callback(self, msg):
		print("highlight position callback")
		self.gui.highlight_position(int(msg.x), int(msg.y))

	def publish_figurine_point(self, region):
		msg = String()
		msg.data = region
		self.figurine_point_pub.publish(msg)

	def publish_add_to_moment(self, data):
		try:
			print("publish add_to_moment")
			msg = StringArray()
			msg.array = data
			self.add_to_moment_pub.publish(msg)
		except:
			print("gui not initialized yet")

	def publish_record_curr_moment(self, name, x, y, theta):
		print("publish_record_curr_moment")
		msg = AgentPosition()
		msg.identifier = name
		position = Point()
		position.x = float(x)
		position.y = float(y)
		position.z = float(theta)
		msg.position = position
		self.record_curr_moment_pub.publish(msg)

	def publish_record_curr_position(self, x, y):
		print("publish_record_curr_position")
		msg = Point()
		msg.x = float(x)
		msg.y = float(y)
		self.record_curr_position_pub.publish(msg)

	def publish_pause_request(self, request):
		print("pause_request")
		if request:
			msg = String()
			self.request_pause_pub.publish(msg)
		else:
			msg = Point()
			self.request_resume_pub.publish(msg)

	def publish_resume_request(self,rewind):
		print("resume_request")
		msg = Int32()
		msg.data = rewind
		self.request_resume_pub.publish(msg)

	def publish_update_mode_stack_pub(self, data):
		print("publish_update_mode_stack_pub")
		msg = String()
		msg.data = data
		self.update_mode_stack_pub.publish(msg)

	def publish_throwaway_speech(self, throwaway_speech):
		print("publish_throwaway_speech")
		msg = String()
		msg.data = throwaway_speech
		self.throwaway_speech_pub.publish(msg)

	def publish_flag_speech(self, flag_speech):
		print("publish_flag_speech")
		msg = String()
		msg.data = flag_speech
		self.flag_speech_pub.publish(msg)

	def record_subsc_callback(self, msg):
		print("record_subsc_callback")
		string = msg.data
		self.record_subsc_signal.emit(string)

	def position_subsc_callback(self, msg):
		try:
			#print("position_subsc_callback")
			self.update_gui_position_signal.emit(msg)
		except:
			print("gui not initialized yet")

	def touch_mouse_callback(self, msg):
		print("touch_mouse_callback")

		# get the event type
		if msg.event_type == "click":
			x = msg.x
			y = msg.y
			click_time = msg.time
			self.gui.click(x,y,click_time)
		elif msg.event_type == "release":
			click_time = msg.time
			x = msg.x
			y = msg.y
			self.gui.release(x,y,click_time)

	def projector_size_ack_subsc_callback(self, msg):
		print("about to ack subsc")
		'''
		If acknowledged, halt the thread that sends the projector size
		'''
		string = msg.data
		self.projector_size_received[string] = True
		print("...projector size acknowledged by {}".format(string))

	def grid_size_callback(self,msg):
		print("grid_size_callback")
		self.gui_creation_lock.acquire()
		x = int(msg.x)
		y = int(msg.y)

		self.gui.set_grid_size(x,y)
		self.gui_creation_lock.release()

	def display_audio_callback(self, msg):
		print("display_audio_callback")
		agent = msg.agent
		speech_or_action = msg.type
		content = msg.content
		content_cat = msg.content_category
		content_id = msg.content_id
		unfinished = msg.unfinished

		self.gui.display_content(agent,speech_or_action,content,content_cat,content_id,unfinished)

	def highlight_agent_callback(self, msg):
		print("highlight_agent_callback")
		agent = msg.agent
		typ = msg.type
		content_id = msg.content_id
		self.gui.highlight_agent(agent,typ,content_id)

	'''
	Threaded method --
	 - every N seconds, attempt to send a message to figure_tracking with the size of the projector
	'''
	def send_projector_size(self):
		print("send_projector_size")
		projector_size_received = True
		for ack in list(self.projector_size_received.values()):
			projector_size_received = projector_size_received and ack

		while not projector_size_received:

			# send every N=5 seconds
			time.sleep(5)

			# get, package, and send the projector size
			x,y = self.gui.get_projector_size()
			msg = Point()
			msg.x = float(x)
			msg.y = float(y)
			self.projector_size_pub.publish(msg)
			print("sent projector size to figure tracker, touch tracker, and ctrl...")

			# add extra time for an acknowledgement to be received
			time.sleep(1)

			# recalculate the break condition
			projector_size_received = True
			for ack in list(self.projector_size_received.values()):
				projector_size_received = projector_size_received and ack

	def respond_to_ping(self,message):
		print("respond to ping")
		name = String()
		name.data = "projector"
		self.linker_pub.publish(name)

	def calibration_request_callback(self,msg):
		print("calibration_request_callback")
		string = String()
		string.data = "ack"
		self.calibration_request_ack.publish(string)
		self.gui.begin_calibration()

	def display_path_callback(self,msg):
		print("display_path_callback")
		self.gui.display_path_components(msg)

	def layout_sub_callback(self,msg):
		print("layout_sub_callback")
		self.gui.update_layout(msg)

	def layout_request_sub_callback(self,msg):
		print("layout_request_sub_callback")
		'''
		Obtain the grid and specific layouts from the physical layout class, send it back to ctrl
		'''
		physical_layout = self.gui.physical_layout

		# setup response message
		layout_msg = PhysicalLayout()
		layout_msg.name = msg.data
		layout_msg.x = physical_layout.grid[0]
		layout_msg.y = physical_layout.grid[1]

		# set the layout data
		for item in physical_layout.layout:
			itemloc_msg = ItemLocations()
			itemloc_msg.name = item
			for coord in physical_layout.layout[item]:
				point = Point()
				point.x = float(coord[0])
				point.y = float(coord[1])
				itemloc_msg.data.append(point)
			layout_msg.object_data.append(itemloc_msg)

		# set the regions
		for region in physical_layout.regions:
			regionloc_msg = RegionLocations()
			regionloc_msg.name = region
			for coord in physical_layout.regions[region]:
				point = Point()
				point.x = float(coord[0])
				point.y = float(coord[1])
				regionloc_msg.data.append(point)
			rgb = physical_layout.region_colors[region]
			regionloc_msg.r = rgb[0]
			regionloc_msg.g = rgb[1]
			regionloc_msg.b = rgb[2]
			regionloc_msg.a = 1.0
			layout_msg.region_data.append(regionloc_msg)

		layout_msg.detailed_json = physical_layout.layout_image_details

		self.layout_pub.publish(layout_msg)

	def figurine_activity_callback(self,msg):
		print("figurine_activity_callback")
		if self.gui is None:
			return
		if self.external_gesture_signal is None:
			return

		if msg.wait:
			self.external_gesture_signal.emit("wait","beginning",-1,"")
		else:
			self.external_gesture_signal.emit("wait","end",-1,"")

		if msg.point:
			self.external_gesture_signal.emit("point","beginning",msg.angle,"")
		else:
			self.external_gesture_signal.emit("point","end",-1,"")

		if msg.tilt:
			self.external_gesture_signal.emit("tilt","beginning",-1,msg.direction)
		else:
			self.external_gesture_signal.emit("tilt","end",-1,"")

		'''
		if msg.wait:
			self.gui.begin_wait()
		else:
			self.gui.end_wait()
		if msg.point:
			print("msg: {}".format(msg.angle))
			self.gui.begin_point(msg.angle)
		else:
			self.gui.end_point()
		if msg.tilt:
			self.gui.begin_nod(msg.direction)
		else:
			self.gui.end_nod()
		'''

	def usage(self):
		print("Usage: ros2 run projector controller_node [optarg: test_ui]")

def exception_hook(exctype, value, traceback):
	print("exception hook")
	print(exctype, value, traceback) # print exception. 
	sys._excepthook(exctype, value, traceback) # call original excepthoot. I do not why 
	sys.exit(1) # terminate program if above do not do this

sys.excepthook = exception_hook # overwrite default excepthook 

def main():
	controller = Controller()

if __name__ == '__main__':
	main()
