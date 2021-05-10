import sys
import time
import os
import threading

from aruco import *
from physical_layout import *

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from figaro_msgs.msg import StringArray
from figaro_msgs.msg import AgentPosition
from figaro_msgs.msg import Mouse
from figaro_msgs.msg import PointArray

class Controller(Node):

	rclpy.init()

	def __init__(self):
		super().__init__('tracking_node')

		# setup publishers and subscribers
		self.add_to_moment_pub = self.create_publisher(StringArray, 'tracker_pub_ctrl_add_to_moment', 10)
		self.record_curr_moment_pub = self.create_publisher(AgentPosition, 'tracker_pub_ctrl_record_moment', 10)
		#self.record_curr_moment_sub = self.create_subscription(Point, 'tracker_pub_ctrl_record_moment', self.position_subsc_callback)
		self.record_curr_position_pub = self.create_publisher(AgentPosition, 'tracker_pub_proj_position', 10)
		self.update_mode_stack_pub = self.create_publisher(String, 'tracker_pub_ctrl_mode_stack', 10)

		self.record_subsc = self.create_subscription(String, 'ctrl_pub_tracker_record', self.record_subsc_callback, 10)

		# for receiving and acknowledging projector pixel size
		self.pixel_size_receiver = self.create_subscription(Point, 'projector_pub_everyone_pixelsize', self.pixel_size_subsc_callback, 10)
		self.pixel_size_ack = self.create_publisher(String, 'tracker_pub_projector_ack_pixelsize', 10)

		# for calibration
		self.pass_calibration = self.create_subscription(PointArray, 'touch_tracker_pub_figure_tracker_pass_calibration', self.receive_calibration_callback, 10)
		self.highlight_pos_pub = self.create_publisher(Point, 'tracker_pub_projector_highlight_pos', 10)

		# setup the physical layout
		self.physical_layout = PhysicalLayout()

		# send message to ctrl node
		self.linker_pub = self.create_publisher(String, "node_link_ctrl", 10)
		name = String()
		name.data = "figure tracking"
		self.linker_pub.publish(name)

		# setup the detection (threadded)
		self.detection = ArucoDetection(self,self.physical_layout)
		# for receiving images from the kinect
		self.kinect_subscriber = self.create_subscription(Image, '/ir/image_raw', self.detection.image_callback, 10)
	

		# thread ros
		thread = threading.Thread(target=self.run_ros)
		thread.daemon = True							# Daemonize thread
		thread.start()
			
		# enables node to run when kinect and/or opencv libraries are not installed
		self.detection.detect()

	def run_ros(self):
		rclpy.spin(self)

	def publish_highlight_position(self, position_tup):
		pt = Point()
		pt.x = float(position_tup[0])
		pt.y = float(position_tup[1])
		self.highlight_pos_pub.publish(pt)

	def publish_add_to_moment(self, data):
		msg = StringArray()
		for item in data:
			msg.array.append(item)
		self.add_to_moment_pub.publish(msg)

	def publish_record_curr_moment(self, ident, x, y):
		'''
		Ident is the identity of the agent (determined by the aruco marker) in the position x,y
		'''
		msg = AgentPosition()
		msg.identifier = ident
		msg.x = float(x)
		msg.y = float(y)
		self.record_curr_moment_pub.publish(msg)

	def publish_record_curr_position(self, ident, x, y, corners):
		'''
		Ident is the identity of the agent (determined by the aruco marker) in the position x,y
		'''
		msg = AgentPosition()
		msg.identifier = ident
		position = Point()
		position.x = float(x)
		position.y = float(y)
		msg.position = position

		if corners is not None:
			top_left = Point()
			top_left.x = float(corners[0][0])
			top_left.y = float(corners[0][1])
			msg.top_left = top_left

			top_right = Point()
			top_right.x = float(corners[1][0])
			top_right.y = float(corners[1][1])
			msg.top_right = top_right

			bottom_right = Point()
			bottom_right.x = float(corners[2][0])
			bottom_right.y = float(corners[2][1])
			msg.bottom_right = bottom_right

			bottom_left = Point()
			bottom_left.x = float(corners[3][0])
			bottom_left.y = float(corners[3][1])
			msg.bottom_left = bottom_left
		else:
			dummy_point = Point()
			dummy_point.x = -1.0
			dummy_point.y = -1.0
			msg.top_left = msg.top_right = msg.bottom_right = msg.bottom_left = dummy_point

		self.record_curr_position_pub.publish(msg)

	def publish_update_mode_stack_pub(self, data):
		msg = String()
		msg.data = data
		self.update_mode_stack_pub.publish(msg)

	def record_subsc_callback(self, msg):
		string = msg.data
		if os.name != "nt":
			self.detection.update_record(string)

	def pixel_size_subsc_callback(self, msg):
		x = int(round(msg.x))
		y = int(round(msg.y))
		
		# send the size to the aruco marker detection
		self.detection.set_projector_pixel_size(x,y)
		
		# acknowledge the size of the projector
		ack = String()
		ack.data = "figure_tracker"
		self.pixel_size_ack.publish(ack)

	def receive_calibration_callback(self,msg):
		print("GUI --> received calibration")
		point_array = msg.array
		tl = (point_array[0].x,point_array[0].y)
		tr = (point_array[1].x,point_array[1].y)
		br = (point_array[2].x,point_array[2].y)
		bl = (point_array[3].x,point_array[3].y)

		self.detection.calibrate_position(tl,tr,br,bl)

def main():
	controller = Controller()

if __name__ == '__main__':
	main()
