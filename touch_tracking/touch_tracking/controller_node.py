import sys
import time
import os
import threading

from touch import *

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

		# for receiving and acknowledging projector pixel size
		self.pixel_size_receiver = self.create_subscription(Point, 'projector_pub_everyone_pixelsize', self.pixel_size_subsc_callback, 10)
		self.pixel_size_ack = self.create_publisher(String, 'touch_tracker_pub_projector_ack_pixelsize', 10)

		# handle touch input
		self.touch_input_pub = self.create_publisher(Mouse, 'tracker_pub_projector_mouse', 10)

		# handle calibration
		self.calibration_request = self.create_publisher(String, 'touch_tracker_pub_projector_calibration',10)
		self.calibration_ack = self.create_subscription(String, 'projector_pub_touch_tracker_calibration_ack', self.calibration_request_received_callback, 10)
		self.calibration_request_acked = False
		self.pass_calibration = self.create_publisher(PointArray, 'touch_tracker_pub_figure_tracker_pass_calibration', 10)

		# send message to ctrl node
		self.linker_pub = self.create_publisher(String, "node_link_ctrl", 10)
		name = String()
		name.data = "touch tracking"
		self.linker_pub.publish(name)

		# setup the detection (threadded)
		self.detection = TouchDetection(self,self.touch_input_pub)
		# for receiving images from the kinect
		self.kinect_subscriber = self.create_subscription(Image, '/ir/image_raw', self.detection.image_callback, 10)

		# thread ros
		thread = threading.Thread(target=self.run_ros)
		thread.daemon = True							# Daemonize thread
		thread.start()

		# thread calibration request
		if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
			thread = threading.Thread(target=self.send_calibration_request)
			thread.daemon = True							# Daemonize thread
			thread.start()
			
		# enables node to run when kinect and/or opencv libraries are not installed
		self.detection.visualize()

	def pixel_size_subsc_callback(self, msg):
		x = int(round(msg.x))
		y = int(round(msg.y))
		
		# send the size to the aruco marker detection
		self.detection.set_projector_pixel_size(x,y)
		
		# acknowledge the size of the projector
		ack = String()
		ack.data = "touch_tracker"
		self.pixel_size_ack.publish(ack)

	def run_ros(self):
		rclpy.spin(self)

	'''
	When we start, send a signal to the projector that we need to calibrate
	'''
	def send_calibration_request(self):
		while not self.calibration_request_acked:
			string = String()
			string.data = "calibrate"
			self.calibration_request.publish(string)
			print("sent calibration request")
			time.sleep(5)

	def calibration_request_received_callback(self,msg):
		print("received possible ack for calibration request")
		if msg.data == "ack":
			self.calibration_request_acked = True

			# begin the calibration routine in the touch detection class
			self.detection.begin_calibration()

	def pass_calibration_to_figure_tracker(self,tl,tr,br,bl):
		pa = PointArray()
		point_tl = Point()
		point_tl.x = tl[0]
		point_tl.y = tl[1]
		pa.array.append(point_tl)

		point_tr = Point()
		point_tr.x = tr[0]
		point_tr.y = tr[1]
		pa.array.append(point_tr)

		point_br = Point()
		point_br.x = br[0]
		point_br.y = br[1]
		pa.array.append(point_br)

		point_bl = Point()
		point_bl.x = bl[0]
		point_bl.y = bl[1]
		pa.array.append(point_bl)

		self.pass_calibration.publish(pa)

def main():
	controller = Controller()

if __name__ == '__main__':
	main()
