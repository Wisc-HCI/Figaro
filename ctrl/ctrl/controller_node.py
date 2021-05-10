import rclpy
import signal
from controller.controller import *

def main():
	rclpy.init()
	controller = Controller()
	signal.signal(signal.SIGINT, controller.ble.signal_handler)
	rclpy.spin(controller)

if __name__=="__main__":
	main()
