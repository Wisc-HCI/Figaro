import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import math
import time
import threading
from pylibfreenect2 import Freenect2, SyncMultiFrameListener
from pylibfreenect2 import FrameType, Registration, Frame

def main():

	try:
		from pylibfreenect2 import OpenGLPacketPipeline
		pipeline = OpenGLPacketPipeline()
	except:
		try:
			from pylibfreenect2 import OpenCLPacketPipeline
			pipeline = OpenCLPacketPipeline()
		except:
			from pylibfreenect2 import CpuPacketPipeline
			pipeline = CpuPacketPipeline()
	print("Packet pipeline:", type(pipeline).__name__)

	fn = Freenect2()
	num_devices = fn.enumerateDevices()
	if num_devices == 0:
		print("No device connected!")
		sys.exit(1)

	serial = fn.getDeviceSerialNumber(0)
	device = fn.openDevice(serial, pipeline=pipeline)

	types = 0
	if True:
		types |= FrameType.Color
	if True:
		types |= (FrameType.Ir | FrameType.Depth)
	listener = SyncMultiFrameListener(types)

	# Register listeners
	device.setColorFrameListener(listener)
	device.setIrAndDepthFrameListener(listener)

	device.start()
	device.startStreams(rgb=True,depth=True)

	registration = Registration(device.getIrCameraParams(),
								device.getColorCameraParams())

	undistorted = Frame(512, 424, 4)
	registered = Frame(512,424,4)

	#cap = cv2.VideoCapture(1)
	#cap.set(3,1920)
	#cap.set(4,1080)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
	parameters =  aruco.DetectorParameters_create()
	#parameters.maxErroneousBitsInBorderRate=2
	#parameters.adaptiveThreshWinSizeMin=25
	#parameters.adaptiveThreshWinSizeMax=45
	#parameters.adaptiveThreshWinSizeStep=4
	#parameters.cornerRefinementMinAccuracy=0.01
	#parameters.markerBorderBits=4
	#print(parameters.cornerRefinementMinAccuracy)

	while True:
		#ret, frame = cap.read()
		frames = listener.waitForNewFrame()
		frame_raw = frames["color"]
		grayir_raw = frames["ir"]

		registration.apply(frame_raw, grayir_raw, undistorted, registered)

		frame = frame_raw.asarray()
		grayir = grayir_raw.asarray()

		mi, ma = grayir.min(), grayir.max()
		grayir = np.uint8(255 * (grayir - mi) / (ma - mi))
		#grayir = np.uint8(grayir)


		# Our operations on the frame come here
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		print(gray[0][0])
		corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

		# IR
		#grayir_gray = cv2.cvtColor(grayir, cv2.COLOR_BGR2GRAY)
		grayir_gray = grayir
		corners, ids, rejectedImgPoints = aruco.detectMarkers(grayir_gray, aruco_dict, parameters=parameters)
		frame_markers_ir = aruco.drawDetectedMarkers(grayir_gray.copy(), corners, ids)

		print("~~~~frame marker")
		print(grayir_raw.asarray()[0][100] / 65535.)
		print(grayir[0][100])

		# Display the resulting frame
		cv2.imshow('frame',frame_markers)
		cv2.imshow('ir',frame_markers_ir)
		listener.release(frames)
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

if __name__=="__main__":
	main()