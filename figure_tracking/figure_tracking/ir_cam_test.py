import numpy as np
import pandas as pd
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import time
import sys
import os

class ArucoDetection:

	HUMAN_IDS = [4]
	ROBOT_ID = 5
	ALL_IDS = [4,5]

	def __init__(self):
		self.recording = False
		#self.cap = cv2.VideoCapture(0)

		# lo-res webcam
		self.width = 512
		self.height = 424
		# TODO: create calibration interface later
		self.left = 35
		self.right = 630
		self.top = 73
		self.bottom = 432

		# initialize projector size to be default values
		self.projector_x = 1910
		self.projector_y = 1035

		# make the transformation matrix
		self.srcQuad = np.zeros((4,2))
		self.srcQuad[0][0] = 382  # top left
		self.srcQuad[0][1] = 64
		self.srcQuad[1][0] = 152  # top right
		self.srcQuad[1][1] = 64
		self.srcQuad[2][0] = 170  # bottom right
		self.srcQuad[2][1] = 217
		self.srcQuad[3][0] = 360  # bottom left
		self.srcQuad[3][1] = 219
		self.distQuad = np.zeros((4,2))
		self.set_dist_quad()

		# mouse click event
		cv2.namedWindow("ir")
		cv2.setMouseCallback("ir",self.mouse_click)

	def set_projector_pixel_size(self,x,y):
		'''
		Set the pixel size of the projector
		'''
		self.projector_x = x
		self.projector_y = y

	def set_dist_quad(self):

		# set the values
		self.distQuad[0][0] = 0  # top left
		self.distQuad[0][1] = 0
		self.distQuad[1][0] = self.projector_x  # top right
		self.distQuad[1][1] = 0
		self.distQuad[2][0] = self.projector_x  # bottom right
		self.distQuad[2][1] = self.projector_y
		self.distQuad[3][0] = 0  # bottom left
		self.distQuad[3][1] = self.projector_y

		# recalculate the transformation matrix
		self.srcQuad = self.srcQuad.astype(np.float32)
		self.distQuad = self.distQuad.astype(np.float32)
		self.tmat = cv2.getPerspectiveTransform(self.srcQuad,self.distQuad)		

	def calibrate_val(self):
		'''
		collect average value of each pixel, store it
		'''

	def calibrate_position(self):
		'''
		calibrate the location of each of the four courners of the tabletop interface
		'''
		# reset everything
		print("resetting corners of projection")
		self.srcQuad[0][0] = -1  # top left
		self.srcQuad[0][1] = -1
		self.srcQuad[1][0] = -1  # top right
		self.srcQuad[1][1] = -1
		self.srcQuad[2][0] = -1  # bottom right
		self.srcQuad[2][1] = -1
		self.srcQuad[3][0] = -1  # bottom left
		self.srcQuad[3][1] = -1

	def mouse_click(self,event, x, y, flags, params):
		'''
		figure out which point to fill
		'''
		if event == cv2.EVENT_LBUTTONDOWN:
			if self.srcQuad[0][0] == -1:
				self.srcQuad[0][0] = x
				self.srcQuad[0][1] = y
				print("filled top left with {}, {}".format(x,y))
			elif self.srcQuad[1][0] == -1:
				self.srcQuad[1][0] = x
				self.srcQuad[1][1] = y
				print("filled top right with {}, {}".format(x,y))
			elif self.srcQuad[2][0] == -1:
				self.srcQuad[2][0] = x
				self.srcQuad[2][1] = y
				print("filled bottom right with {}, {}".format(x,y))
			elif self.srcQuad[3][0] == -1:
				self.srcQuad[3][0] = x
				self.srcQuad[3][1] = y
				print("filled bottom left with {}, {}".format(x,y))
				print("srcQuad reset to \n{}".format(self.srcQuad))

	def update_record(self, msg):
		string = msg
		if string == "enable":
			self.recording = True
		elif string == "disable":
			self.recording = False

	def detect(self):
		'''
		PURPOSE: detect markers in the background
		'''

		cap = cv2.VideoCapture(1)
		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		parameters =  aruco.DetectorParameters_create()

		while(True):

			# Capture frame-by-frame
			ret, frame = cap.read()


			# Our operations on the frame come here
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			frame_markers = aruco.drawDetectedMarkers(gray.copy(), corners, ids)

			cv2.imshow('frame',frame_markers)
			cv2.imshow('vis',frame)

			# Display the resulting frame
			#cv2.imshow('frame',frame_markers)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

if __name__=="__main__":
	ad = ArucoDetection()
	ad.detect()