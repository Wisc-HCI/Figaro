import numpy as np
import scipy as sp
import scipy.ndimage
import pandas as pd
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import math
import time
import sys

# import shared modules
import os
import sys
import json

from cv_bridge import CvBridge, CvBridgeError

from figaro_msgs.msg import Mouse

class TouchDetection:

	def __init__(self,controller, touch_input_pub):
		self.controller = controller
		self.recording = False
		#self.cap = cv2.VideoCapture(0)

		# initialize projector size to be default values
		self.projector_x = 1910
		self.projector_y = 1035

		# make the transformation matrix
		self.srcQuad = np.zeros((4,2))
		self.srcQuad[0][0] = 640  # top left
		self.srcQuad[0][1] = 0
		self.srcQuad[1][0] = 0  # top right
		self.srcQuad[1][1] = 0
		self.srcQuad[2][0] = 0  # bottom right
		self.srcQuad[2][1] = 480
		self.srcQuad[3][0] = 640  # bottom left
		self.srcQuad[3][1] = 480
		self.aoi_src_quad = np.zeros((4,2))
		self.aoi_src_quad[0][0] = 0  # top left
		self.aoi_src_quad[0][1] = 0
		self.aoi_src_quad[1][0] = 640  # top right
		self.aoi_src_quad[1][1] = 0
		self.aoi_src_quad[2][0] = 0    # bottom left
		self.aoi_src_quad[2][1] = 480
		self.aoi_src_quad[3][0] = 640  # bottom right
		self.aoi_src_quad[3][1] = 480
		if not( len(sys.argv) > 1 and sys.argv[1] == "calibrate"):
			try:
				calfile = open("tracker_calibration.json","r")
				caldata = json.load(calfile)
				tl = caldata["tl"]
				tr = caldata["tr"]
				bl = caldata["bl"]
				br = caldata["br"]
				self.aoi_src_quad[0][0] = tr[0]  # top left
				self.aoi_src_quad[0][1] = tr[1]
				self.aoi_src_quad[1][0] = tl[0]  # top right
				self.aoi_src_quad[1][1] = tl[1]
				self.aoi_src_quad[2][0] = br[0]    # bottom left
				self.aoi_src_quad[2][1] = br[1]
				self.aoi_src_quad[3][0] = bl[0]  # bottom right
				self.aoi_src_quad[3][1] = bl[1]
				print("successfully found calibration file")

				# send a message to the figure tracker to reset its position
				self.controller.pass_calibration_to_figure_tracker(tl,tr,br,bl)
			except:
				print("could not find calibration file -- please manually calibrate")
		self.distQuad = np.zeros((4,2))
		self.aoi_quad = np.zeros((4,2))
		self.set_dist_quad()

		# detected image components
		self.img = None
		self.ids = None
		self.cornder = None

		# initialize aruco detection parameters
		self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
		self.parameters =  aruco.DetectorParameters_create()
		self.parameters.polygonalApproxAccuracyRate = 0.1
		self.parameters.perspectiveRemoveIgnoredMarginPerCell = 0.2
		self.parameters.maxErroneousBitsInBorderRate = 0.45
		self.parameters.errorCorrectionRate = 0.8
		self.bridge = CvBridge()

		# the background capture
		self.background_captures = []
		self.background_capture = None
		self.eq_background_capture = None
		self.smoothed_background = None
		self.subtracted_capture = None
		self.blob_capture = None
		self.smoothed_capture = None

		self.scalars = np.zeros((480,640))

		# calibration
		self.calibrator = Calibrator(self.calibrate_position)

		# mouse clicks
		new_params = cv2.SimpleBlobDetector_Params()
		new_params.maxArea = 800
		new_params.minArea = 50
		self.blob_detector = cv2.SimpleBlobDetector_create(new_params)
		self.im_with_keypoints = None
		self.im_with_agents = None
		self.click_db = BlobDB(touch_input_pub, self.calibrator, self.point_transform)

	def begin_calibration(self):
		self.calibrator.reset()
		self.calibrator.begin_calibration()
		self.calibrator.increment()

	def image_callback(self,msg):
		#start_time = time.time()
		img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
		irMinVal = 0.0
		irMaxVal = 1000.0
		img = img.copy()
		img[img > irMaxVal] = irMaxVal
		img[img < irMinVal] = irMinVal
		img = np.uint8(255 * (img - irMinVal) / (irMaxVal - irMinVal))

		'''
		for i in range(len(grayir_gray)):
			for j in range(len(grayir_gray[i])):
				oldval = grayir_gray[i][j]
				colorval = 0.0
				if oldval <= irMinVal:
					colorval = 0.0
				elif oldval >=irMaxVal:
					colorval = 1.0
				else:
					colorval = (255 * (oldval - irMinVal)) / (irMaxVal - irMinVal)

				img[i][j] = colorval

		img = np.uint8(img)
		'''

		if len(self.background_captures) < 10:
			self.background_captures.append(img)

			if len(self.background_captures) == 10:

				self.background_capture = np.uint8(np.mean(np.array(self.background_captures),axis=0))
				print(np.shape(np.array(self.background_capture)))

				sigma_x = 3.0
				sigma_y = 3.0
				sigma = [sigma_y, sigma_x]
				smoothed_capture = np.uint8(sp.ndimage.filters.gaussian_filter(self.background_capture, sigma, mode='constant'))	

				'''
				# determine whether each pixel is inside or outside area of interest
				max_val = 0
				aoi = np.zeros((480,640))
				for i in range(len(aoi)):
					for j in range(len(aoi[i])):
						tx,ty = self.point_transform(i,j, self.aoi_tmat)
						if not (tx < 0 or tx > 640 or ty < 0 or ty > 480):
							if smoothed_capture[i][j] > max_val:
								max_val = smoothed_capture[i][j]
				'''

				ave_val = np.average(smoothed_capture)

				# calculate the scalar array
				#print(max_val)
				print(ave_val)
				for i in range(len(self.background_capture)):
					for j in range(len(self.background_capture[i])):
						if smoothed_capture[i][j] == 0:
							scale_factor = 0
						else:
							scale_factor = 1.0*ave_val*1.0/smoothed_capture[i][j]
						self.scalars[i][j] = scale_factor
				self.scalars = np.float32(self.scalars)

				#cv2.imshow('back',self.background_capture)

		# TEST
		self.blob_capture = np.uint8(np.minimum(255,img*self.scalars))

		# get a equalized background capture
		if self.background_capture is not None:
			self.eq_background_capture = np.uint8(self.background_capture*self.scalars)

		# smooth the equalized background capture
		if self.eq_background_capture is not None:
			sigma_x = 1.0
			sigma_y = 1.0
			sigma = [sigma_y, sigma_x]
			self.smoothed_background = np.uint8(sp.ndimage.filters.gaussian_filter(self.eq_background_capture, sigma, mode='constant'))	

		# smooth the regular capture
		sigma_x = 1.0
		sigma_y = 1.0
		sigma = [sigma_y, sigma_x]
		self.smoothed_capture = np.uint8(sp.ndimage.filters.gaussian_filter(self.blob_capture, sigma, mode='constant'))	

		# subtract the background
		if self.smoothed_background is not None:
			subcap = np.absolute(np.int16(self.smoothed_background) - np.int16(self.smoothed_capture))
			#self.subtracted_capture = np.uint8(np.maximum(0,subcap-35)+35)
			subcap = np.maximum(0,subcap-85)
			subcap = np.minimum(255,4.0*subcap)
			subcap = 255-subcap
			#subcap = 10.0*subcap
			#subcap = np.maximum(0,subcap-240)
			#subcap = 10.0*subcap
			#subcap = np.minimum(255,subcap)
			self.subtracted_capture = np.uint8(subcap)

			# transform the subtracted capture so that the projected image is a perfect rectangle
			# this ensures that the size of a touch is relatively constant
			dst_img = cv2.warpPerspective(self.subtracted_capture,self.aoi_tmat, (640,480))
			#dst_img = self.subtracted_capture
			keypoints = self.blob_detector.detect(dst_img)
			self.click_db.update_list(keypoints)
			self.im_with_keypoints = cv2.drawKeypoints(dst_img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
			#self.img = cv2.warpPerspective(self.smoothed_background,self.aoi_tmat,(640,480))
		#end_time = time.time()

		#print(end_time - start_time)
	def visualize(self):
		while True:
			if self.blob_capture is not None:
				cv2.imshow('test',self.blob_capture)
			if self.im_with_keypoints is not None:
				cv2.imshow('blob', self.im_with_keypoints)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

	def set_projector_pixel_size(self,x,y):
		'''
		Set the pixel size of the projector
		'''
		self.projector_x = x
		self.projector_y = y
		print("projector set to {}x{}".format(self.projector_x,self.projector_y))
		self.set_dist_quad()

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

		# set the aoi quad
		self.aoi_quad[0][0] = 0  # top left
		self.aoi_quad[0][1] = 0
		self.aoi_quad[1][0] = 640  # top right
		self.aoi_quad[1][1] = 0
		self.aoi_quad[2][0] = 0    # bottom left
		self.aoi_quad[2][1] = 480
		self.aoi_quad[3][0] = 640  # bottom right
		self.aoi_quad[3][1] = 480

		# recalculate the transformation matrix
		self.aoi_src_quad = self.aoi_src_quad.astype(np.float32)
		self.aoi_quad = self.aoi_quad.astype(np.float32)
		self.aoi_tmat = cv2.getPerspectiveTransform(self.aoi_src_quad,self.aoi_quad)	

	def calibrate_position(self, tl, tr, br, bl):
		'''
		calibrate the location of each of the four courners of the tabletop interface
		'''
		# reset everything
		print("resetting corners of projection")
		print(tl)
		print(tr)
		print(br)
		print(bl)
		self.aoi_src_quad[0][0] = tr[0]  # top left
		self.aoi_src_quad[0][1] = tr[1]
		self.aoi_src_quad[1][0] = tl[0]  # top right
		self.aoi_src_quad[1][1] = tl[1]
		self.aoi_src_quad[2][0] = br[0]  # bottom left
		self.aoi_src_quad[2][1] = br[1]
		self.aoi_src_quad[3][0] = bl[0]  # bottom right
		self.aoi_src_quad[3][1] = bl[1]

		self.set_dist_quad()

		# send a message to the figure tracker to reset its position
		self.controller.pass_calibration_to_figure_tracker(tl,tr,br,bl)

		# create a calibration file
		caldata = {"tl":tl, "tr":tr, "bl": bl, "br": br}
		json_object = json.dumps(caldata)
		with open("tracker_calibration.json", "w") as calfile:
			calfile.write(json_object)

	def capture_background(self):
		self.background_captures = []
		self.background_capture = None

	def point_transform(self, x, y, tmat=None):

		if tmat is None:
			tmat = self.tmat
		
		a = np.array([[[x,y]]],dtype=np.float32)
		tpoints = cv2.perspectiveTransform(a,tmat)
		#print(tpoints)
		return tpoints[0][0][0],tpoints[0][0][1]

class Calibrator:

	def __init__(self, calibrate_position_callback):
		self.step = 0
		self.is_calibrating = False
		self.points = []
		self.corners = [] # in order of top left, top right, bottom right, bottom left
		self.calibrate_position_callback = calibrate_position_callback

	def begin_calibration(self):
		self.is_calibrating = True

	def add_point(self,x,y):
		self.points.append((x,y))

	def increment(self):
		self.step += 1
		if self.step == 6:
			print("points: {}".format(self.points))
			self.is_calibrating = False
			self.step = 0

			# center
			points = self.points
			center_x = points[4][0]
			center_y = points[4][1]

			tl = points[0]
			tr = points[1]
			br = points[2]
			bl = points[3]

			# compute the corners
			tl_x = tl[0] + (tl[0] - center_x)/4.0
			tl_y = tl[1] + (tl[1] - center_y)/4.0

			tr_x = tr[0] + (tr[0] - center_x)/4.0
			tr_y = tr[1] + (tr[1] - center_y)/4.0

			br_x = br[0] + (br[0] - center_x)/4.0
			br_y = br[1] + (br[1] - center_y)/4.0

			bl_x = bl[0] + (bl[0] - center_x)/4.0
			bl_y = bl[1] + (bl[1] - center_y)/4.0

			self.calibrate_position_callback((tl_x,tl_y),(tr_x,tr_y),(br_x,br_y),(bl_x,bl_y))

	def reset(self):
		self.points = []
		self.step = 0

class BlobDB:

	def __init__(self, touch_input_pub, calibrator, point_transform):

		self.click_list = []
		self.touch_input_pub = touch_input_pub
		self.calibrator = calibrator
		self.point_transform = point_transform

		# set point ages
		self.calibrator_POINT_AGE = 8
		self.normal_POINT_AGE = 6

	def update_list(self,keypoints):

		candidate_points = {}

		for kp in keypoints:
			x = kp.pt[0]
			y = kp.pt[1]

			point = self.get_candidate_point(x,y)
			if point is None:
				new_point = PotentialMouseClick(x,y,self.touch_input_pub,self.point_transform)
				self.click_list.append(new_point)
				candidate_points[new_point] = kp
			elif point not in candidate_points:
				candidate_points[point] = kp
			else:
				curr_kp = candidate_points[point]
				curr_dist = point.calculate_distance(curr_kp.pt[0],curr_kp.pt[1])
				prop_dist = point.calculate_distance(x,y)

				if prop_dist < curr_dist:
					candidate_points[point] = kp

		to_remove = []
		for point in self.click_list:
			if point not in candidate_points:
				point.break_age += 1
				if point.break_age >=2:
					to_remove.append(point)
			else:
				kp = candidate_points[point]
				point.age += 1
				point.update_point(kp.pt[0],kp.pt[1])

				# determine correct point age
				if self.calibrator.is_calibrating:
					age_lim = self.calibrator_POINT_AGE
				else:
					age_lim = self.normal_POINT_AGE

				if point.age >= age_lim and not point.is_click:
					point.confirm_click()

					# handle calibration if needed
					if self.calibrator.is_calibrating:
						self.calibrator.add_point(point.x,point.y)
						self.calibrator.increment()

		for point in to_remove:
			if point.is_click:
				point.remove_click()
			self.click_list.remove(point)

	def get_candidate_point(self,x,y):
		candidate_point = None

		for point in self.click_list:
			if point.is_same_point(x,y):
				if candidate_point is None:
					candidate_point = point
				else:
					cand_dist = candidate_point.calculate_distance(x,y)
					new_dist = point.calculate_distance(x,y)

					if new_dist < cand_dist:
						candidate_point = point

		return candidate_point


class PotentialMouseClick:

	def __init__(self,x,y,touch_input_pub,point_transform):

		self.x = x
		self.y = y
		self.tx = x
		self.ty = y
		self.seconds = int(round(time.time()))

		self.touch_input_pub = touch_input_pub
		self.point_transform = point_transform

		self.age = 0
		self.break_age = 0

		self.is_click = False

	def confirm_click(self):
		print("clicking mouse at {} - {} ({})".format(self.x,self.y,self.seconds))
		self.is_click = True
		msg = Mouse()
		msg.event_type = "click"
		self.tx,self.ty = self.point_transform(self.x,self.y)
		msg.x = int(round(self.tx))
		msg.y = int(round(self.ty))
		msg.time = self.seconds
		print("  --  transformed click: {} - {}".format(self.tx,self.ty))
		self.touch_input_pub.publish(msg)

	def remove_click(self):
		print("  --  removing mouse click at {} - {}\n".format(self.x,self.y))
		self.is_click = False
		msg = Mouse()
		msg.event_type = "release"
		msg.x = int(round(self.tx))
		msg.y = int(round(self.ty))
		msg.time = self.seconds
		self.touch_input_pub.publish(msg)

	def calculate_distance(self,x,y):

		xdiff = (self.x-x)**2
		ydiff = (self.y-y)**2

		inside = xdiff*1.0 + ydiff
		sqrt = math.sqrt(inside)
		return sqrt

	def is_same_point(self,x,y):
		if self.calculate_distance(x,y) < 30.0:
			return True
		return False

	def update_point(self,x,y):
		self.x = x
		self.y = y
