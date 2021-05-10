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
import threading
import copy
import pickle

# import shared modules
import os
import sys

from cv_bridge import CvBridge, CvBridgeError

cwd = os.getcwd()
idx = cwd.index("/figure_tracking/figure_tracking")
if idx + 32 == len(cwd):
	path =  cwd[0:idx]
	sys.path.append("{}/share".format(path))
from physical_layout import *

from figaro_msgs.msg import Mouse

class ArucoDetection:

	HUMAN_IDS = [3]
	ROBOT_ID = 12
	ALL_IDS = [3,12]

	def __init__(self,controller,physical_layout):
		self.controller = controller
		self.physical_layout = physical_layout
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
		self.distQuad = np.zeros((4,2))
		self.aoi_src_quad = np.zeros((4,2))
		self.aoi_src_quad[0][0] = 0  # top left
		self.aoi_src_quad[0][1] = 0
		self.aoi_src_quad[1][0] = 640  # top right
		self.aoi_src_quad[1][1] = 0
		self.aoi_src_quad[2][0] = 640  # bottom right
		self.aoi_src_quad[2][1] = 480
		self.aoi_src_quad[3][0] = 0  # bottom left
		self.aoi_src_quad[3][1] = 480
		self.aoi_quad = np.zeros((4,2))
		self.set_dist_quad()

		# set the initial bounds to 0
		self.left_bound = 0
		self.right_bound = 640
		self.top_bound = 0
		self.bottom_bound = 480

		# detected image components
		self.img = None
		self.ids = None
		self.corners = None

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
		self.im_with_agents = None
		self.subtracted_smoother_capture = None
		self.crop_img = None
		self.halt_capturing = False

		self.smoother_capture = None
		self.smoother_background = None
		self.raw_scalars = np.zeros((480,640))
		self.scalars = np.zeros((480,640))

		# agent tracking
		params = cv2.SimpleBlobDetector_Params()
		params.filterByConvexity = True
		params.minConvexity = 0.57
		#params.filterByColor = True
		#params.blobColor = 0
		params.filterByArea = True
		params.minArea = 200
		params.maxArea = 2000
		params.filterByInertia = True
		params.minInertiaRatio = 0.0
		self.agent_blob_detector = cv2.SimpleBlobDetector_create(params)
		self.agent_tracker = AgentTracker(self)

		# TEMPLATES
		self.robot_templates = {}
		self.human_templates = {}
		angles = ["0","15","30","45","60","75",
				  "90","105","120","135","150","165",
				  "180","195","210","225","240","255",
				  "270","285","300","315","330","345"]
		for i in angles:
			raw = cv2.imread('templates/arrow{}.png'.format(i),0)
			#processed = self.add_template_contrast(raw)
			self.robot_templates[i] = raw
			cv2.imshow(i,self.robot_templates[i])

		raw_human = cv2.imread('templates/human_noangle.png',0)
		self.human_templates["0"] = raw_human
		cv2.imshow("human_0",self.human_templates["0"])

		# mouse click event
		cv2.namedWindow("ir")

		# sensing params
		self.cv_params = {
			"subt":55,
			"mult":8.0,
			"thresh":0.8,
			"temp_subt":20,
			"temp_mult":5.0
		}
		try:
			with open("figurine_params.pkl","rb") as infile:
				self.cv_params = pickle.load(infile)
				self.cv_params["subt"]
				self.cv_params["mult"]
				self.cv_params["thresh"]
				self.cv_params["temp_subt"]
				self.cv_params["temp_mult"]
		except:
			print("could not open figurine parameters")
			self.cv_params = {
				"subt":55,
				"mult":8.0,
				"thresh":0.8,
				"temp_subt":20,
				"temp_mult":5.0
			}

		self.calibration_modes = [
			[3,(600,40)] , [3,(40,40)] , [3,(40,440)] , [3,(600,440)] , [3,(320,240)] ,
			[12,(600,40)] , [12,(40,40)] , [12,(40,440)] , [12,(600,440)], [12,(320,240)]
		]

		self.curr_calib_mode = None
		self.good_calib_params = None
		self.calibrate_results = None

		# to make things thread safe
		self.aruco_lock = threading.Lock()
		self.background_lock = threading.Lock()
		self.halt_lock = threading.Lock()
		self.calibrate_lock = threading.Lock()

	def add_template_contrast(self, img):
		#processed = np.int8(img)
		processed = sp.ndimage.filters.gaussian_filter(img, [1.0,1.0], mode='constant')
		processed = np.int16(processed)
		processed = np.maximum(0,processed-self.cv_params["temp_subt"])
		processed = self.cv_params["temp_mult"]*processed
		processed = np.minimum(255,processed)
		'''
		processed = np.int8(np.maximum(0,raw-self.cv_params["temp_subt"]))
		processed = self.cv_params["temp_mult"]*processed
		processed = np.minimum(255,processed)
		processed = np.uint8(sp.ndimage.filters.gaussian_filter(processed, [3.0,3.0], mode='constant'))
		'''
		processed = np.uint8(processed)
		return processed

	def image_callback(self,msg):
		self.halt_lock.acquire()
		if self.halt_capturing:
			self.halt_lock.release()
			return
		self.halt_lock.release()

		img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
		img = self.image_transform(img)
		old_img = img

		irMinVal = 0.0
		irMaxVal = 1000.0
		img = img.copy()
		img[img > irMaxVal] = irMaxVal
		img[img < irMinVal] = irMinVal
		img = np.uint8(255 * (img - irMinVal) / (irMaxVal - irMinVal))
		self._rawimg = img

		sigma_x = 3.0
		sigma_y = 3.0
		sigma = [sigma_y, sigma_x]
		self.background_lock.acquire()
		if len(self.background_captures) < 10:
			self.background_captures.append(img)

			if len(self.background_captures) == 10:

				self.background_capture = np.uint8(np.mean(np.array(self.background_captures),axis=0))
				print(np.shape(np.array(self.background_capture)))

				smoothed_capture = np.uint8(sp.ndimage.filters.gaussian_filter(self.background_capture, sigma, mode='constant'))	

				ave_val = np.average(smoothed_capture)

				# calculate the scalar array
				#print(max_val)
				print(ave_val)
				for i in range(len(self.background_capture)):
					for j in range(len(self.background_capture[i])):
						if smoothed_capture[i][j] == 0:
							scale_factor = 0
						else:
							scale_factor = 1.7*ave_val*1.0/smoothed_capture[i][j]
						self.raw_scalars[i][j] = scale_factor
				self.raw_scalars = np.float32(self.raw_scalars)
				self.scalars = None
		self.background_lock.release()

		if self.scalars is None: # or (len(self.scalars) != self.right_bound - self.left_bound) or (len(self.scalars[0]) != self.bottom_bound - self.top_bound): 
			self.scalars = np.copy(self.raw_scalars)

			# get the equalized background
			self.eq_background_capture = np.uint8(self.background_capture*self.scalars)

			# get the smoothed (equalized) background
			sigma_x = 3.0
			sigma_y = 3.0
			sigma = [sigma_y, sigma_x]
			self.smoother_background = np.uint8(sp.ndimage.filters.gaussian_filter(self.eq_background_capture, sigma, mode='constant'))	

		# TEST
		self.blob_capture = np.minimum(255,img*self.scalars)
		self.blob_capture = np.uint8(self.blob_capture)

		sigma_x = 3.0
		sigma_y = 3.0
		sigma = [sigma_y, sigma_x]
		self.smoother_capture = np.uint8(sp.ndimage.filters.gaussian_filter(self.blob_capture, sigma, mode='constant'))	

		# subtract the background
		self.calibrate_lock.acquire()
		if self.smoother_background is not None:
			subcap = np.absolute(np.int8(self.smoother_background) - np.int8(self.smoother_capture))
			subcap = np.maximum(0,subcap-self.cv_params["subt"])
			subcap = self.cv_params["mult"]*subcap
			subcap = np.minimum(255,subcap)
			subcap = 255-subcap
			self.subtracted_smoother_capture = np.uint8(subcap)

			keypoints = self.agent_blob_detector.detect(self.subtracted_smoother_capture)
			self.agent_tracker.update_list(keypoints=keypoints)
			self.im_with_agents = cv2.drawKeypoints(self.subtracted_smoother_capture, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

			# crop images around keypoints
			# 1) first get an image that is flattened
			old_img = img
			img = self.blob_capture
			#img = self.image_transform(img)

			# 2) then look at each keypoint within the flattened image to get the corners and IDs
			corners = None
			for kp in keypoints:

				# convert the keypoint to aoi space
				#tx,ty = self.point_transform(kp.pt[0],kp.pt[1],tmat=self.aoi_tmat)
				tx = kp.pt[0]
				ty = kp.pt[1]

				top_cutoff = abs(min(ty-40,0))
				bottom_cutoff = max(ty+40,len(img)) - len(img)
				top=max(ty-40,0) - bottom_cutoff
				bottom = min(ty + 40,len(img)) + top_cutoff

				left_cutoff = abs(min(tx-40,0))
				right_cutoff = max(tx+40,len(img[0])) - len(img[0])
				left = max(tx-40,0) - right_cutoff
				right = min(tx+40,len(img[0])) + left_cutoff
				#print("{} {} {} {}".format(top,bottom,left,right))
				crop_img = img[int(round(top)):int(round(bottom)),int(round(left)):int(round(right))]
				crop_img = self.add_template_contrast(crop_img)
				self.crop_img = crop_img
				#print(np.shape(crop_img))
				rh,angle,confidence = self.calculate_orientation(crop_img)
				#print("{} - {}".format(rh,angle))

				if rh is None:
					continue

				if corners == None:
					corners = {rh:[angle,confidence,kp]}
				else:
					if rh not in corners:
						corners[rh] = [angle,confidence,kp]
					else:
						if corners[rh][1] < confidence:
							corners[rh] = [angle,confidence,kp]

			# handle calibration
			if self.calibrate_results is not None:
				rh_array = []
				if corners is not None:
					for rh in corners:
						rh_array.append(rh)
				self.calibrate_results.append(rh_array)

			corners = self.compute_corners(corners) if corners is not None else None
			self.update_aruco_vars(corners)
		self.calibrate_lock.release()

		self.img = old_img

	def calculate_orientation(self, img):

		def get_angle(templates):
			aves = {"nothing":0}
			angle_detected = False
			for angle, template in templates.items():
				w, h = template.shape[::-1]
				res = cv2.matchTemplate(img,template,cv2.TM_CCOEFF_NORMED)
				threshold = self.cv_params["thresh"]
				loc = np.where( res >= threshold)
				vals = []
				aves[angle] = 0
				for pt in zip(*loc[::-1]):
					vals.append(res[pt[1]][pt[0]])
					angle_detected = True
				if len(vals) > 0:	
					aves[angle] = np.max(res)

			return aves,angle_detected

		r_aves,r_angle_detected = get_angle(self.robot_templates)
		h_aves,h_angle_detected = get_angle(self.human_templates)

		#print("\n")
		rh=None
		angle=None
		r_angle = max(r_aves,key=r_aves.get)
		h_angle = max(h_aves,key=h_aves.get)
		if r_angle_detected and h_angle_detected:
			# set corners and ID's
			if r_aves[r_angle] > h_aves[h_angle]:
				return 12, int(r_angle), r_aves[r_angle]
			else:
				return 3, int(h_angle), h_aves[h_angle]
		elif r_angle_detected:
			return 12, int(r_angle), r_aves[r_angle]
		elif h_angle_detected:
			return 3, int(h_angle), h_aves[h_angle]
		else:
			return None, None, None

	def compute_corners(self,corners):
		for a_id in corners:
			kp = corners[a_id][2]
			x = kp.pt[0]
			y = kp.pt[1]
			angle = corners[a_id][0]
			corners[a_id] = []
			corners[a_id].append([x+math.cos(math.radians(angle+225))*10,
								  y+math.sin(math.radians(angle+225))*10])
			corners[a_id].append([x+math.cos(math.radians(angle+315))*10,
								  y+math.sin(math.radians(angle+315))*10])
			corners[a_id].append([x+math.cos(math.radians(angle+405))*10,
								  y+math.sin(math.radians(angle+405))*10])
			corners[a_id].append([x+math.cos(math.radians(angle+495))*10,
								  y+math.sin(math.radians(angle+495))*10])

		return corners
		
	def update_aruco_vars(self,corners):
		self.aruco_lock.acquire()
		self.corners = corners
		self.aruco_lock.release()

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

		self.aoi_quad[0][0] = 0  # top left
		self.aoi_quad[0][1] = 0
		self.aoi_quad[1][0] = 640  # top right
		self.aoi_quad[1][1] = 0
		self.aoi_quad[2][0] = 640  # bottom right
		self.aoi_quad[2][1] = 480
		self.aoi_quad[3][0] = 0  # bottom left
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
		print(tl)
		print(tr)
		print(br)
		print(bl)
		self.srcQuad[0][0] = 640  # top left
		self.srcQuad[0][1] = 0
		self.srcQuad[1][0] = 0  # top right
		self.srcQuad[1][1] = 0
		self.srcQuad[2][0] = 0  # bottom right
		self.srcQuad[2][1] = 480
		self.srcQuad[3][0] = 640  # bottom left
		self.srcQuad[3][1] = 480	

		self.aoi_src_quad[0][0] = tr[0]  # top left
		self.aoi_src_quad[0][1] = tr[1]
		self.aoi_src_quad[1][0] = tl[0]  # top right
		self.aoi_src_quad[1][1] = tl[1]
		self.aoi_src_quad[2][0] = bl[0]  # bottom right
		self.aoi_src_quad[2][1] = bl[1]
		self.aoi_src_quad[3][0] = br[0]  # bottom left
		self.aoi_src_quad[3][1] = br[1]		

		# reset the bounds
		print("resetting corners of projection")
		self.left_bound = max(0,min(tl[0],bl[0]))
		self.right_bound = min(640,max(tr[0],br[0]))
		self.top_bound = max(0,min(tl[1],tr[1]))
		self.bottom_bound = min(480,max(bl[1],br[1]))
		self.set_dist_quad()

		# start a new child thread to toggle the calibration mode
		thread = threading.Thread(target=self.initiate_figurine_calibration)
		thread.daemon = True							# Daemonize thread
		thread.start()

	def initiate_figurine_calibration(self):
		print("started thread")
		self.halt_lock.acquire()
		self.halt_capturing = True
		self.halt_lock.release()

		time.sleep(3)

		print("acquiring background thread")
		self.background_lock.acquire()
		self.background_captures = []
		self.background_capture = None
		self.eq_background_capture = None
		self.smoothed_background = None
		self.subtracted_capture = None
		self.blob_capture = None
		self.smoothed_capture = None
		self.im_with_agents = None
		self.subtracted_smoother_capture = None
		self.background_lock.release()

		self.halt_lock.acquire()
		self.halt_capturing = False
		self.halt_lock.release()

		print("about to wait for background_capture to be re-made")
		while True:
			print("waiting...")
			can_break = False
			self.background_lock.acquire()
			if self.background_capture is not None:
				can_break = True
			self.background_lock.release()

			if can_break:
				break
			time.sleep(2)
		print("background capture re-made")

		print(sys.argv)
		if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
			# when the position is calibrated, we must now calibrate the figurines
			self.toggle_calibration_mode()
		else:
			print("nocalibrate")
			exit(0)

	def toggle_calibration_mode(self):
		if self.curr_calib_mode is None:
			self.curr_calib_mode = 0
			self.test_calibration()
		elif self.curr_calib_mode == 9:
			# send message to end calibration
			self.controller.publish_highlight_position((-1,-1))
			
			self.curr_calib_mode = None
			print("NEW PARAMETERS ARE: (len {})\n{}".format(len(self.good_calib_params),self.good_calib_params))
			self.select_best_params()
		else:
			print("~~~calib mode incremented~~~")
			self.curr_calib_mode += 1
			self.test_calibration()

	def select_best_params(self):
		center = copy.deepcopy(self.cv_params)
		for item in center:
			center[item] = []
		for params in self.good_calib_params:
			for item in params:
				center[item].append(params[item])

		for item in center:
			center[item] = np.average(center[item])

		print(center)
		best_distance = -1
		best_params = None

		def distance(center,params):
			diffs = []
			for item in center:
				diffs.append(abs(params[item]-center[item]))
			summa = 0
			for d in diffs:
				summa += d**2
			return math.sqrt(summa)

		for params in self.good_calib_params:
			d = distance(center,params)
			if best_distance == -1:
				best_distance = d
				best_params = params
			elif best_distance > d:
				best_distance = d
				best_params = params

		self.cv_params = best_params

		with open("figurine_params.pkl", "wb") as outfile:
			pickle.dump(self.cv_params,outfile)
		print(self.cv_params)

	def test_calibration(self):
		def test_params(subt, mult, thresh, temp_subt, temp_mult, good_calib_params):

			# change parameters
			self.calibrate_lock.acquire()
			self.cv_params["subt"] = subt
			self.cv_params["mult"] = mult
			self.cv_params["thresh"] = thresh
			self.cv_params["temp_subt"] = temp_subt
			self.cv_params["temp_mult"] = temp_mult
			self.calibrate_results = []
			self.calibrate_lock.release()

			print(self.cv_params)

			# collect images
			can_break = False
			results = []
			while True:
				self.calibrate_lock.acquire()
				if len(self.calibrate_results) >= 2:
					can_break = True
					for result in self.calibrate_results:
						results.append(result)
					self.calibrate_results = None
				self.calibrate_lock.release()
				if can_break:
					break
				time.sleep(0.1)

			# decide whether params were successful
			success = True
			true_agent = self.calibration_modes[self.curr_calib_mode][0]
			for result in results:
				if len(result) == 0 or len(result)== 2:
					success = False
					break
				if result[0] != true_agent:
					success = False
					break
			print("success? {} ({})".format(success,results))

			# store params if sucessful
			if success:
				good_calib_params.append(copy.deepcopy(self.cv_params))

		# publish to projector
		x = self.calibration_modes[self.curr_calib_mode][1][0]
		y = self.calibration_modes[self.curr_calib_mode][1][1]
		x,y = self.point_transform(x,y)
		self.controller.publish_highlight_position((x,y))

		# wait 3 seconds for user to place figurine
		time.sleep(5)

		# now calibrate
		new_params = []
		print("NEW TEST {}".format(self.curr_calib_mode))
		if self.good_calib_params is None:

			for subt in [35,45,55]:
				for mult in [7.0,8.0,9.0]:
					for thresh in [0.7,0.8]:
						for temp_subt in [0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120]:
							for temp_mult in [1.0, 3.0, 5.0, 7.0, 9.0]:
								test_params(subt, mult, thresh, temp_subt, temp_mult, new_params)
		else:
			for params in self.good_calib_params:
				test_params(params["subt"],params["mult"],params["thresh"], params["temp_subt"],params["temp_mult"], new_params)

		self.good_calib_params = new_params

		self.toggle_calibration_mode()

	def update_record(self, msg):
		string = msg
		if string == "enable":
			self.recording = True
		elif string == "disable":
			self.recording = False

	def detect(self):
		'''
		PURPOSE: detect position of actors in scene
		'''

		# UNCOMMENT TO GENERATE THE MARKERS
		'''
		fig = plt.figure()
		nx = 4
		ny = 3
		for i in range(1, nx*ny+1):
			ax = fig.add_subplot(ny,nx, i)
			img = aruco.drawMarker(aruco_dict,i, 700)
			plt.imshow(img, cmap = mpl.cm.gray, interpolation = "nearest")
			ax.axis("off")

		#plt.savefig("_data/markers.pdf")
		plt.show()
		'''

		prev_gridpoints = {}
		for ident in self.ALL_IDS:
			prev_gridpoints[ident] = None
		midpoint_was_none = {}
		midpoint = {}
		midpoint_tf = {}
		curr_gridpoints = {}
		#prev_gridpoint = None

		time.sleep(1)
		while(True):
			if self.img is not None:
				#time.sleep(1)

				# TODO: this code is wrong
				# maybe needs to be places outside loop
				for ident in self.ALL_IDS:
					midpoint_was_none[ident] = True

				cv2.imshow('IMG',self.img)
				'''
				if self.background_capture is not None:
					cv2.imshow("1background_capture", self.background_capture)
				if self.eq_background_capture is not None:
					cv2.imshow("2eq_background",self.eq_background_capture)
				if self.smoother_background is not None:
					cv2.imshow("3smoother_background", self.smoother_background)
				'''
				if self.blob_capture is not None:
					cv2.imshow("4blob_capture",self.blob_capture)
				
				'''
				if self.smoother_capture is not None:
					cv2.imshow("5smoother_capture",self.smoother_capture)
				'''
				'''
				if self.subtracted_smoother_capture is not None:
					cv2.imshow("6subt_smoother_capture",self.subtracted_smoother_capture)
				'''
				if self.im_with_agents is not None:
					cv2.imshow('7agents',self.im_with_agents)

				if self.crop_img is not None:
					cv2.imshow('crop',self.crop_img)

				# Display the resulting frame
				#cv2.imshow('frame',self.frame_markers_ir)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

				'''
				TODO: add support for multiple figurines
				'''

				for ident in self.ALL_IDS:
					midpoint[ident] = None
					midpoint_tf[ident] = None
					curr_gridpoints[ident] = None

				#midpoint = None

				self.aruco_lock.acquire()
				if self.corners is not None:
					for a_id in self.corners:
						c = self.corners[a_id]
						top_left = (c[0][0],c[0][1])
						top_right = (c[1][0],c[1][1])
						bottom_right = (c[2][0],c[2][1])
						bottom_left = (c[3][0],c[3][1])
						top = bottom = top_left[1]
						left = right = bottom_right[0]
						for pt in c:
							if pt[0] > right:
								right = pt[0]
							if pt[0] < left:
								left = pt[0]
							if pt[1] > bottom:
								bottom = pt[1]
							if pt[1] < top:
								top = pt[1]
						midpoint[a_id] = (left + (right-left),top+(bottom-top))
						#print("original midpoint: {}".format(midpoint))
						# TODO:
						# TRANSFORM THE GRIDPOINTS
						tf_x,tf_y = self.point_transform(midpoint[a_id][0],midpoint[a_id][1])
						midpoint_tf[a_id] = (int(tf_x),int(tf_y))
						self.agent_tracker.update_list(aruco_locations=midpoint_tf)
						transformed_corners = []
						top_left_x,top_left_y = self.point_transform(c[0][0],c[0][1])
						top_right_x,top_right_y = self.point_transform(c[1][0],c[1][1])
						bottom_right_x,bottom_right_y = self.point_transform(c[2][0],c[2][1])
						bottom_left_x,bottom_left_y = self.point_transform(c[3][0],c[3][1])
						c = [[top_left_x,top_left_y],[top_right_x,top_right_y],[bottom_right_x,bottom_right_y],[bottom_left_x,bottom_left_y]]
						self.agent_tracker.update_orientation("human" if a_id in self.HUMAN_IDS else "robot",c)
				self.aruco_lock.release()

				# set the new midpoint
				midpoint_tf[3] = self.agent_tracker.agent_locations["human"]
				midpoint_tf[12] = self.agent_tracker.agent_locations["robot"]

				for a_id in [3,12]:
					if midpoint_tf[a_id] is not None:
						grid_x = int(math.floor((midpoint_tf[a_id][0]*24)/self.projector_x))
						grid_y = int(math.floor((midpoint_tf[a_id][1]*16)/self.projector_y))
						curr_gridpoints[a_id] = (grid_x,grid_y)
				
				if self.recording:

					for a_id in self.ALL_IDS:

						if midpoint_tf[a_id] is not None:
							ident = "r"
							if a_id in self.HUMAN_IDS:
								ident = "h1"

							# SET THE PREV GRIDPOINT
							if prev_gridpoints[a_id] is not None:
								grid_x = curr_gridpoints[a_id][0]
								grid_y = curr_gridpoints[a_id][1]
								if grid_x != prev_gridpoints[a_id][0] or grid_y != prev_gridpoints[a_id][1]:
									print("publishming movement")
									self.controller.publish_add_to_moment(["{}movement".format("h1_" if ident == "h1" else ""),"True"])
							prev_gridpoints[a_id] = (grid_x,grid_y)
							
							# Respond to user picking up and re-placing figurine
							if midpoint_was_none[a_id]:
								midpoint_was_none[a_id] = False
							'''
								self.controller.publish_update_mode_stack_pub("{}determine_state&&&position&&&none".format("h{}_".format(a_id) if a_id in self.HUMAN_IDS else ""))
								#self.mode_stack.update(("determine_state","position",close_objects))
								self.controller.publish_update_mode_stack_pub("{}determine_moment&&&{}&&&{}".format("h{}_".format(a_id) if a_id in self.HUMAN_IDS else "", midpoint_tf[0],midpoint_tf[1]))
							'''

							#self.controller.publish_add_to_moment("{}position&&&none".format("h{}_".format(a_id) if a_id in self.HUMAN_IDS else ""))

							# TODO: determine if need to publish directly to moment from aruco tracker
							#self.controller.publish_record_curr_moment("{}".format(ident),midpoint_tf[a_id][0],midpoint_tf[a_id][1])

							corners = self.agent_tracker.agent_corners["robot" if ident == "r" else "human"]
							self.controller.publish_record_curr_position("{}".format(ident),midpoint_tf[a_id][0],midpoint_tf[a_id][1],corners)

						else:
							midpoint_was_none[a_id] = True
				else:
					for a_id in self.ALL_IDS:
						prev_gridpoints[a_id] = None
						if midpoint_tf[a_id] is not None:
							ident = "r"
							if a_id in self.HUMAN_IDS:
								ident = "h1"
							corners = self.agent_tracker.agent_corners["robot" if ident == "r" else "human"]
							self.controller.publish_record_curr_position("{}".format(ident),midpoint_tf[a_id][0],midpoint_tf[a_id][1],corners)
					#ret, frame = self.cap.read()
					#cv2.imshow('frame',frame_markers)

			else:
				if cv2.waitKey(1) & 0xFF == ord('q'):
					break

		# When everything done, release the capture
		#self.cap.release()
		cv2.destroyAllWindows()

	def point_transform(self, x, y, tmat=None):

		if tmat is None:
			tmat = self.tmat
		
		a = np.array([[[x,y]]],dtype=np.float32)
		tpoints = cv2.perspectiveTransform(a,tmat)
		#print(tpoints)
		return tpoints[0][0][0],tpoints[0][0][1]

	def image_transform(self, a):
		return cv2.warpPerspective(a,self.aoi_tmat, (a.shape[1],a.shape[0]))

class AgentTracker:

	def __init__(self,detector):
		self.detector = detector
		self.agent_locations = {"robot": None, "human": None}
		self.agent_corners = {"robot": None, "human": None}

	def update_list(self, keypoints=[], aruco_locations={}):

		'''
		Precondition: only keypoints XOR aruco locations can be provided at once
		'''
		if len(keypoints) > 0 and len(aruco_locations) > 0:
			print("ERROR: cannot provide keypoints and aruco locations at the same time")
			exit(1)

		def test_and_update_coord(x,y,agent,candidate_keypoint):
			# calculate new distance
			prev_x = self.agent_locations[agent][0]
			prev_y = self.agent_locations[agent][1]
			new_dist = self.calculate_distance(x,y,prev_x,prev_y)
			#print("candidate keypoint: {}     new_dist: {}".format(candidate_keypoint,new_dist))
			if len(candidate_keypoint) == 0 and new_dist < 1000:
			#	print("sdfsfadsf")
				candidate_keypoint.append(x)
				candidate_keypoint.append(y)
			elif new_dist < 1000:
				curr_x = candidate_keypoint[0]	
				curr_y =  candidate_keypoint[1]
				curr_dist = self.calculate_distance(curr_x,curr_y,prev_x,prev_y)
				if curr_dist > new_dist:
					candidate_keypoint[0] = x
					candidate_keypoint[1] = y

		if 3 in aruco_locations and aruco_locations[3] is not None:
			new_x = aruco_locations[3][0]
			new_y = aruco_locations[3][1]
			dist = 101
			if self.agent_locations["human"] is not None:
				prev_x = self.agent_locations["human"][0]
				prev_y = self.agent_locations["human"][1]
				dist = self.calculate_distance(new_x,new_y,prev_x,prev_y)
			if dist > 100:
				self.agent_locations["human"] = [new_x,new_y]
		if 12 in aruco_locations and aruco_locations[12] is not None:
			new_x = aruco_locations[12][0]
			new_y = aruco_locations[12][1]
			dist = 101
			if self.agent_locations["robot"] is not None:
				prev_x = self.agent_locations["robot"][0]
				prev_y = self.agent_locations["robot"][1]
				dist = self.calculate_distance(new_x,new_y,prev_x,prev_y)
			if dist > 100:
				self.agent_locations["robot"] = [new_x,new_y]

		# search through the keypoints
		candidate_robot_keypoint = []
		candidate_human_keypoint = []
		for kp in keypoints:

			# transform the kp
			x,y = self.detector.point_transform(kp.pt[0],kp.pt[1])

			if self.agent_locations["robot"] is not None:
				test_and_update_coord(x,y,"robot",candidate_robot_keypoint)
			if self.agent_locations["human"] is not None:
				test_and_update_coord(x,y,"human",candidate_human_keypoint)

		if len(candidate_robot_keypoint) > 0:
			self.agent_locations["robot"][0] = candidate_robot_keypoint[0]
			self.agent_locations["robot"][1] = candidate_robot_keypoint[1]
		if len(candidate_human_keypoint) > 0:
			self.agent_locations["human"][0] = candidate_human_keypoint[0]
			self.agent_locations["human"][1] = candidate_human_keypoint[1]

	def update_orientation(self, agent, corners):
		self.agent_corners[agent] = corners

	def calculate_distance(self, x1,y1,x2,y2):

		xdiff = (x1-x2)**2
		ydiff = (y1-y2)**2

		inside = xdiff*1.0 + ydiff
		sqrt = math.sqrt(inside)
		return sqrt
