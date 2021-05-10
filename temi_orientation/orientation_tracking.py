import firebase_admin
from firebase_admin import credentials
from firebase_admin import messaging
from std_msgs.msg import String

import sys

import numpy as np
import cv2
from cv2 import aruco
import math
import time

class Streamer:
	def __init__(self):
		cred = credentials.Certificate("verification-of-hri-firebase-adminsdk-y2m4o-a7e966cf63.json")
		firebase_admin.initialize_app(cred)

	def deploy(self, speech_pub):
		'''
		1) First clear the interrupts
		'''
		topic = "wipe_interrupts"
		message = messaging.Message(
			data={
				'msgType': 'wipe interrupts',
			},
			topic=topic,
		)
		# Send a message to the devices subscribed to the provided topic.
		response = messaging.send(message)
		# Response is a message ID string.
		print('Successfully sent wipe interrupts message:', response)

		

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
	Rt = np.transpose(R)
	shouldBeIdentity = np.dot(Rt, R)
	I = np.identity(3, dtype = R.dtype)
	n = np.linalg.norm(I - shouldBeIdentity)
	
	return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def rotationMatrixToEulerAngles(R) :
	assert(isRotationMatrix(R))
	sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
	singular = sy < 1e-6
	if  not singular :
		x = math.atan2(R[2,1] , R[2,2])
		y = math.atan2(-R[2,0], sy)
		z = math.atan2(R[1,0], R[0,0])
	else :
		x = math.atan2(-R[1,2], R[1,1])
		y = math.atan2(-R[2,0], sy)
		z = 0
	return np.array([x, y, z])


def main():
	#user input selects which camera we use
	camera_id = sys.argv[1]

	#based on which camera ID we have, select the appropriate base angle
	base_rot = 0
	if camera_id == 1:
		base_rot = 0
	elif camera_id == 2:
		base_rot = 0
	else:
		base_rot = 0

	#authenticate firebase
	cred = credentials.Certificate("verification-of-hri-firebase-adminsdk-y2m4o-a7e966cf63.json")
	firebase_admin.initialize_app(cred)

	#initialize opencv image stuff
	cap = cv2.VideoCapture(0)
	aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
	parameters =  aruco.DetectorParameters_create()
	size_of_marker =  0.03
	send_data = True

	#process/stream the data to Temi
	while True:
		#capture frame
		ret, frame = cap.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		if(send_data):
			
			#detect the markers and show video feed on computer
			corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
			frame_markers_ir = aruco.drawDetectedMarkers(gray, corners, ids)
			cv2.imshow('frame',frame_markers_ir)
			
			#if there is a marker found, process it
			if np.all(ids is not None):
				send_data = False
				zipped = zip(ids, corners)
				ids, corners = zip(*(sorted(zipped)))

				for i in range(0, len(ids)):  # Iterate in markers
					#estimate pose of each marker
					imsize = gray.shape
					dist = np.zeros((5,1))
					mtx = np.array([[	2000.,	0.,		imsize[0]/2.],
									[		0.,	2000.,	imsize[1]/2.],
									[		0.,	0.,		1.]])
					rvec, tvec, trash = aruco.estimatePoseSingleMarkers(corners[i], size_of_marker, mtx, dist )

					#calculate euler angle
					rmat, _ = cv2.Rodrigues(rvec)
					test = isRotationMatrix(rmat)

					#if we have an angle, send to temi
					if test:
						#calculate z rotation
						angles = rotationMatrixToEulerAngles(rmat)
						z_rot = angles[2] * 180 / math.pi
						print(z_rot)
						
						#send over Firebase
						topic = "orientation"
						message = messaging.Message(
							data={
								'id': str(camera_id),
								'angle': str(z_rot - base_rot)
							},
							topic=topic,
						)

						# Send a message to the devices subscribed to the provided topic.
						print('Preparing to send message...')
						response = messaging.send(message)
						# Response is a message ID string.
						print('Successfully sent orientation message:', response)
						send_data = True

					#otherwise, set send_data to true to try again
					else:
						send_data = True
						print("error getting z rotation")


		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	cap.release()
	cv2.destroyAllWindows()

if __name__=="__main__":
	main()

#ref angle
#input ID num
#send in firebase "ID":"angle"