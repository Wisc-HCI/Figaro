class BLEClassifier:

	def __init__(self):
		pass

	def classify(self, angle,magnitude,voltage):

		_class = {}

		# correct voltage if necessary
		if voltage == -99989:
			voltage = 500
		elif voltage == -99990:
			voltage = 10
			
		if voltage < 200: # button pressed

			if magnitude > 5000 or magnitude < -5000:
				print("tilt")
				### IN DEGREES FOR NOW
				if angle > -45 and angle <= 45:
					_class["tilt"] = "down"
				elif angle < -135 or angle > 135:
					_class["tilt"] = "up"
			else:
				print("classifying wait! {}".format(voltage))
				_class["wait"] = "True"

		else:
			if magnitude > 5000 or magnitude < -5000:
				#print("RECEIVED ANGLE: {}".format(angle))
				#print("INTEGER ANGLE: {}".format(int(round(angle))))
				integer_angle = int(round(angle))
				if integer_angle < 0:
					integer_angle = 180 + (180+integer_angle)
				#print("classifying point! {}".format(magnitude))
				_class["point"] = int(round(angle))

		return _class
