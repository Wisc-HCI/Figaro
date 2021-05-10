from copy import deepcopy
import sys

class SessionTraceUtils:

	def __init__(self, axioms, design, objects, regions):
		self.axioms = axioms
		self.objects = objects
		self.regions = regions

		# import session signal processing rules
		sys.path.append("session_parameters/{}".format(design))
		from robot_axioms import RobotAxioms
		self.robot_axioms = RobotAxioms()

		from social_axioms import SocialAxioms
		self.social_axioms = SocialAxioms()

	def post_process_stream(self, moments, modality_info):

		'''
		AXIOMS
		'''
		# remove moments with none positions or angles
		self.robot_axioms.remove_absent_info(moments)

		# convert positions and angles to integers
		self.robot_axioms.convert_strings_to_integers(moments)

		# remove minute movement
		self.robot_axioms.remove_minute_movement(moments)

		# remove minute angle changes
		self.robot_axioms.remove_minute_angle_changes(moments)

		# create movement_time (this sets up interpolation)
		#self.robot_axioms.add_moment_time(moments)

		# interpolate
		#self.robot_axioms.interpolate_old(moments)
		self.robot_axioms.interpolate(moments)

		# make all values text
		self.robot_axioms.convert_to_text(moments)
