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
		3 types:
		global "axioms" -- processing procedures that apply to any interaction
		robot/scenario specific "robot axioms" -- processing procedures that apply to a specific robot and scenario
		robot/scenario specific "social axioms" -- a special category of robot axioms that enforce social rules
		'''
		self.axioms.discard_non_modality_tracks(moments)
		self.robot_axioms.remove_unrecognizable_objects_or_regions(moments,self.objects,self.regions)
		self.axioms.axiom_continuous_movement(moments)
		self.axioms.axiom_continuous_position(moments)
		self.axioms.axiom_no_short_points(moments)
		self.axioms.axiom_only_tilt(moments)
		self.axioms.axiom_continuous_wait(moments)
		self.axioms.axiom_constant_wait(moments)
		self.axioms.axiom_continuous_gesturing(moments)
		self.axioms.axiom_no_meaningless_movement(moments)
		self.axioms.axiom_human_position_constant_between_movement(moments)
		self.axioms.axiom_robot_position_constant_between_movement(moments)
		
		# add for slower movement
		#self.robot_axioms.ensure_position_and_movement_overlap(moments)
		
		self.axioms.axiom_remove_redundancies(moments)
		self.robot_axioms.axiom_only_final_movement_destination_matters(moments)
		self.robot_axioms.process_position_movement(moments)
		self.axioms.exclude_tracks(moments)

		# to get rid of the issue with duplicate actions
		self.axioms.instantaneous_behaviors(moments)