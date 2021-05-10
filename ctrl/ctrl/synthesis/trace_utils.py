from copy import deepcopy
import sys

class TraceUtils:

	def __init__(self, axioms, design, objects, regions):
		self.axioms = axioms
		self.objects = objects
		self.regions = regions

		# import session signal processing rules
		sys.path.append("session_parameters/{}".format(design))
		from session_trace_utils import SessionTraceUtils
		self.trace_utils = SessionTraceUtils(axioms,design,objects,regions)

	def post_process_stream(self, moments, modality_info):

		'''
		AXIOMS
		3 types:
		global "axioms" -- processing procedures that apply to any interaction
		robot/scenario specific "robot axioms" -- processing procedures that apply to a specific robot and scenario
		robot/scenario specific "social axioms" -- a special category of robot axioms that enforce social rules
		'''
		self.trace_utils.post_process_stream(moments,modality_info)