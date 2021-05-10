from synthesis.helper_superclasses.super_solve_helper import SolverHelperSuper

class SolverHelper(SolverHelperSuper):

	def __init__(self):
		super().__init__()

	def fill_cause_effect_env_lists(self,mod_list,
										 robot_mod_list,
										 human_mod_list,
										 cause_mod_list,
										 env_mod_list,
										 robot_env_mod_list,
										 human_env_mod_list,
										 other_env_mod_list):

		# hardcoded for each use case
		for mod in mod_list:
			if mod == "xpos" or mod == "ypos" or mod == "rotation":
				robot_env_mod_list.append(mod)
				env_mod_list.append(mod)
			elif mod == "h_xpos" or mod == "h_ypos":
				human_env_mod_list.append(mod)
				env_mod_list.append(mod)
			else:
				robot_mod_list.append(mod)

	def classify_event_behavior(self,label):
		if label == "":
			return "event"
		elif label == "xpos" or label == "ypos" or label == "rotation":
				return "env"
		elif label == "h_xpos" or label == "h_ypos":
			return "env"
		else:
			return "behavior"

	# override
	def get_behavior_description(self, beh, beh_val, agent):
		if beh == "speech":
			return "The robots utters a {}. ".format(beh_val)
		else:
			return ""

	# override
	def get_trace_description(self,trace):
		return "one sec"