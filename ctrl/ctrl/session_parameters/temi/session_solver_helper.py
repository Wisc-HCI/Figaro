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
			if "position" in mod or "close_to_human" in mod:    # TODO: add environmental variables later! May decide to move position over here in the future
				if mod[0] == "h" and mod[2] == "_":
					human_env_mod_list.append(mod)
				else:
					robot_env_mod_list.append(mod)
				env_mod_list.append(mod)
			elif "speech" in mod or "actions" in mod or "movement" in mod or "near_rob" in mod or "point" in mod or "tilt" in mod or "wait" in mod:
				if mod[0] == "h" and mod[2] == "_":
					human_mod_list.append(mod)
					cause_mod_list.append(mod)
				else:
					robot_mod_list.append(mod)

	def classify_event_behavior(self,label):
		if label == "":
			return "event"
		elif "position" in label or "close_to_human" in label:
			return "env"
		elif "speech" in label or "actions" in label or "movement" in label or "near_rob" in label or "point" in label or "tilt" in label or "wait" in label:
			if label[0] == "h" and label[2] == "_":
				return "event"
			else:
				return "behavior"
		else:
			print("ERROR: {} unclassifiable as event, environmental state, or robot behavior".format())

	# override
	def get_behavior_description(self, beh, beh_val, agent):
		if beh_val is None or beh_val is "False" or beh_val == False or beh_val == True:
			return ""
		elif len(beh_val) == 0:
			return ""
		else:
			if agent == "r":
				if beh == "speech":
					return "The robot utters a {}. ".format(beh_val)
				elif beh == "actions":
					return "The robot does: {}".format(beh_val)
				elif beh == "wait":
					return "The robot waits"
				elif beh == "point":
					return "The robot does points to the {}".format(beh_val)
				elif beh == "tilt":
					return "The robot tilts its head {}".format(beh_val)
				else:
					return ""
			else:
				if beh == "h1_speech":
					return "The human utters a {}. ".format(beh_val)
				elif beh == "h1_actions":
					return "The human does: {}".format(beh_val)
				else:
					return ""

	def get_robot_behaviors_description(self, beh, beh_val, tense):
		if tense == "command":
			if beh == "speech":
				return "Say: {}.".format(beh_val)
			elif beh == "action":
				return "Do the following: {}.".format(beh_val)
			elif beh == "point":
				return "Point to the {}.".format(beh_val)
			elif beh == "tilt":
				return "Tilt your head {}.".format(beh_val)
			elif beh == "wait":
				return "Wait".format(beh_val)
			elif beh == "movement":
				if len(beh_val)==0:
					return ""
				string = "Go to "
				for val in beh_val:
					string += "{}, ".format(val)
				if string[-2:] == ", ":
					string = string[:-2]
				string += "."
				return string
			else:
				return ""
		elif tense == "present":
			if beh == "speech":
				return "Says: {}.".format(beh_val)
			elif beh == "action":
				return "Does the following: {}.".format(beh_val)
			elif beh == "wait":
				return "Waits."
			elif beh == "point":
				return "Does points to: {}.".format(beh_val)
			elif beh == "tilt":
				return "Tilts its head {}.".format(beh_val)
			elif beh == "movement":
				if len(beh_val)==0:
					return ""
				string = "Goes to "
				for val in beh_val:
					string += "{}, ".format(val)
				if string[-2:] == ", ":
					string = string[:-2]
				string += "."
				return string
			else:
				return ""

	def get_conditionals_description(self,cond, cond_val):
		if cond == "position":
			if len(cond_val)==0:
				return ""
			string = "The robot is at: "
			for val in cond_val:
				string += "{}, ".format(val)
			if string[-2:] == ", ":
				string = string[:-2]
			string += "."
			return string
		if cond == "close_to_human":
			if cond_val == "False":
				return "The robot is not close to anyone."
			else:
				return "The robot is close to someone."
		return ""

	# override
	def get_trace_description(self,trace):
		return "one sec"