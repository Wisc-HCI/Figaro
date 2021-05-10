class SolverHelper:

	def __init__(self):
		pass

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
			elif "speech" in mod or "actions" in mod or "movement" in mod or "near_rob" in mod:
				if mod[0] == "h" and mod[2] == "_":
					human_mod_list.append(mod)
					cause_mod_list.append(mod)
				else:
					robot_mod_list.append(mod)