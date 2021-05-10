from synthesis.helper_superclasses.super_glue_generalizer import GlueGeneralizerSuper

class GlueGeneralizer(GlueGeneralizerSuper):

	def __init__(self):
		super().__init__()

	def generalize(self,glue,event_beh_classifier):

		'''
		Rule 1: For states with a SINGLE outgoing transition of a particular event,
		        generalize the environmental state within that transition
		(NOTE: if the state involves movement, do not generalize the position)
		'''

		for gstate in glue.gstates:

			# is it a single-output state?
			single_output = False
			gstate_action_structure = gstate.actions
			actions = [] # assemble a list of actions
			for label,action_list in gstate_action_structure.items():
				for action in action_list:
					if "sys" in action.target.behaviors and action.target.behaviors["sys"] == "OFF" and main:
						continue
					actions.append(action)
			action_dict = {} # key = event, values = list of actions
			
			for action in actions:
				for i in range(len(action.action_label)):
					label = action.action_label[i]
					val = action.action_label[i]
					if event_beh_classifier(label) == "event":
						if label not in action_dict:
							action_dict[label] = {}
						if val not in action_dict[label]:
							action_dict[label][val] = []
						action_dict[label][val].append(action)
			
			candidate_actions = []
			keylist = action_dict.keys()
			for label in keylist:
				vallist = action_dict[label].keys()
				for val in vallist:
					if len(action_dict[label][val]) == 1:
						candidate_actions.append(action_dict[label][val][0])

			if len(action_dict) == 0:
				continue

			# does the state contain movement?
			is_movement = False
			if "movement" in gstate.behaviors and len(gstate.behaviors["movement"]) > 0:
				is_movement = True

			for action in candidate_actions:
				new_cats = []
				new_vals = []
				for i in range(len(action.action_label)):
					label = action.action_label[i]
					value = action.action_val[i]

					if event_beh_classifier(label) != "env" or (is_movement and label == "position"):
						new_cats.append(label)
						new_vals.append(value)
				new_cats_tup = tuple(new_cats)
				new_vals_tup = tuple(new_vals)
				action.action_label = new_cats_tup
				action.action_val = new_vals_tup

		return glue