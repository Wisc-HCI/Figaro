import copy

class TransitionSystem:

	def __init__(self, init, states, transitions, transition_labels):
		self.states = states
		self.init = init
		self.transitions = transitions
		self.transition_function = {}
		self.transition_labels = transition_labels

		# can be linked to other callback functions
		self.update_callback = self.dummy_update_callback

		self.build_all_and_update()

	def dummy_update_callback(self):
		return

	def build_all_and_update(self):
		self.build_TS()
		self.build_transition_function()
		self.check()
		self.update_callback()
		print(str(self))

	def unbuild_TS(self):
		for transition in self.transitions:
			transition.source = None
			transition.target = None

		for state in self.states:
			state.trans_in = []
			state.trans_in_dict = {}
			state.trans_out = []
			state.trans_out_dict = {}

	def build_TS(self):
		for transition in self.transitions:
			for state in self.states:
				if state.id == transition.source_id:
					transition.source = state
					state.trans_out.append(transition)
				if state.id == transition.target_id:
					transition.target = state
					state.trans_in.append(transition)

		for state in self.states:
			for trans in state.trans_out:
				for label in trans.transition_labels:
					state.trans_out_dict[label] = trans
			for trans in state.trans_in:
				for label in trans.transition_labels:
					state.trans_in_dict[label] = trans

			for other_state in self.states:
				if other_state != state:
					for trans in state.trans_in:
						if trans.source == other_state:
							state.trans_in_dict[other_state] = trans
					for trans in state.trans_out:
						if trans.target == other_state:
							state.trans_out_dict[other_state] = trans

	def unbuild_transition_function(self):
		self.transition_function = {}

	def build_transition_function(self):
		for trans in self.transitions:
			if trans.source.id not in self.transition_function:
				self.transition_function[trans.source.id] = {}
			if trans.target is not None and trans.target.id not in self.transition_function[trans.source.id]:
				self.transition_function[trans.source.id][trans.target.id] = trans

	'''
	Editing
	'''
	def delete(self, state):

		# handle inputs
		new_transitions = []
		for trans in state.trans_in:

			# get all possible new transitions
			for condition in trans.transition_labels:
				# find the new target id
				target_id = None
				for out_trans in state.trans_out:
					for out_condition in out_trans.transition_labels:
						if condition == out_condition:
							target_id = out_trans.target.id
				if target_id is None:
					print("ERROR: deleting a state")
					exit(1)

				new_transitions.append(Transition(trans.source_id,target_id,[condition]))

		trans_to_remove = []
		for trans in state.trans_in:
			trans_to_remove.append(trans)
		for trans in state.trans_out:
			trans_to_remove.append(trans)

		self.unbuild_TS()
		self.unbuild_transition_function()

		combined_transitions = []
		transitions_removed_bc_combined = []
		for trans in new_transitions:
			if trans not in transitions_removed_bc_combined:
				for other_trans in new_transitions:
					if trans != other_trans and other_trans not in transitions_removed_bc_combined:
						if trans.source_id == other_trans.source_id and trans.target_id == other_trans.target_id:
							for other_trans_label in other_trans.transition_labels:
								trans.transition_labels.append(other_trans_label)
							transitions_removed_bc_combined.append(other_trans)
			combined_transitions.append(trans)

		for trans in combined_transitions:
			self.transitions.append(trans)

		# remove the offending state and transitions
		self.states.remove(state)
		for trans in trans_to_remove:
			self.transitions.remove(trans)

		self.build_all_and_update()

	def modify(self, state, new_material):
		'''
		Precondition is that the new material has both a start and an end state
		'''
		
		# find the start and end states
		start_state = new_material.init
		end_state = None
		for st in new_material.states:
			if len(st.trans_out) == 0:
				end_state = st

		if end_state is None:
			print("modification failed as there is no end state")
			return

		# get transitions to re-link
		start_trans_relink = []
		for trans in state.trans_in:
			start_trans_relink.append(trans)

		end_trans_relink = []
		for trans in state.trans_out:
			end_trans_relink.append(trans)

		# find the highest-numbered state in the current TS
		highest_id = -1
		for st in self.states:
			if st.id > highest_id:
				highest_id = st.id

		self.unbuild_TS()
		self.unbuild_transition_function()
		new_material.unbuild_TS()
		new_material.unbuild_transition_function()

		# renumber the states in the new material
		st_renumber_dict = {}
		curr_id = highest_id + 1
		for st in new_material.states:
			st_renumber_dict[st.id] = curr_id
			st.id = curr_id
			curr_id += 1
		for trans in new_material.transitions:
			if trans.source_id > -1:
				trans.source_id = st_renumber_dict[trans.source_id]
			if trans.target_id > -1:
				trans.target_id = st_renumber_dict[trans.target_id]
		print(st_renumber_dict)

		# add all new states and transitions
		for st in new_material.states:
			self.states.append(st)
		for trans in new_material.transitions:
			self.transitions.append(trans)

		# relink transitions
		for trans in start_trans_relink:
			trans.target_id = start_state.id
		for trans in end_trans_relink:
			trans.source_id = end_state.id

		self.states.remove(state)
		self.build_all_and_update()

	def check(self):
		is_good = True
		# check that no transitions lead to -1 or a Nonetype
		for trans in self.transitions:
			assert(trans.target_id != -1)
			assert(trans.target is not None)

		# check that -1 is not a state
		for state in self.states:
			assert(state.id != -1)

		# check that no transition has an undefined transition label

		# check that behavior on every transition label is defined for each state

		# check that no state has a duplicate id

		# check that the transition function is correct

		# check that the source/target ids match up with the sources/targets for each transition

		# check correctness of the trans in_dict and out_dict for each state
		return is_good

	def compare(self,other,allowable_deviations=[]):

		# check each one for correctness
		if not self.check() or not other.check():
			return False

		# compare inits
		if not self.init.compare_compatibility(other.init,allowable_deviations):
			return False

		# compare transition labels
		labels_matched = True
		for label in self.transition_labels:
			label_matched = False
			for other_label in other.transition_labels:
				if label.compare(other_label):
					label_matched = True
					break
			if not label_matched:
				labels_matched = False
				break
		if not labels_matched:
			return False

		# compare states and transitions within
		if not self.compare_traverse(self.init,other.init,[],allowable_deviations):
			return False

		return True

	def compare_traverse(self, curr_st, other_st, traversed,allowable_deviations):
		if not curr_st.compare_compatibility(other_st,allowable_deviations):
			return False
		elif curr_st in traversed:
			return True
		else:
			traversed.append(curr_st)

		# go through output transitions
		if len(curr_st.trans_out) != len(curr_st.trans_out):
			return False

		outputs_good = True
		for trans in curr_st.trans_out:
			matching_trans = None
			for other_trans in other_st.trans_out:
				if trans.compare_compatibility(other_trans):
					matching_trans = other_trans
					break

			if matching_trans is None:
				outputs_good = False
				break

			result = self.compare_traverse(trans.target,other_trans.target,traversed,allowable_deviations)
			if result is False:
				outputs_good = False
				break

		return outputs_good

	def copy(self):

		# copy the transition labels
		transition_labels = []
		for t_label in self.transition_labels:
			transition_labels.append(t_label.copy())

		# copy the callback
		callback = self.update_callback

		# copy the states
		states = []
		init = None
		for old_state in self.states:
			new_state = old_state.copy()
			if self.init == old_state:
				init = new_state
			states.append(new_state)

		# copy the transitions
		transitions = []
		for old_trans in self.transitions:
			new_trans = old_trans.copy()
			new_trans.transition_labels = transition_labels
			transitions.append(new_trans)

		# make the new transition system
		ts = TransitionSystem(init,states,transitions,transition_labels)
		ts.update_callback = callback

		# build the new transition system
		ts.build_TS()
		ts.build_transition_function()
		ts.check()

		# return
		return ts

	def __str__(self):
		string="<< TRANSITION SYSTEM >>\n\n<< transition labels >>\n"
		for t_label in self.transition_labels:
			string += "{}\n".format(str(t_label))
		string += "\n<< states >>\n"
		for state in self.states:
			string += "{}\n".format(str(state))
		string += "\n<< transitions >>\n"
		for trans in self.transitions:
			string += "{}\n".format(str(trans))

		return string

class State:

	def __init__(self, state_id, modalities):
		self.id = state_id
		self.modalities = ModalityPropositions(modalities)
		self.trans_in = []
		self.trans_in_dict = {}
		self.trans_out = []
		self.trans_out_dict = {}

	def compare_compatibility(self, other, allowable_deviations=[]):

		# compare modalities
		if not self.modalities.compare(other.modalities,allowable_deviations):
			return False

		return True

	def copy(self):
		propositions = self.modalities.copy()
		new_state = State(self.id,{})
		new_state.modalities = propositions
		return new_state

	def __str__(self):
		string = "{}\n".format(self.id)
		string += str(self.modalities)
		return string

class Transition:

	def __init__(self, source_id, target_id, transition_labels, probability=1.0):
		self.source_id = source_id
		self.target_id = target_id
		self.source = None
		self.target = None
		self.transition_labels = transition_labels
		self.probability = probability

	def t_labels_str(self):
		string = ""
		for t_label in self.transition_labels:
			string += "{} - ".format(str(t_label))
		string = string[:-3]
		return string

	def compare_compatibility(self,other):
		if self.probability != other.probability:
			return False

		# compare transition labels
		labels_matched = True
		for label in self.transition_labels:
			label_matched = False
			for other_label in other.transition_labels:
				if label.compare(other_label):
					label_matched = True
					break
			if not label_matched:
				labels_matched = False
				break
		if not labels_matched:
			return False

		return True

	def copy(self):
		return Transition(self.source_id,self.target_id,None,self.probability)

	def __str__(self):
		string = "{} -- ({}) --> {} (p={})".format(self.source_id,self.t_labels_str(),self.target_id,self.probability)
		return string

class TransitionLabel:

	def __init__(self, label_name):
		self.label = label_name

	def compare(self,other):
		if self.label != other.label:
			return False
		return True

	def copy(self):
		return TransitionLabel(self.label)

	def __str__(self):
		return "{}".format(self.label)

class ModalityPropositions:

	def __init__(self, prop_dict):
		self.propositions = prop_dict

	def copy(self):
		new_props = copy.copy(self.propositions)
		return ModalityPropositions(new_props)

	def compare(self, other, allowable_deviations=[]):
		if len(list(self.propositions.keys())) != len(list(other.propositions.keys())):
			return False

		props_matched = True
		for prop,val in self.propositions.items():
			if not (prop in other.propositions and val == other.propositions[prop]):

				# check allowable deviations
				allowed = False
				for ad in allowable_deviations:
					if ad["type"] == "state" and ad["mod"] == prop and ad["used"] == False:
						allowed = True
						ad["used"] = True
						break

				if not allowed:
					props_matched = False
					break

		return props_matched

	def __str__(self):
		string = ""
		for prop,val in self.propositions.items():
			string += "  {} - {}\n".format(prop,val)
		return string