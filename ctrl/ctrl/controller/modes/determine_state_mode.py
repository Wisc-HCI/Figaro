from controller.modes.mode import *

class DetermineStateMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,transition_system,input_modalities):
		super().__init__(name,done_callback,terminate_callback)
		self.transition_system = transition_system
		self.modality_exclusion_criteria = {}
		self.other_exclusion_criteria = []
		self.candidate_states = []
		self.type = "determine_state"

		for inp in input_modalities:
			self.modality_exclusion_criteria[inp] = None

	def execute(self):
		return

	def check_done(self):
		if len(self.candidate_states) != 1:
			return 
		self.done_callback(self.candidate_states[0])

	def update(self, arg):
		if arg[0] != "determine_state":
			return
		self.modality_exclusion_criteria[arg[1]] = arg[2]
		for state in self.transition_system.states:
			is_good = True
			for mod,val in self.modality_exclusion_criteria.items():
				if val is not None:
					if state.modalities.propositions[mod] != val:
						is_good = False
			if is_good:
				self.candidate_states.append(state)

		print(self.modality_exclusion_criteria)
		print(self.candidate_states)

		self.check_done()