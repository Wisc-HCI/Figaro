from controller.modes.mode import *

class ModifyMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,controller,transition_system):
		super().__init__(name,done_callback,terminate_callback)
		self.transition_system = transition_system
		self.controller = controller
		self.state_to_modify = None
		self.type = "modify"

	def execute(self):
		# begin the determine state mode
		self.controller.determine_state("det.delete",self.done_callback)

	def notify_child_terminated(self):
		self.done_callback(None)

	def check_done(self):
		self.done_callback(None)

	def update(self, arg):
		'''
		There will be two updates:
		(1) notifying which state will be the one to be modified
		(2) performing the actual modification
		'''

		if self.state_to_modify is None:
			# record the state to modify
			self.state_to_modify = arg
			print("found state to modify")
			# begin a sketch
			self.controller.begin_sketch("sketch")
		else:
			print("getting modification and updating TS")
			modification = self.controller.get_solution_update_data()[-1]
			self.transition_system.modify(self.state_to_modify, modification)
			
			# check if done (always returns true)
			self.check_done()