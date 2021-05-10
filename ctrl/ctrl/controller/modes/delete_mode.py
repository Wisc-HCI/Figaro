from controller.modes.mode import *

class DeleteMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,controller,transition_system):
		super().__init__(name,done_callback,terminate_callback)
		self.transition_system = transition_system
		self.controller = controller
		self.type = "delete"

	def execute(self):
		# begin the determine state mode
		self.controller.determine_state("det.delete",self.done_callback)

	def notify_child_terminated(self):
		self.done_callback(None)

	def check_done(self):
		self.done_callback(None)

	def update(self, arg):
		# perform the deletion
		state_to_delete = arg
		self.transition_system.delete(state_to_delete)
		
		# check if done (always returns true)
		self.check_done()