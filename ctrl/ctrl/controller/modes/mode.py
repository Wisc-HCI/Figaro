class Mode:

	'''
	The purpose of a mode is to temporarily restrict the actions of Figaro
	to a specific subset of actions. Until the mode is complete, the only
	actions the user can perform are those specified by the mode.
	'''

	def __init__(self,name,done_callback,terminate_callback):
		self.name = name
		self.done_callback = done_callback
		self.terminate_callback = terminate_callback
		self.type = "Generic"

	def execute(self):
		return

	def terminate(self):
		'''
		Method solely meant for cleanup. 
		Cleanup is defined externally and may not be required.
		'''
		if self.terminate_callback is not None:
			self.terminate_callback()

	def notify_child_terminated(self):
		return

	def check_done(self):
		return True

	def update(self, arg):
		return