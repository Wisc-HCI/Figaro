from controller.modes.mode import *

class RewindMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,controller,recorder):
		super().__init__(name,done_callback,terminate_callback)
		self.controller = controller
		self.recorder = recorder
		self.type = "rewind"

	def execute(self):
		print("beginning rewind")
		# pause the recording
		self.recorder.pause_scene()

		# determine moment to rewind to
		self.controller.determine_moment("det_mo.rewind",self.done_callback)

	def notify_child_terminated(self):
		self.done_callback(None)

	def check_done(self):
		self.recorder.resume_scene()
		self.done_callback(None)

	def update(self, arg):
		print("updating rewind")
		# perform the rewind
		idx_to_rewind = arg
		self.recorder.rewind(idx_to_rewind)
		
		# check if done (always returns true)
		self.check_done()