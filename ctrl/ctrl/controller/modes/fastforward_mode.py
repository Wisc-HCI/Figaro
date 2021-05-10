from controller.modes.mode import *

class FastForwardMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,controller,recorder):
		super().__init__(name,done_callback,terminate_callback)
		self.controller = controller
		self.recorder = recorder
		self.type = "fastforward"

	def execute(self):
		print("beginning rewind")
		# pause the recording
		self.recorder.pause_scene()

		# determine moment to rewind to
		self.controller.determine_moment("det_mo.rewind",self.done_callback,ff=True)

	def notify_child_terminated(self):
		self.done_callback(None)

	def check_done(self):
		self.recorder.resume_scene()
		self.done_callback(None)

	def update(self, arg):
		print("updating fastforward")
		# perform the rewind
		idx_to_ff = arg
		self.recorder.fastforward(idx_to_ff)
		
		# check if done (always returns true)
		self.check_done()