from controller.modes.mode import *

class DetermineMomentMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,recorder,ff):
		super().__init__(name,done_callback,terminate_callback)
		self.recorder = recorder
		self.candidate_moment = None
		self.candidate_moment_idx = None
		self.ff = ff
		self.type = "determine_moment"

	def execute(self):
		return

	def check_done(self):
		self.done_callback(self.candidate_moment_idx)

	def update(self, arg):
		print("updating")
		if arg[0] != "determine_moment":
			return

		x = arg[1]
		y = arg[2]

		self.candidate_moment,self.candidate_moment_idx = self.recorder.find_closest_moment(x,y,self.ff)

		self.check_done()