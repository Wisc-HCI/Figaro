from controller.modes.mode import *

class SketchMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,synthesize_scenes,unaccompanied_datapass_processor):
		super().__init__(name,done_callback,terminate_callback)
		self.synthesize_scenes = synthesize_scenes
		self.unaccompanied_datapass_processor = unaccompanied_datapass_processor
		self.type = "sketch"

	def execute(self):
		print("executing sketch mode")

	def check_done(self):
		self.synthesize_scenes()
		self.done_callback(None)

	def update(self, arg):
		if arg is None:
			return
		elif arg[0] == "sketch":
			self.check_done()
		elif arg[0] == "query":
			# received a datapass unaccompanied by a query
			# pass along the message
			self.unaccompanied_datapass_processor(arg[1])
