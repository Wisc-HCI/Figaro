from controller.modes.mode import *

class SceneMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,begin_scene,end_scene):
		super().__init__(name,done_callback,terminate_callback)
		self.begin_scene = begin_scene
		self.end_scene = end_scene
		self.type = "scene"

	def execute(self):
		self.begin_scene()

	def check_done(self):
		self.end_scene()
		self.done_callback(None)

	def update(self, arg):
		if arg is None:
			return
		if arg[0] == "scene":
			self.check_done()