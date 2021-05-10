from controller.modes.delete_mode import *
from controller.modes.modify_mode import *
from controller.modes.rewind_mode import *
from controller.modes.fastforward_mode import *
from controller.modes.sketch_mode import *
from controller.modes.scene_mode import *

from std_msgs.msg import String

class CommandHub:

	def __init__(self, controller, mode_stack):
		self.controller = controller
		self.solution_manager = controller.solution_manager
		self.mode_stack = mode_stack
		self.current_command = None

	'''
	ROS stuff
	'''
	#voice initiated commands
	def stream_record_sub_callback(self, msg):
		string = msg.data
		if "cancel" in string:
			string = "cancel"

		source = "audio"
		self.trigger_command(string, source)

	#tablet initiated commands
	def tablet_sub_callback(self, msg):
		string = msg.data
		source = "tablet"
		self.trigger_command(string, source)

	#triggers a command to be called from tablet or audio input
	#TODO: when voice commands are called in the future,
	#not all of them properly trigger the tablet to change view
	def trigger_command(self, string, source):
		#Single-part commands
		if string == "cancel":
			self.cancel(source)
		elif string == "cancel_scene":
			print("cancelling scene")
			self.controller.publish_table_record("disable")
		elif string == "undo":
			self.begin_undo()
		elif string == "redo":
			self.begin_redo()

		#physical layout commands
		elif string == "begin_physicallayout":
			self.begin_physicallayout(source)
		elif string == "end_physicallayout":
			self.end_physicallayout(source)

		#Sketch commands - sketch
		elif string == "begin_sketch":
			self.begin_sketch(source)
		elif string == "end_sketch":
			self.end_sketch(source)
		#Sketch commands - scene
		elif string == "begin_scene":
			self.begin_scene(source)
		elif string == "end_scene":
			self.end_scene(source)
		#Deploy commands
		elif string == "begin_deploy":
			self.begin_deploy(source)
		elif string == "end_deploy":
			self.end_deploy(source)

		'''
		#Sketch commands - rewind
		elif string == "begin_rewind":
			self.begin_rewind(source)
		elif string == "end_rewind":
			self.end_rewind(source)
		#Sketch commands - fastforward
		elif string == "begin_fastforward":
			self.begin_fastforward(source)
		elif string == "end_fastforward":
			self.end_fastforward(source)
		#Sketch commands - pause
		elif string == "begin_pause":
			self.begin_pause(source)
		elif string == "end_pause":
			self.end_pause(source)
		'''

		'''
		#Modify commands
		elif string == "begin_modify":
			self.begin_modify(source)
		elif string == "end_modify":
			self.end_modify(source)

		#Insert commands
		elif string == "begin_insert":
			self.begin_insert(source)
		elif string == "end_insert":
			self.end_insert(source)

		#Delete commands
		elif string == "begin_delete":
			self.begin_delete(source)
		elif string == "end_delete":
			self.end_delete(source)
		'''

	'''
	Single-part commands
	NOTE: I think we are only using cancel currently
	'''
	def cancel(self, input_mode):
		if input_mode == "audio":
			msg = String()
			msg.data = "end_" + self.mode_stack.peek_mode_type()
			self.controller.tablet_publisher.publish(msg)

		print("cancelling")
		self.controller.publish_table_record("disable")
		self.mode_stack.cancel_mode()

	'''
	Physical Layout commands
	'''
	def begin_physicallayout(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "begin_physicallayout"
			self.controller.tablet_publisher.publish(msg)
			print(msg.data)

	def end_physicallayout(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "end_physicallayout"
			self.controller.tablet_publisher.publish(msg)
			print(msg.data)

		#TODO: right now ending this mode does nothing, but I don't think it needs to

	'''
	Sketch commands:
		sketch, scene, (future: rewind, fastforward, pause)
	'''
	def begin_sketch(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "begin_sketch"
			self.controller.tablet_publisher.publish(msg)
			print(msg.data)
		sm = SketchMode("sketch",self.mode_stack.pop_mode,self.mode_stack.pop_mode,self.controller.synthesize_scenes,self.controller.data_hub.process_unaccompanied_datapass)
		self.mode_stack.add_mode(sm)

	def end_sketch(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "end_sketch"
			self.controller.tablet_publisher.publish(msg)

		self.mode_stack.update(("sketch",))

	def begin_scene(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "begin_scene"
			self.controller.tablet_publisher.publish(msg)
			print(msg.data)

		self.controller.recorder.begin_scene()

		# tell the table to begin recording! Old code below
		#self.table.enable_recording()
		self.controller.publish_table_record("enable")

		# tell the audio recorder to begin recording! Old code below
		#self.audio_in.begin_recording()
		msg = String()
		msg.data = "record"
		self.controller.audio_record_publisher.publish(msg)

	def end_scene(self, input_mode):
		# end everything
		self.controller.recorder.end_scene()
		#self.table.disable_recording()
		self.controller.publish_table_record("disable")

		# halt the audio recorder
		msg = String()
		msg.data = "end"
		print("starting audio processing")
		self.controller.audio_record_publisher.publish(msg)
		print("ending audio processing")
		
		# go ahead and process the scene if there is no audio signal
		if not self.controller.links["audio"]:
			self.controller.process_scene([],[],[],[])

		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "end_scene"
			self.controller.tablet_publisher.publish(msg)
			print(msg.data)


	'''
	Deploy commands
	'''
	def begin_deploy(self, input_mode):

		'''
		THIS FUNCTION IS NOT FINISHED
		This function contains temporary code

		In reality, this function should begin execution of the transition system (solution)
		'''
		print("telling controller to deploy")
		if self.mode_stack.is_empty():
			#update the tablet interface if the audio is the command source
			if input_mode == "audio":
				msg = String()
				msg.data = "begin_deploy"
				self.controller.tablet_publisher.publish(msg)

		self.controller.publish_begin_deploy()

	def end_deploy(self, input_mode):
		#update the tablet interface if the audio is the command source
		if input_mode == "audio":
			msg = String()
			msg.data = "end_deploy"
			self.controller.tablet_publisher.publish(msg)

		# TODO: I assume there's more to do here from the backend