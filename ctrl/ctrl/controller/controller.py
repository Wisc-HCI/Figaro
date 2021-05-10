import threading
import time
import sys
import json

from controller.recorder import Recorder
from controller.data_hub import DataHub
from controller.save_load_manager import *
from controller.input_modality_handler import *
from controller.command_processor import *
from controller.mode_stack import *
from controller.modes.determine_state_mode import *
from controller.modes.determine_moment_mode import *
from controller.modes.query_mode import *
from controller.solution_manager import *
from controller.speech_database import *
from controller.ts_exporter import *
from controller.deployer import *
from controller.ble_input import *
from synthesis.speech_utils import *
from synthesis.solver import *

from rclpy.node import Node

# Figaro messages
from std_msgs.msg import String, Int32, Float64
from geometry_msgs.msg import Point
from figaro_msgs.msg import StringArray
from figaro_msgs.msg import AgentPosition
from figaro_msgs.msg import AudioContent
from figaro_msgs.msg import QueryArray
from figaro_msgs.msg import Query
from figaro_msgs.msg import PhysicalLayout
from figaro_msgs.msg import ItemLocations
from figaro_msgs.msg import PathComponentArray
from figaro_msgs.msg import PathComponent
from figaro_msgs.msg import SceneSummaryArray
from figaro_msgs.msg import SceneSummary
from figaro_msgs.msg import DisplayContent
from figaro_msgs.msg import DataPassUndefinedIntent
from figaro_msgs.msg import DataPass
from figaro_msgs.msg import DataPassArray
from figaro_msgs.msg import DataPassAllIntentCategories
from figaro_msgs.msg import QueryUndefinedIntent
from figaro_msgs.msg import Figurines

class Controller(Node):

	def __init__(self):

		###################################
		# set up publishers and subscribers
		###################################
		super().__init__('controller_node')

		# initialize the mode stack
		self.mode_stack = ModeStack()

		# initialize the solution manager
		self.solution_manager = SolutionManager()

		# initialize the command hub
		self.command_hub = CommandHub(self, self.mode_stack)

		# initialize command interface subscriber
		self.audio_stream_subscriber = self.create_subscription(String, 'audio_stream_pub_ctrl',self.command_hub.stream_record_sub_callback, 10)

		# initialize node link subscriber (when a new ROS node links to ctrl, a notification is given)
		self.linker_subscriber = self.create_subscription(String, "node_link_ctrl", self.link_node, 10)

		# get and acknowledge the size of the projector
		self.pixel_size_receiver = self.create_subscription(Point, 'projector_pub_everyone_pixelsize', self.pixel_size_subsc_callback, 10)
		self.pixel_size_ack = self.create_publisher(String, 'ctrl_pub_projector_ack_pixelsize', 10)
		self.pixelx = 1280
		self.pixely = 960

		# initialize the database in which speech categories and examples are maintained
		self.speech_db = SpeechDatabase()
		self.intent_subscriber = self.create_subscription(StringArray, 'audio_intent_pub_ctrl', self.populate_speech_database_callback, 10)

		# tablet
		self.tablet_publisher = self.create_publisher(String, 'ctrl_pub_tablet', 10)
		self.tablet_subscriber = self.create_subscription(String,'tablet_pub_ctrl', self.command_hub.tablet_sub_callback, 10)
		self.layout_subscriber = self.create_subscription(PhysicalLayout, 'tablet_layout_pub_ctrl', self.layout_sub_callback, 10)
		self.scene_summary_subscriber = self.create_subscription(String, 'tablet_scene_summary_pub_ctrl', self.receive_scene_summary_request_callback, 10)
		self.scene_summary_publisher = self.create_publisher(SceneSummaryArray, 'ctrl_scene_summary_pub_tablet', 10)
		self.scene_info_subscriber = self.create_subscription(Int32, 'tablet_scene_info_pub_ctrl', self.receive_scene_info_request_callback, 10)
		self.scene_info_publisher = self.create_publisher(PathComponentArray, 'ctrl_scene_info_pub_tablet', 10)
		self.delete_request_subscriber = self.create_subscription(String, 'tablet_pub_ctrl_delete_request', self.delete_scene_callback, 10)
		self.publish_grid_size = self.create_publisher(Point,'ctrl_pub_grid_size',10)
		self.publish_proj_grid_size = self.create_publisher(Point,'ctrl_pub_projector_grid_size',10)

		# passing layout to other nodes
		self.ctrl_layout_publisher = self.create_publisher(PhysicalLayout, 'ctrl_layout_pub', 10)

		# figure tracker
		self.update_figurine_vis_pub = self.create_publisher(Figurines,'ctrl_figurines_projector_pub',10)
		self.table_record_publisher = self.create_publisher(String, 'ctrl_pub_tracker_record', 10)
		self.table_add_to_moment_subscriber = self.create_subscription(StringArray, 'tracker_pub_ctrl_add_to_moment', self.table_sub_add_to_moment_callback, 10)
		self.table_record_curr_moment_subscriber = self.create_subscription(AgentPosition,'tracker_pub_ctrl_record_moment', self.table_sub_record_curr_moment_callback, 10)
		self.table_update_mode_stack_subscriber = self.create_subscription(String, 'tracker_pub_ctrl_mode_stack', self.table_sub_update_mode_stack_callback, 10)

		# audio
		self.audio_record_publisher = self.create_publisher(String, 'ctrl_pub_audio_record', 10)
		self.audio_record_subscriber = self.create_subscription(AudioContent,'audio_record_pub_ctrl',self.audio_record_sub_callback,10)
		self.audio_display_subscriber = self.create_subscription(DisplayContent, 'audio_display_pub_ctrl', self.display_content_sub_callback, 10)
		self.audio_display_publisher = self.create_publisher(DisplayContent, 'ctrl_pub_audio_display', 10)
		self.highlight_agent_sub = self.create_subscription(DisplayContent, 'audio_pub_ctrl_highlight_agent', self.highlight_agent_callback, 10)
		self.highlight_agent_pub = self.create_publisher(DisplayContent, 'ctrl_pub_projector_highlight_agent', 10)
		self.data_hub = DataHub(self)
		self.throwaway_speech_sub = self.create_subscription(String, 'projector_pub_ctrl_throwaway_speech', self.throwaway_speech_callback, 10)
		self.throwaway_speech_pub = self.create_publisher(String, 'ctrl_pub_audio_throwaway_speech', 10)
		self.flag_speech_sub = self.create_subscription(String, 'projector_pub_ctrl_flag_speech', self.flag_speech_callback, 10)
		self.flag_speech_pub = self.create_publisher(String, 'ctrl_pub_audio_flag_speech', 10)

		# pausing
		# handle pausing
		self.request_pause_sub = self.create_subscription(String, 'projector_pub_ctrl_pause', self.request_pause_callback, 10)
		self.send_pause_path_pub = self.create_publisher(PathComponentArray, 'ctrl_pub_projector_path', 10)
		self.request_resume_sub = self.create_subscription(Int32, 'projector_pub_ctrl_resume', self.request_resume_callback, 10)
		self.pause_pub = self.create_publisher(String, 'ctrl_pub_audio_pause', 10)
		self.rewind_pub = self.create_publisher(Float64, 'ctrl_pub_audio_rewind', 10)

		# figurines
		self.figurine_point_sub = self.create_subscription(String, 'projector_figurine_point_pub', self.figurine_point_callback,10)

		# store figurine info here so that we know what to send
		self.is_waiting = True
		self.point_val = "none"
		self.tilt_val = "none"

		# saving and loading
		load_folder = "session_parameters/{}".format(sys.argv[1])
		self.filemanager = FileManager(self, load_folder)

		# import ble rules
		sys.path.append("session_parameters/{}".format(sys.argv[1]))
		from ble_classifier import BLEClassifier
		self.ble_classifier = BLEClassifier()

		# bluetooth controllers
		self.ble = BLEInput(self.ble_callback)

		# deployment
		self.deployment_preparer = self.create_publisher(String, '/deploy/messages',10)
		self.speech_phrase_publisher = self.create_publisher(String,"/deploy/speech_phrases",10)
		self.deployer = Deployer(self.deployment_preparer, self.speech_phrase_publisher)

		###################################
		# finish initialization of controller
		###################################
		# read in the input modalities json data
		input_modality_handler = InputModalityHandler(self.filemanager.curr_param_folder)
		self.modalities = input_modality_handler.get_input_modality_dict()

		# read in the grid size
		try:
			dimensions_json = open("{}/dimensions.json".format(self.filemanager.curr_param_folder),"r")
			grid_size = json.load(dimensions_json)
			self.grid = Point()
			self.grid.x = float(grid_size["x"])
			self.grid.y = float(grid_size["y"])
		except:
			print("ERROR: could not load grid size")
			exit(1)

		# read in the remaining session parameters
		design = sys.argv[1]

		# initialize the recorder
		self.recorder = Recorder(self.modalities, design)

		# initialize the ts exporter
		self.ts_exporter = TSExporter()

		# initialize the trace components
		self.solver = Solver(self.modalities, design, self.speech_db)

		# initlialize the synthesizer
		self.speech_utils = SpeechUtils()

		# keep track of which nodes are linked to ctrl
		self.links = {
			"audio": False,
			"server": False,
			"projector": False,
			"figure tracking": False,
			"touch tracking": False,
			"ROS1 Bridge": False
		}

		# send a ping to any already existing nodes out there
		self.linker_ping = self.create_publisher(String, "ctrl_link_node", 10)
		thread = threading.Thread(target=self.ping_nodes_on_loop, args=(20,))
		thread.daemon = True
		thread.start()

		###################################
		# other initialization
		###################################
		self.scene_process_lock = False

	def ping_nodes_on_loop(self, num_pings):
		i = 0
		msg = String()
		msg.data = "ping"
		print("pinging nodes {} times".format(num_pings))
		while i < num_pings:
			self.linker_ping.publish(msg)
			time.sleep(1)

			if all(self.links.values()):
				break

			i += 1
		print("done pinging")
		print(self.links)

	def link_node(self, msg):
		'''
		ROS2 Callback function that tells ctrl which nodes have been connected
		'''
		data = msg.data
		if self.links[data]:
			return
		self.links[data] = True
		if data == "server":
			print("publishing grid size to server")
			self.publish_grid_size.publish(self.grid)
		if data == "projector":
			print("publishing grid size to projector")
			self.publish_proj_grid_size.publish(self.grid)
		print("{} linked".format(data))

	def populate_speech_database_callback(self, intent_msg):
		array = intent_msg.array
		for intent in array:
			self.speech_db.learned_intents.append(intent)

	def process_scene(self, robot_speech_segments, robot_action_segments, human_speech_segments, human_action_segments):

		def add_speech_and_actions_to_moments(speech_segments, action_segments, prefix=""):
			speech_cats = self.speech_utils.discretize_speech(speech_segments,self.modalities)
			for item in speech_cats:
				print("adding {} to previous moments".format(item["cat"]))
				self.recorder.add_to_previous_moments("{}speech".format(prefix),item["cat"],item["start"],item["end"])
			#self.recorder.print_moments()

			for item in action_segments:
				self.recorder.add_to_previous_moments("{}actions".format(prefix),item["action"],item["start"],item["end"])

		# incorporate speech and action segments into the recording
		add_speech_and_actions_to_moments(robot_speech_segments, robot_action_segments)
		add_speech_and_actions_to_moments(human_speech_segments, human_action_segments, prefix="h1_")

		# visualize the recordings on a timeline
		self.recorder.visualize_current_recording("before_{}".format(len(self.solver.trace_components.demos)))

		# create the sequence and then the trace
		self.recorder.post_process()
		conflicts = self.solver.make_trace(self.recorder.curr_recording)
		#self.recorder.print_moments()

		# visualize the recordings on a timeline
		self.recorder.visualize_current_recording("after_{}".format(len(self.solver.trace_components.demos)))

		# unlock the scene process lock
		self.scene_process_lock = False

		# resolve any conflicts if they exist
		if conflicts is not None:
			# TEMPORARY BAND-AID: discard the most recent trace
			#print("DELETING PREVIOUS TRACE DUE TO CONFLICT")
			#last_trace = self.solver.trace_components.demos[-1]
			#print(last_trace.pretty_string())
			#self.solver.trace_components.demos.remove(last_trace)
			self.initiate_conflict_resolution(conflicts,self.conflict_resolution_callback,None)
		else:
			# send blank query
			print("sending a blank conflict query")
			qmsg = QueryArray()
			self.data_hub.publish_query(qmsg,"tablet")

	def synthesize_scenes(self):

		# wait for the lock to be False
		while self.scene_process_lock:
			pass

		# call synthesizer and visualization
		sketch = self.solver.glue_traces()
		if sketch is not None:
			self.solution_manager.add_solution(sketch)

	def record_current_moment_position(self,ident,x,y,theta):
		self.recorder.record_current_moment_position(ident,x,y,theta)

	def add_to_current_moment(self, event_type, event_val):
		self.recorder.add_to_current_moment(event_type,event_val)

	def visualize(self, sketch):
		# must have web browser open to see visualization
		self.solution_manager.solution.export_links()

	'''
	getters and setters
	'''
	def get_solution(self):
		return self.solution_manager.get_solution()

	def get_solution_update_data(self):
		return self.solution_manager.get_solution_update_data()

	def get_recorder(self):
		return self.recorder

	'''
	modes
	'''
	def determine_state(self,name,done_callback):
		dsm = DetermineStateMode(name,done_callback,None,self.get_solution(),self.modalities)
		self.mode_stack.add_mode(dsm)

	def determine_moment(self,name,done_callback,ff=False):
		dsm = DetermineMomentMode(name,done_callback,None,self.get_recorder(),ff)
		self.mode_stack.add_mode(dsm)

	def begin_scene_mode(self,name):
		sm = SceneMode(name,self.mode_stack.pop_mode,self.end_scene,self.begin_scene,self.end_scene)
		self.mode_stack.add_mode(sm)

	def end_scene_mode(self, input_mode):
		# introduce a locking mechanism to ensure that if synthesis is called before
		# a scene is fully processed, it waits for the scene to be processed
		self.scene_process_lock = True
		self.mode_stack.update(("scene",))

	'''
	BLE
	'''
	def ble_callback(self, timestamp, rh, magnitude, angle, voltage):

		modality_dict = self.ble_classifier.classify(angle,magnitude,voltage)

		fig_msg = Figurines()
		if "wait" not in modality_dict:
			fig_msg.wait = False
		if "point" not in modality_dict:
			fig_msg.point = False
		if "tilt" not in modality_dict:
			fig_msg.tilt = False

		# do something if there is a classification
		new_wait_val = False
		new_point_val = "none"
		new_tilt_val = "none"
		if len(modality_dict) > 0: 
			# notify projector

			for item in modality_dict:
				if item == "wait":
					print("starting WAIT")
					fig_msg.wait = True
					new_wait_val = True
				if item == "point":
					print("starting POINT")
					fig_msg.point = True
					fig_msg.angle = modality_dict[item]
					new_point_val = modality_dict[item]
				if item == "tilt":
					print("starting TILT")
					fig_msg.tilt = True
					fig_msg.direction = modality_dict[item]
					new_tilt_val = modality_dict[item]

		if (new_wait_val != self.is_waiting) or (new_point_val != self.point_val) or (new_tilt_val != self.tilt_val):
			self.update_figurine_vis_pub.publish(fig_msg)
			self.is_waiting = new_wait_val
			self.point_val = new_point_val
			self.tilt_val = new_tilt_val

		# add to recording
		for mod,val in modality_dict.items():
			# PUBLISH TILT AND WAIT
			# BUT DO NOT PUBLISH POINT (projector must be the one to publish point)
			if mod != "point":
				print('adding to CURRENT moment (close enough to real time)')
				if self.recorder.curr_recording is not None and self.recorder.curr_recording.rolling:
					self.recorder.add_to_current_moment(mod,val)

			# handle passing this information to anyone else as dictated by the session parameters

	def figurine_point_callback(self,msg):
		mod = "point"
		val = msg.data
		self.recorder.add_to_current_moment(mod,val)

	'''
	ROS business
	'''
	def layout_sub_callback(self, msg):
		'''
		Update the physical layout
		'''

		# do any preprocessing necessary
		# collect lists of regions and objects
		objects = []
		regions = []
		for item_loc in msg.object_data:
			objects.append(item_loc.name)
		for region_loc in msg.region_data:
			regions.append(region_loc.name)
		print("initialized objects and regions")
		print(objects)
		self.recorder.initialize_objects_regions(objects,regions)

		# pass the message to other nodes
		self.ctrl_layout_publisher.publish(msg)

	def publish_table_record(self, data):
		msg = String()
		msg.data = data
		self.table_record_publisher.publish(msg)

	def table_sub_add_to_moment_callback(self, msg):
		string_array = msg.array
		event_type = string_array[0]
		if event_type not in self.modalities:
			return # discard modalities that the session does not use

		if len(string_array) == 1:
			event_val = None
		elif len(string_array) == 2 and not self.modalities[event_type]["array"]:  # single-valued variable
			event_val = string_array[1]
		elif self.modalities[event_type]["array"]:    # array-valued variable
			event_val = string_array[1:]
		else:
			print("ERROR: received array for single-valued modality or vice versa")
			exit(1)

		# TODO: ensure that multiple vals can be added
		self.add_to_current_moment(event_type, event_val)

	def table_sub_record_curr_moment_callback(self, msg):
		ident = msg.identifier
		position = msg.position
		x = int(position.x)
		y = int(position.y)
		theta = int(position.z)
		self.record_current_moment_position(ident, x, y, theta)

	def table_sub_update_mode_stack_callback(self, msg):
		string = msg.data
		item1 = string[0:string.index("&&&")]

		start_end = string[string.index("&&&")+3:]
		item2 = start_end[0:start_end.index("&&&")]

		item3 = start_end[start_end.index("&&&")+3:]

		self.mode_stack.update((item1,item2,item3))

	def audio_record_sub_callback(self, msg):
		robot_speech_segments = []
		robot_action_segments = []
		human_speech_segments = []
		human_action_segments = []

		# must resolve uncertainties with these speech segments
		uncertain_speech_segments = []

		for figure_content in msg.figure_data:
			if figure_content.figure == "robot":
				speech_segments = robot_speech_segments
				action_segments = robot_action_segments
			elif figure_content.figure == "h1":
				speech_segments = human_speech_segments
				action_segments = human_action_segments

			# handle speech first
			speech_msg = figure_content.speech
			for speech in speech_msg.array:
				seg = {}
				seg["speech"] = speech.speech
				seg["start"] = speech.starttime
				seg["end"] = speech.endtime

				if seg["speech"] == "UNDEFINED":
					seg["whole_text"] = speech.whole_text
					seg["text"] = speech.text
					uncertain_speech_segments.append(seg)
				else:
					speech_segments.append(seg)

			# now handle actions
			action_msg = figure_content.actions
			for action in action_msg.array:
				seg = {}
				seg["action"] = action.intent
				seg["start"] = action.starttime
				seg["end"] = action.endtime

				action_segments.append(seg)

		# activate the audio_query_mode
		print(uncertain_speech_segments)
		if len(uncertain_speech_segments) == 0:
			# send an empty query, do not go into query mode
			query = QueryArray()
			print("sending a blank undefined intent query")
			self.data_hub.publish_query(query,"tablet")

			# hacky
			time.sleep(1)

			self.process_scene(robot_speech_segments, robot_action_segments, human_speech_segments, human_action_segments)
			#navigate tablet back to sketch
			msg = String()
			msg.data = "begin_sketch"
			self.tablet_publisher.publish(msg)
		else: # set up a query to resolve uncertainties
			# start a new mode
			data = (robot_speech_segments, robot_action_segments, human_speech_segments, human_action_segments)
			message = QueryArray()
			for seg in uncertain_speech_segments:
				query = Query()
				query.destination = "tablet"
				query.type = "UndefinedIntentQuery"
				qdata = query.undefined_intent_query

				# add the text
				qdata.whole_text = seg["whole_text"]

				# add the content array
				# TODO: improve this in case there are duplicate texts of interest within the whole text
				text_sarray = StringArray()
				char_start_idx = seg["whole_text"].index(seg["text"])
				split_whole = seg["whole_text"].split()
				counter = 0
				start_idx = 0
				for word in split_whole:
					for c in word:
						counter += 1
					counter += 1
					start_idx += 1
					if counter == char_start_idx:
						break
				end_idx = len(split_whole)
				for i in range(len(seg["whole_text"].split())):
					if i >=start_idx and i < end_idx:
						text_sarray.array.append("query")
					else:
						text_sarray.array.append("none")
				qdata.text_class = text_sarray

				# add whether it is a robot or a human
				rob_hum_sarray = "Robot"

				# add the categories
				cat_sarray = StringArray()
				for cat in self.speech_db.get_all_intents():
					cat_sarray.array.append(cat)
				qdata.intent_categories = cat_sarray

				# add a list of user-defined categories
				user_cat_sarray = StringArray()
				for cat in self.speech_db.provided_intents:
					user_cat_sarray.array.append(cat)
				qdata.user_defined_intent_categories = user_cat_sarray

				# add the list of phrases from each category
				q_supporting_data = qdata.user_defined_speech_examples
				for cat,provided_intent in self.speech_db.provided_intents.items():
					sys_user_defined_sarray = StringArray()
					for speech in provided_intent.speech:
						sys_user_defined_sarray.array.append(speech)
					q_supporting_data.append(sys_user_defined_sarray)

				# add the path to this query
				path = PathComponentArray()
				speech_moment_starttime = seg["start"]
				speech_moment = None
				for moment in self.recorder.curr_recording.moment_history:
					if moment.start_time == speech_moment_starttime:
						speech_moment = moment
				self.record_path_component_array(path,self.recorder.curr_recording.moment_history,self.recorder.curr_recording,criteria=speech_moment)
				qdata.path = path

				print(qdata)
				message.queries.append(query)

			held_data = [uncertain_speech_segments,data]
			qm = QueryMode("UndefinedIntentQuery",self.mode_stack.pop_mode,None,self.data_hub.publish_query,self.fix_undefined_intents,message,"tablet",held_data)
			self.mode_stack.add_mode(qm)

	def fix_undefined_intents(self, datapass_msg, held_data):
		print("fixing undefined intents")
		uncertain_speech_segments = held_data[0]
		data = held_data[1]
		if len(uncertain_speech_segments) != len(datapass_msg.data_bundle):
			print("ERROR: the number of queries does not match the number of answers")
			exit()
		
		'''
		iterate through each query/answer
		'''
		for i in range(len(datapass_msg.data_bundle)):
			ans = datapass_msg.data_bundle[i]

			undefined_intent_data = ans.undefined_intent_data
			label = undefined_intent_data.label
			speaker = undefined_intent_data.speaker
			new_cat = undefined_intent_data.new_cat
			speech = undefined_intent_data.speech_examples.array

			if label == "None":
				continue

			# determine if new cat
			if new_cat:
				print("new provided intent!")
				print(label)
				print(speech)
				print("{} - {}".format(label,speech))
				new_intent = ProvidedIntent(label,speech)
				self.speech_db.provided_intents[label] = new_intent

			seg = uncertain_speech_segments[i]
			start_time = seg["start"]
			end_time = seg["end"]
			new_seg = {"speech":label,"start":start_time,"end":end_time}

			# determine where to store it
			if speaker == "Robot":
				data[0].append(new_seg)
			else:
				data[2].append(new_seg)

		self.process_scene(data[0], data[1], data[2], data[3])

	def initiate_conflict_resolution(self,conflicts,resolved_callback,supplementary_data):
		print("initiating conflict resolution")

		message = QueryArray()

		# can only process one conflict query at a time
		query = Query()
		query.type = "ConflictQuery"
		query.destination = "tablet"

		qconflict = query.conflict_query

		# base robot actions, cause, and the path off of a single trace
		sample_trace = conflicts.conflicts[0][0]  # index 0 is the first cluster, index [0][0] is the first trace in the cluster

		# get a list of robot action descriptors
		rob_actions_array = []
		# grab the behavior right before the conflict
		if conflicts.index == 0:
			rob_actions_array.append("nothing")
		else:
			behavior_state = sample_trace.demo_array[conflicts.index - 1][1]
			for behavior_key,behavior_val in behavior_state.output_dict.items():
				rob_actions_array.append("{}".format(self.solver.solver_helper.get_robot_behaviors_description(behavior_key,behavior_val,"present")))
			if rob_actions_array == [""]:
				rob_actions_array = ["nothing yet."]

		# get a list of cause descriptors
		cause_array = []
		# grab the action right before the conflict
		cause = sample_trace.demo_array[conflicts.index][0]
		for cause_key,cause_val in cause.input_dict.items():
			cause_array.append("{}".format(self.solver.solver_helper.get_conditionals_description(cause_key,cause_val)))

		# get a list of options for robot behaviors
		behavior_options = []
		beh_string_2_dict = {}  # dictionary containing behavior strings mapped to dictionaries
		for cluster in conflicts.conflicts:
			for trace in cluster:
				behavior_state = trace.demo_array[conflicts.index][1]
				behavior_string = ""
				for behavior_key,behavior_val in behavior_state.output_dict.items():
					behavior_string += "{}\n".format(self.solver.solver_helper.get_robot_behaviors_description(behavior_key,behavior_val,"command"))
				if behavior_string not in behavior_options:
					behavior_options.append(behavior_string)
					beh_string_2_dict[behavior_string] = behavior_state.output_dict

		# create robot action list
		rob_actions = StringArray()
		for rob_action in rob_actions_array:
			rob_actions.array.append(rob_action)
		qconflict.robot_action_list = rob_actions

		# create event_env_list
		action_cause = StringArray()
		for cause in cause_array:
			action_cause.array.append(cause)
		qconflict.event_env_list = action_cause

		# create behavior_option_list
		beh_options = StringArray()
		for option in behavior_options:
			beh_options.array.append(option)
		qconflict.behavior_option_list = beh_options

		print(rob_actions)
		print(action_cause)
		print(beh_options)

		# create path
		print("creating paths")
		path = PathComponentArray()
		trimmed_moment_starttime = sample_trace.demo_array[conflicts.index][0].start_moment.start_time
		trimmed_moment_idx = None
		for i in range(len(sample_trace.recording.unprocessed_moment_history)):
			moment = sample_trace.recording.unprocessed_moment_history[i]
			if moment.start_time == trimmed_moment_starttime:
				trimmed_moment_idx = i
		trimmed_moments = sample_trace.recording.unprocessed_moment_history[:trimmed_moment_idx]
		self.record_path_component_array(path,trimmed_moments,sample_trace.recording)
		qconflict.path = path

		message.queries.append(query)
		
		'''
		held data must contain whatever data was passed into this method
		AND a dictionary of responses to behavior dicts
		'''
		held_data = [conflicts,supplementary_data,beh_string_2_dict]
		qm = QueryMode("ConflictQuery",self.mode_stack.pop_mode,None,self.data_hub.publish_query,resolved_callback,message,"tablet",held_data)
		self.mode_stack.add_mode(qm)
		print("now in conflict query mode")

	def conflict_resolution_callback(self, query_answer, supplementary_data):
		answer = query_answer.data_bundle[0].conflict_resolution.behavior_option
		conflicts = supplementary_data[0]
		beh_string_2_dict = supplementary_data[2]
		conflict_idx = conflicts.index
		conflicted_traces = []
		for cluster in conflicts.conflicts:
			for trace in cluster:
				conflicted_traces.append(trace)

		# get the correct beh dict
		beh_dict = beh_string_2_dict[answer]

		# fix the traces
		for trace in conflicted_traces:
			io_pair = trace.demo_array[conflict_idx]
			fixed = (io_pair[1].output_dict != beh_dict)
			io_pair[1].output_dict = beh_dict

			# note that the trace was modified post-scene
			if fixed:
				trace.description = "Modified post-demonstration."

		# done!
		print("conflict resolved!")

	def respond_to_query(self,query_msg):
		'''
		In responding to another node's query, the ctrl node does not need to enter a new mode.

		Preconditions: The destination of the query is ctrl
		'''
		answer_array_msg = DataPassArray()
		answer_array = answer_array_msg.data_bundle

		queries = query_msg.queries
		for query in queries:
			typ = query.type
			if typ == "GetIntentDataQuery":
				print("tablet requested custom speech categories")
				answer_array_msg = DataPassArray()
				answer_array = answer_array_msg.data_bundle

				answer = DataPass()
				answer.destination = "tablet"
				answer.type = "DataPassAllIntentCategories"
				answer_content = DataPassAllIntentCategories()

				# load the user-defined data
				# add a list of user-defined categories
				user_cat_sarray = StringArray()
				for cat in self.speech_db.provided_intents:
					user_cat_sarray.array.append(cat)
				answer_content.edited_intent_categories = user_cat_sarray
				print(user_cat_sarray.array)

				# add the list of phrases from each category
				intent_examples = answer_content.edited_speech_examples
				for cat,provided_intent in self.speech_db.provided_intents.items():
					sys_user_defined_sarray = StringArray()
					for speech in provided_intent.speech:
						sys_user_defined_sarray.array.append(speech)
					intent_examples.append(sys_user_defined_sarray)
					print(sys_user_defined_sarray.array)

				answer.all_intent_cats_data = answer_content
				answer_array.append(answer)

		self.data_hub.publish_answer(answer_array_msg,"tablet")


	def receive_scene_summary_request_callback(self, msg):
		# fill a scene summary array with scene data
		response = SceneSummaryArray()
		trace_components = self.solver.get_trace_components()
		scenes = trace_components.demos
		print("RECEIVED SCENE SUMMARY REQUEST")
		print(len(scenes))
		for scene in scenes:
			scene_summary = SceneSummary()
			scene_summary.scene_number = scene.scene_id
			print(scene.scene_id)
			scene_summary.title = scene.name
			scene_summary.duration_in_seconds = scene.duration
			scene_summary.summary = scene.description
			response.array.append(scene_summary)
		self.scene_summary_publisher.publish(response)

	def receive_scene_info_request_callback(self, msg):
		# currently uses dummy data
		# TODO: fill in with actual data, not dummy data
		response = PathComponentArray()
		scene_id = msg.data
		trace_components = self.solver.get_trace_components()
		scenes = trace_components.demos
		scene_of_interest = None
		for scene in scenes:
			if scene.scene_id == scene_id:
				scene_of_interest = scene
				break
		moments = scene_of_interest.recording.unprocessed_moment_history
		self.record_path_component_array(response,moments,scene_of_interest.recording)

		self.scene_info_publisher.publish(response)

	def record_path_component_array(self,response,moments,recording,criteria=None):
		print("recording path")
		print(self.pixelx)
		print(moments[0].x)
		for moment in moments:

			# assemble the robot array
			# the moment coordinates will be Nonetype if the projector was not used
			# this is a special case that requires handling
			robot_data = PathComponent()
			if moment.x is not None and moment.y is not None:
				robot_data.x = moment.x*1.0/self.pixelx
				robot_data.y = moment.y*1.0/self.pixely
			else:
				robot_data.x = 0
				robot_data.y = 0
			robot_data.agent = "robot"
			if moment.start_time in recording.time2moment:
				processed_moment = recording.time2moment[moment.start_time]
				if criteria is None or criteria == moment:
					robot_data.description = processed_moment.description["r"]
			response.array.append(robot_data)

			# assemble the human array
			for h,tup in moment.human_coords.items():
				human_data = PathComponent()
				if tup[0] is not None and tup[1] is not None:
					human_data.x = tup[0]*1.0/self.pixelx
					human_data.y = tup[1]*1.0/self.pixely
				else:
					human_data.x = 0
					human_data.y = 0
				human_data.agent = "h1"
				if moment.start_time in recording.time2moment:
					processed_moment = recording.time2moment[moment.start_time]
					if criteria is None or criteria == moment:
						human_data.description = processed_moment.description["h1"]
				response.array.append(human_data)

	def delete_scene_callback(self, msg):
		scene_id = int(msg.data)
		print("deleting scene {}".format(scene_id))

		to_delete = None
		for trace in self.solver.trace_components.demos:
			if trace.scene_id == scene_id:
				to_delete = trace

		if to_delete is None:
			print("ERROR: could not find scene to delete")
			exit()

		self.solver.trace_components.remove_trace(to_delete)

		self.receive_scene_summary_request_callback(None)

	def throwaway_speech_callback(self, msg):

		# process message
		pass

		# publish
		self.throwaway_speech_pub.publish(msg)

	def flag_speech_callback(self, msg):

		# process message
		pass

		# publish
		self.flag_speech_pub.publish(msg)

	def publish_begin_deploy(self):
		'''
		This function has a dual purpose:
			(1) package up the solution (program) into a ROS message
			(2) send the interaction to the robot
		'''

		# package up the interaction program (ts = 'transition system')
		print("sending deployment files to TemiTracery")
		self.deployer.deploy()

	def display_content_sub_callback(self, msg):
		self.audio_display_publisher.publish(msg)

	def highlight_agent_callback(self,msg):
		self.highlight_agent_pub.publish(msg)

	def request_pause_callback(self,msg):
		self.recorder.curr_recording.pause()
		amsg = String()
		amsg.data = "pause"
		self.pause_pub.publish(amsg)
		print("pausing")

		# discard the previous moment if the position has not been set
		negative_moments = []
		for moment in self.recorder.curr_recording.moment_history:
			if moment.x == -1 or moment.y == -1:
				print("negative moment")
				negative_moments.append(moment)
		for moment in negative_moments:
			self.recorder.curr_recording.moment_history.remove(moment)

		# send back a message containing a path component array
		response = PathComponentArray()
		for moment in self.recorder.curr_recording.moment_history:

			# assemble the robot array
			# the moment coordinates will be Nonetype if the projector was not used
			# this is a special case that requires handling
			robot_data = PathComponent()
			if moment.x is not None and moment.y is not None:
				robot_data.x = moment.x*1.0/self.pixelx
				robot_data.y = moment.y*1.0/self.pixely
			else:
				robot_data.x = 0
				robot_data.y = 0
			robot_data.agent = "robot"
			response.array.append(robot_data)

			# assemble the human array
			for h,tup in moment.human_coords.items():
				human_data = PathComponent()
				if tup[0] is not None and tup[1] is not None:
					human_data.x = tup[0]*1.0/self.pixelx
					human_data.y = tup[1]*1.0/self.pixely
				else:
					human_data.x = 0
					human_data.y = 0
				human_data.agent = "h1"
				response.array.append(human_data)

		self.send_pause_path_pub.publish(response)

	def request_resume_callback(self,msg):
		print("unpausing")
		rewind = msg.data
		if rewind != -1:
			print("rewinding from moment idx {} to {}".format(len(self.recorder.curr_recording.moment_history),rewind))

			# get the start time of the rewind point, discard all audio past that start time
			rewind_time = self.recorder.curr_recording.moment_history[rewind].clock_time
			rmsg = Float64()
			rmsg.data = rewind_time
			self.rewind_pub.publish(rmsg)

			self.recorder.curr_recording.moment_history = self.recorder.curr_recording.moment_history[0:rewind+1]
		# unpause the audio
		amsg = String()
		amsg.data = "resume"
		self.pause_pub.publish(amsg)
		# unpause the recording
		self.recorder.curr_recording.unpause()

	def pixel_size_subsc_callback(self,msg):
		self.pixelx = msg.x
		self.pixely = msg.y

		string = String()
		string.data = "ctrl"

		self.pixel_size_ack.publish(string)

	def publish_end_deploy(self):
		'''
		TODO: implement
		**Make sure the tablet gets notified that deploy has ended.
		'''
		pass
