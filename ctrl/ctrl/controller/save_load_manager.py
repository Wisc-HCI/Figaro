from figaro_msgs.msg import PhysicalLayout
from std_msgs.msg import String

import os
import shutil
import pickle
import sys
import traceback

class FileManager:

	def __init__(self, controller, curr_param_folder):
		self.controller = controller
		self.curr_param_folder = curr_param_folder

		# saving
		self.save_subscriber = self.controller.create_subscription(String,'tablet_pub_ctrl_save', self.save_callback, 10)
		self.layout_request_publisher = self.controller.create_publisher(String, 'ctrl_layout_request_pub', 10)
		self.layout_request_receiver = self.controller.create_subscription(PhysicalLayout, 'layout_pub_ctrl', self.serialize_layout_msg, 10)

		# loading
		self.load_subscriber = self.controller.create_subscription(String,'tablet_pub_ctrl_load', self.load_callback, 10)
		self.load_display_publisher = self.controller.create_publisher(String, 'ctrl_pub_tablet_load', 10)
		self.load_layout_publisher = self.controller.create_publisher(PhysicalLayout, 'ctrl_layout_pub', 10)

	def serialize_layout_msg(self,layout_msg):
		# save the grid
		print(layout_msg.object_data)
		with open("{}/layout.pkl".format(layout_msg.name),"wb") as outfile:
			pickle.dump(layout_msg,outfile)

	def save_callback(self, msg):
		'''
		Save the following:
		1) in_modes.json
		2) lists of recorded moments
		3) grid
		4) detailed physical layout json (stored in same pickle as above)
		5) list of objects and regions
		'''
		print("saving session")

		# make the save directory
		filepath = "session_parameters/{}/{}".format(sys.argv[1],msg.data)
		if not os.path.isdir(filepath):
			os.mkdir(filepath)

		# save the list of recorded moments
		demo_array = self.controller.solver.trace_components.demos
		with open("{}/traces.pkl".format(filepath),"wb") as outfile:
			pickle.dump(demo_array,outfile)

		# save the set of provided intents
		provided_intents = self.controller.speech_db.provided_intents
		with open("{}/intents.pkl".format(filepath),"wb") as outfile:
			pickle.dump(provided_intents,outfile)

		# save the objects and regions
		objects = self.controller.recorder.trace_utils.objects
		regions = self.controller.recorder.trace_utils.regions
		dic = {"objects":objects,"regions":regions}
		with open("{}/objects_regions.pkl".format(filepath),"wb") as outfile:
			pickle.dump(dic,outfile)

		# get the grid and detailed json layout
		request = String()
		request.data = filepath
		self.layout_request_publisher.publish(request)

	def load_callback(self, msg):
		'''
		Load the following:
		1) in_modes.json
		2) lists of recorded moments
		3) grid
		4) detailed physical layout json (stored in same pickle as above)
		'''
		print("loading session")

		filepath = "session_parameters/{}/{}".format(sys.argv[1],msg.data)
		if not os.path.isdir(filepath):
			print("WARNING: filepath to load a session does not exist.\nReturning without loading.")
			return

		# load the objects and regions
		#try:
		infile = open("{}/objects_regions.pkl".format(filepath),"rb")
		obj_reg = pickle.load(infile)
		objects = obj_reg["objects"]
		regions = obj_reg["regions"]
		print(objects)
		print(regions)
		self.controller.recorder.initialize_objects_regions(objects,regions)
		#except:
		#	print("WARNING: could not initialize recorder's objects or region data")
		#	exit()

		# load the list of recorded moments
		#try:
		infile = open("{}/traces.pkl".format(filepath),"rb")
		demos = pickle.load(infile)
		#self.controller.solver.trace_components.demos = demos

		print("loaded the following demos")
		for demo in demos:
			print(demo.pretty_string())

		# re-synthesize everything
		# clear the trace history
		self.controller.solver.trace_components.demos = []

		# collect the unprocessed moments
		recordings = []
		for demo in demos:
			recordings.append(demo.recording)
			demo.recording.trace_utils.trace_utils.objects = demo.recording.trace_utils.objects
			demo.recording.trace_utils.trace_utils.regions = demo.recording.trace_utils.regions

		# go through each modality, check if it exists
		modalities = self.controller.modalities
		absent_modalities = []
		for modality in modalities:
			if len(demos) == 0:
				break
			if modality not in demos[0].recording.unprocessed_moment_history[0].tracks:
				absent_modalities.append(modality)
		if len(absent_modalities) > 0:
			print(absent_modalities)
			for rec in recordings:
				for moment in rec.unprocessed_moment_history:
					for mod in absent_modalities:
						moment.tracks[mod] = None

		unprocessed_recordings,conflicts = self.post_process_all_recordings(recordings)
		self.attempt_to_synthesize(conflicts,unprocessed_recordings)
		#except Exception as e:
		#	print("WARNING: could not load any saved traces")
		#	print(e)

		# load the layout
		try:
			infile = open("{}/layout.pkl".format(filepath),"rb")
			layout_msg = pickle.load(infile)

			# process objects and regions
			objects = []
			regions = []
			for item_loc in layout_msg.object_data:
				objects.append(item_loc.name)
			for region_loc in layout_msg.region_data:
				regions.append(region_loc.name)
			self.controller.recorder.initialize_objects_regions(objects,regions)

			# send the grid to the projector
			self.load_layout_publisher.publish(layout_msg)

			# send the detailed layout to the tablet
			data = String()
			data.data = layout_msg.detailed_json
			print("publishing layout...")
			self.load_display_publisher.publish(data)
		except Exception as e:
			print("WARNING: could not load layout message file")
			print(e)

		# load the provided intents
		try:
			infile = open("{}/intents.pkl".format(filepath),"rb")
			provided_intents = pickle.load(infile)
			self.controller.speech_db.provided_intents = provided_intents
		except Exception as e:
			print("WARNING: could not load provided intents")
			print(e)

	def post_process_all_recordings(self, recordings):
		# process each batch of moments
		# make the traces from each processed batch
		conflicts = None
		unprocessed_recordings = [r for r in recordings]
		print("\n~~~~~~~~~~~~")
		print(self.controller.modalities)
		print(self.controller.recorder.trace_utils.axioms.modalities)
		for recording in recordings:
			recording.standardize_saved_recording(self.controller.modalities,self.controller.recorder.trace_utils)
			print(self.controller.recorder.trace_utils.axioms.modalities)
			# visualize the recordings on a timeline
			self.controller.recorder.visualize_current_recording(name="before_{}".format(len(self.controller.solver.trace_components.demos)),recording=recording)

			# create the sequence and then the trace
			print(self.controller.recorder.trace_utils.axioms.modalities)
			recording.post_process(self.controller.modalities)
			unprocessed_recordings.remove(recording)
			conflicts = self.controller.solver.make_trace(recording)
			# visualize the recordings on a timeline
			self.controller.recorder.visualize_current_recording(name="after_{}".format(len(self.controller.solver.trace_components.demos)),recording=recording)
			if conflicts is not None:
				#conflicts = None
				break

		return unprocessed_recordings,conflicts

	def attempt_to_synthesize(self,conflicts,unprocessed_recordings):
		# synthesize
		if conflicts is None:
			self.controller.synthesize_scenes()
		else:
			print("could not synthesize interaction from loaded traces as 1 conflict found")
			self.controller.initiate_conflict_resolution(conflicts,self.conflict_resolution_callback,unprocessed_recordings)

	def conflict_resolution_callback(self,query_answer,supplementary_data):
		# call the conflict resolution handler in controller
		self.controller.conflict_resolution_callback(query_answer, supplementary_data)

		# handle the remaining recordings
		recordings = supplementary_data[1]
		unprocessed_recordings,conflicts = self.post_process_all_recordings(recordings)
		self.attempt_to_synthesize(conflicts,unprocessed_recordings)
