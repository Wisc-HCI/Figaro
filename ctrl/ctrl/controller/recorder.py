import time
import threading
import math
import copy

from controller.record_visualizer import *
from synthesis.axiom import *
from synthesis.trace_utils import *

class Recorder:

	def __init__(self, input_modalities, design, objects=[], regions=[]):
		self.curr_recording = None
		self.input_modality_info = input_modalities
		self.input_modalities = []
		for item in input_modalities:
			self.input_modalities.append(item)

		# set up the ability to visualize a recording on a timeline
		# on the timeline, each input modality has a horizontal track
		self.record_visualizer = RecordVisualizer(self.input_modality_info)

		# setup the trace utils class to be shared with all recordings
		self.axioms = Axioms(self.input_modality_info, design)
		self.trace_utils = TraceUtils(self.axioms, design, objects, regions)

	def initialize_objects_regions(self,objects,regions):
		self.trace_utils.objects = objects
		self.trace_utils.regions = regions

		self.trace_utils.trace_utils.objects = objects
		self.trace_utils.trace_utils.regions = regions

	def begin_scene(self):
		self.curr_recording = Recording(self.input_modality_info, self.trace_utils, granularity=0.1)
		self.curr_recording.roll()

	def record_current_moment_position(self,ident,x,y,theta):
		self.curr_recording.record_current_moment_position(ident,x,y,theta)

	def add_to_current_moment(self,modality,value):
		self.curr_recording.add_to_current_moment(modality,value)

	def add_to_previous_moments(self, modality, value, starttime, endtime):
		self.curr_recording.add_to_previous_moments(modality, value, starttime, endtime)

	def add_to_previous_moment(self, modality, value, timestamp):
		if self.curr_recording is not None:
			print("adding...")
			self.curr_recording.add_to_previous_moment(modality,value,timestamp)

	def pause_scene(self):
		self.curr_recording.pause()

	def resume_scene(self):
		self.curr_recording.unpause()

	def find_closest_moment(self,x,y,ff):
		return self.curr_recording.find_closest_moment(x,y,self.input_modality_info,ff)

	def rewind(self, idx):
		self.curr_recording.rewind(idx)

	def fastforward(self, idx):
		self.curr_recording.fastforward(idx)

	def check_if_can_fastforward(self):
		return self.curr_recording.check_if_can_fastforward()

	def end_scene(self):
		self.curr_recording.cut()

	# visualize the current recording on a timeline and save to PDF
	def visualize_current_recording(self, name="vis", recording=None):
		if recording is None:
			if self.curr_recording is not None:
				self.record_visualizer.visualize(self.curr_recording,name=name)
		else:
			self.record_visualizer.visualize(recording,name=name)

	def post_process(self):
		self.curr_recording.post_process(self.input_modality_info)

	def get_moments(self):
		return self.curr_recording.moment_history

	def get_recording_duration(self):
		if self.curr_recording is not None:
			return int(round(self.curr_recording.end_time - self.curr_recording.start_time))
		return -1

	def print_moments(self):
		if self.curr_recording is not None:
			self.curr_recording.print_moments()

class Recording:

	def __init__(self, input_modality_info, trace_utils, granularity=0.05):
		self.log = []
		self.start_time = -1
		self.end_time = -1
		self.input_modality_info = input_modality_info
		self.input_modalities = list(input_modality_info.keys())
		self.curr_moment = Moment(self.input_modalities)
		self.moment_history = []
		self.unprocessed_moment_history = None
		self.time2moment = {}
		self.fast_forwardable_history = []
		self.granularity = granularity
		self.rolling = False
		self.processing = False
		self.paused = False

		self.trace_utils = trace_utils

		# pausing
		self.start_pause_time = -1
		self.accumulated_pause_time = 0.0

		# set up the ability to visualize a recording on a timeline
		# on the timeline, each input modality has a horizontal track
		self.record_visualizer = RecordVisualizer(self.input_modality_info)

	def record_current_moment_position(self,ident,x,y,theta):
		if self.paused:
			return
		self.curr_moment.update_position(ident,x,y,theta)

	def add_to_current_moment(self, modality, value):
		if self.paused:
			return
		self.curr_moment.add_to_track(modality,value)

	def add_to_previous_moments(self, modality, value, starttime, endtime):
		if self.paused:
			return
		for i in range(0,len(self.moment_history)-1):
			moment = self.moment_history[i]
			next_moment = self.moment_history[i+1]
			if (moment.clock_time - self.start_time) > starttime and (moment.clock_time - self.start_time) < endtime:
				moment.add_to_track(modality,value)

		if (self.moment_history[-1].clock_time - self.start_time) > starttime and (self.moment_history[-1].clock_time - self.start_time) < endtime:
			self.moment_history[-1].add_to_track(modality,value)

	def add_to_previous_moment(self, modality, value, timestamp):
		if self.paused:
			return
		for i in range(0,len(self.moment_history)-1):
			moment = self.moment_history[i]
			next_moment = self.moment_history[i+1]
			if moment.clock_time <= timestamp and next_moment.clock_time > timestamp:
				moment.add_to_track(modality,value)

	def check_if_can_fastforward(self):
		return len(self.fast_forwardable_history) > 0

	def roll(self):
		self.rolling = True
		self.start_time = time.time()
		self.curr_moment.set_start_time(self.start_time)
		thread = threading.Thread(target=self.run_clock, args=())
		thread.daemon = True
		thread.start()

	def pause(self):
		self.paused = True
		self.start_pause_time = time.time()

	def unpause(self):
		self.paused = False
		self.accumulated_pause_time += (time.time() - self.start_pause_time)


	def rewind(self,idx):
		print("rewinding")

		# store the rewound history
		for i in range(idx,len(self.moment_history)):
			self.fast_forwardable_history.append(self.moment_history[i])

		# delete the rewound history
		i = len(self.moment_history) - 1
		while i >= idx:
			del self.moment_history[i]
			i -= 1
		print(self.moment_history)

	def fastforward(self,idx):
		print("fastforwarding")

		# append and delete the fastforwardable history up to the point of idx
		i = 0
		while i <= idx:
			self.moment_history.append(self.fast_forwardable_history[0])
			del self.fast_forwardable_history[0]
			i += 1

	def find_closest_moment(self,x,y,input_modality_info,ff):
		if ff:
			processed_moments = self.trace_utils.post_process_stream(self.fast_forwardable_history, input_modality_info)
		else:
			processed_moments = self.trace_utils.post_process_stream(self.moment_history, input_modality_info)
		closest_moment = processed_moments[0]
		closest_moment_idx = 0
		closest_distance = closest_moment.get_distance_from(x,y)
		for i in range(len(processed_moments)):
			moment = processed_moments[i]
			distance = moment.get_distance_from(x,y)
			if distance < closest_distance:
				closest_distance = distance
				closest_moment = moment
				closest_moment_idx = i

		print("closest moment: {} ({})".format(closest_moment,closest_moment_idx))
		return closest_moment,closest_moment_idx

	def cut(self):
		self.processing = True
		self.rolling = False

		# wait for the thread to finish
		while self.processing:
			time.sleep(0.1)

	def post_process(self, input_modality_info):
		self.unprocessed_moment_history = copy.deepcopy(self.moment_history)
		self.trace_utils.post_process_stream(self.moment_history, input_modality_info)

		for moment in self.moment_history:
			self.time2moment[moment.start_time] = moment

	def standardize_saved_recording(self,input_modality_info,trace_utils):
		self.input_modality_info = input_modality_info
		self.input_modalities = list(input_modality_info.keys())
		self.trace_utils = trace_utils
		print("")
		print(self.trace_utils.axioms.modalities)
		self.moment_history = self.unprocessed_moment_history
		self.unprocessed_moment_history = None
		self.time2moment = {}

	def run_clock(self):
		
		while self.rolling:
			if self.paused:
				continue
			time.sleep(self.granularity)
			self.moment_history.append(self.curr_moment)
			self.curr_moment = Moment(self.input_modalities)
			clock_time = time.time()
			self.curr_moment.set_start_time(clock_time - self.accumulated_pause_time)
			self.curr_moment.set_clock_time(clock_time)

		self.end_time = time.time()
		self.processing = False

	def print_moments(self):
		for moment in self.moment_history:
			print("start: {}     position: {}, movement: {}".format(moment.start_time,moment.tracks["position"], moment.tracks["movement"]))

class Moment:

	def __init__(self, input_modalities):
		self.tracks = {}
		for inp in input_modalities:
			self.tracks[inp] = None
		self.start_time = -1
		self.clock_time = -1
		self.x = -1
		self.y = -1
		self.theta = 90
		self.human_coords = {}

		# the description can be used for moments that see a significant change in events
		self.description = {"r":"","h1":""}

	def update_position(self,ident,x,y,theta):
		if ident == "r":
			self.x = x
			self.y = y
			self.theta = theta
		else:
			self.human_coords[ident] = (x,y)

	def add_to_track(self, track, val):
		self.tracks[track] = val

	def set_start_time(self, time):
		self.start_time = time

	def set_clock_time(self, time):
		self.clock_time = time

	def get_distance_from(self,x,y):
		return math.sqrt(float(x-self.x)**2 + float(y-self.y)**2)

	def __str__(self):
		return self.description["r"]