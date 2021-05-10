from copy import *
import random
from random import shuffle
import math
import importlib

class TraceComponents():

	def __init__(self):

		self.demos = []
		self.procedures = {}
		self.demo2procedure = {}
		self.demo_number = 1
		self.design = "test"
		self.curr_demo_id = 0

	def add_trace(self,trace,event_beh_classifier):
		self.demos.append(trace)

		# get the starting event
		event = None
		first_action = trace.demo_array[1][0]

		for key in first_action.input_dict:
			if event_beh_classifier(key) == "event":
				event = key
				break

		if event is None:
			print("ERROR: there is no event to trigger the start of a trace")
			exit(1)

		event_val = first_action.input_dict[event]

		if event not in self.procedures:
			self.procedures[event] = {}
		if event_val not in self.procedures[event]:
			self.procedures[event][event_val] = []

		self.procedures[event][event_val].append(trace)
		self.demo2procedure[trace] = (event,event_val)

	def remove_trace(self,trace):
		tup = self.demo2procedure[trace]
		event = tup[0]
		event_val = tup[1]
		del self.demo2procedure[trace]
		self.demos.remove(trace)
		self.procedures[event][event_val].remove(trace)
		if len(self.procedures[event][event_val]) == 0:
			del self.procedures[event][event_val]
		if len(self.procedures[event]) == 0:
			del self.procedures[event]

	def get_and_increment_demo_id(self):
		val = self.curr_demo_id
		self.curr_demo_id += 1
		return val

	def exists_in_demos(self, inp, out, demos=None):
		if demos is None:
			demos = self.demos

		return_val = False
		for demo_object in demos:
			demo = demo_object.demo_array
			for pair in demo:
				if inp == pair[0].inp and out == pair[1].output:
					return_val = True

		return return_val

class Demo():

	def __init__(self, demo_array, recording, name, scene_id, callback=None):
		self.recording = recording
		self.demo_array = demo_array

		self.delete_callback=callback
		self.should_be_deleted=False

		# parameters for the string
		self.id = -1
		self.name = name if name is not None else "Example"
		self.scene_id = scene_id
		self.satisfied = 1

		# activated
		self.activated = True

		# metadata
		self.duration = -1
		self.description = ""

	def setActivated(self, val):
		self.activated = val
		if val == False:
			self.satisfied = 0

	def markForDeletion(self):
		self.should_be_deleted=True
		if self.delete_callback is not None:
			self.delete_callback(self)

	def is_same(self, demo):
		if len(self.demo_array) != len(demo.demo_array):
			return False
		else:
			is_same = True
			for i in range(len(self)):
				if self.demo_array[i][0].inp != demo.demo_array[i][0].inp:
					is_same = False
					break
				elif self.demo_array[i][1].output != demo.demo_array[i][1].output:
					is_same = False
					break
				elif self.demo_array[i][1].gesture != demo.demo_array[i][1].gesture:
					is_same = False
					break
				#elif self.demo_array[i][1].gesture != demo.demo_array[i][1].gesture:
				#	is_same = False
				#	break
			return is_same

	def pretty_string(self):
		string = "START >>>    "
		for item in self.demo_array:
			string += "{} -> {}    ".format(item[0].input_dict, item[1].output_dict)
		return string

	def __len__(self):
		return len(self.demo_array)

	def __str__(self):
		args = ("{}".format(self.name), "Length: {}".format(len(self)), "Sat.: {}".format(int(round(self.satisfied*100))))
		string = "{0:<0} {1:>12} {2:>12}%".format(*args)
		return string

class Output():

	def __init__(self, output, start_moment):

		self.output = output
		self.output_dict = {}
		self.start_moment = start_moment
		self.relaxed_output = output

	def __str__(self):
		return self.output

class Input():
	def __init__(self, inp, start_moment):
		self.inp = inp
		self.input_dict = {}
		self.start_moment = start_moment

	def __str__(self):
		return self.inp
