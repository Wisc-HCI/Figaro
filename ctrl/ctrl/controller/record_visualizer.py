import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from matplotlib.collections import PolyCollection

class RecordVisualizer:
	'''
	Visualizes recordings on a timeline
	Each input modality is a horizontal track on the timeline
	'''

	def __init__(self,input_mod_handler):
		self.modalities = input_mod_handler
		plt.rcParams.update({'font.size': 6})

		# get a list of the input modality names
		self.modality_list = list(self.modalities.keys())

		# map of modalities (categories) to integers representing position on y axis
		self.cats = {}
		# get a list of colors to distinguish different tracks on the visualization
		self.color_dict = {}
		int_counter = len(self.modality_list)
		color_counter = 0
		for mod in self.modality_list:
			self.cats[mod] = int_counter
			self.color_dict[mod] = "C{}".format(color_counter)
			int_counter -= 1
			color_counter += 1

		self.label_height_idx = 0
		self.label_height_options = [-0.4,-0.2,0.0,0.2]

	def visualize(self, recording, name="vis"):
		'''
		Creates and exports a PDF visualization of the recording
		'''
		moment_history = recording.moment_history

		# check which modalities have nothing recorded
		has_info_flag = {}  # flags for whether the moments for each modality contain useful
							# (not-none) information recorded
		for modality in self.modality_list:
			has_info_flag[modality] = False  # init to false

		# determine whether information information exists within each modality
		for moment in moment_history:
			for modality in has_info_flag:
				if moment.tracks[modality] is not None and moment.tracks[modality] is not "False":
					has_info_flag[modality] = True

		# shorten list of modalities to those that have information
		updated_modality_list = []
		updated_cats = {}
		updated_counter = 1
		for modality in self.modality_list:
			if has_info_flag[modality]:
				updated_modality_list.append(modality)
				updated_cats[modality] = updated_counter
				updated_counter += 1

		colors = []
		labels = []

		# create the vertices
		# 1) initialize dict that contains current state of each modality at a given moment
		verts = []
		modality_curr_state = {}
		for modality in updated_modality_list:
			modality_curr_state[modality] = None

		# 1.5) (important) the time (x-axis) should start from 0
		overall_start_time = moment_history[0].start_time

		# 2) iterate from moment-to-moment keeping track of what state each modality is in
		for moment in moment_history:
			tracks = moment.tracks

			for modality in updated_modality_list:
				val = tracks[modality]

				# case 1: there is information where there previously wasn't
				if self.is_val_visible(modality,val) and not self.is_val_visible(modality,modality_curr_state[modality]):
					self.init_new_clip(moment.start_time - overall_start_time, val, modality_curr_state, modality)

				# case 2: there is no information there where there previously was
				elif not self.is_val_visible(modality,val) and self.is_val_visible(modality,modality_curr_state[modality]):
					self.finish_clip(modality_curr_state[modality], moment.start_time - overall_start_time, verts, updated_cats, modality, modality_curr_state, colors, labels)
				# case 3: there is information there where there previously was different information
				elif self.is_val_visible(modality,val) and self.is_val_visible(modality,modality_curr_state[modality]) and val != modality_curr_state[modality]:
					self.finish_clip(modality_curr_state[modality], moment.start_time - overall_start_time, verts, updated_cats, modality, modality_curr_state, colors, labels)
					self.init_new_clip(moment.start_time - overall_start_time, val, modality_curr_state, modality)
				# case 4: there is no information where there previously wasn't
				#else:
				#	pass
			self.label_height_idx = (self.label_height_idx+1)%4	

		# wrap up final clip
		for modality in updated_modality_list:
			val = tracks[modality]

			if modality_curr_state[modality] is not None:
				self.finish_clip(modality_curr_state[modality], recording.end_time - overall_start_time, verts, updated_cats, modality, modality_curr_state, colors, labels)

		# post-process the labels so that there aren't duplicate labels back-to-back
		curr_val = {}
		for mod,val in updated_cats.items():
			curr_val[val-0.4] = None
		for label in labels:
			if label.text == curr_val[label.track]:
				label.text = ""
			else:
				curr_val[label.track] = label.text

		# Make the plot
		bars = PolyCollection(verts, facecolors=colors)

		fig, ax = plt.subplots()
		ax.add_collection(bars)
		ax.autoscale()

		ax.set_yticks([i for i in range(1,len(updated_modality_list)+1)])
		#updated_modality_list.reverse()
		ax.set_yticklabels(updated_modality_list)

		# draw labels
		for label in labels:
			plt.text(label.x,label.y,label.text)

		plt.tight_layout()
		plt.savefig('{}.pdf'.format(name))
		plt.close(fig)

	def init_new_clip(self, time, val, modality_curr_state, modality):
		new_clip = Clip(time,val)
		modality_curr_state[modality] = new_clip

	def finish_clip(self, clip, end_time, verts, updated_cats, modality, modality_curr_state, colors, labels):

		# first choose the label height
		lheight = self.label_height_options[self.label_height_idx]

		clip.end_time = end_time
		verts.append([(clip.start_time, updated_cats[modality]-0.4), 
					  (clip.start_time, updated_cats[modality]+0.4), 
					  (clip.end_time, updated_cats[modality]+0.4), 
					  (clip.end_time, updated_cats[modality]-0.4), 
					  (clip.start_time, updated_cats[modality]-0.4)])
		modality_curr_state[modality] = None

		# add a color for the vertex
		colors.append(self.color_dict[modality])

		# determine if a label is required
		if self.modalities[modality]["type"] == "boolean" and clip.val == "False":
			pass
		elif clip.val == None:
			pass
		else:
			labels.append(Label(str(clip.val), clip.start_time, updated_cats[modality] + lheight, updated_cats[modality] - 0.4))

	def is_val_visible(self, modality, val):
		if val is None:
			return False
		elif self.modalities[modality]["type"] == "boolean" and val == "False":
			return False
		return True

class Clip:
	'''
	A clip is an event that occurs on the timeline
	E.g. the robot speaks within a certain time interval
	'''

	def __init__(self, start_time, val):
		self.start_time = start_time
		self.end_time = None
		self.val = val

class Label:
	'''
	Helper class to store locations of labels on the visualization
	'''

	def __init__(self, text, x, y, track):
		self.text = text
		self.x = x
		self.y = y
		self.track = track