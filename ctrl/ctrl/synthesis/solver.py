from synthesis.trace_components import *
from synthesis.glue_checker import *
from synthesis.glue_exporter import *

import json
import os
import sys
import time

from copy import deepcopy,copy
import random

class Solver:

	def __init__(self, input_modalities, design, speech_categories):
		self.trace_components = TraceComponents()
		self.modalities = input_modalities

		# glue
		self.glue_checker = GlueChecker()
		self.exporter = GlueExporter()

		# speech categories for exporting
		self.speech_db = speech_categories

		# access the solver helper
		# append the extra axioms
		sys.path.append("session_parameters/{}".format(design))
		from session_trace_processor import TraceProcessor
		self.trace_processor = TraceProcessor()
		from session_solver_helper import SolverHelper
		self.solver_helper = SolverHelper()
		from session_glue_generalizer import GlueGeneralizer
		self.glue_generalizer = GlueGeneralizer()

		self.flagflag = False

	def make_trace(self, curr_recording):
		'''
		First clean the sequence by removing unused modalities
		'''
		orig_moments = curr_recording.moment_history
		robot = []
		environment = []

		# create a copy of sequence
		moments = deepcopy(orig_moments)

		# create a 1:1 mapping between the copy and the original
		moment_copy2orig = {}
		for i in range(len(moments)):
			moment_copy2orig[moments[i]] = orig_moments[i]

		# determine which tracks to remove
		to_remove = {}
		for modality in self.modalities:
			to_remove[modality] = True

		for moment in moments:
			for item,val in moment.tracks.items():
				if self.modalities[item]["type"] == "boolean" and val is not None:
					to_remove[item] = False
				elif item == "speech" and val != "unknown" and val is not None:
					to_remove[item] = False
				elif val is not None:
					to_remove[item] = False

		# remove the tracks
		for moment in moments:
			for mod in to_remove:
				if to_remove[mod]:
					del moment.tracks[mod]

		'''
		Assemble a preliminary trace
		'''
		mod_list = [mod for mod in to_remove if not to_remove[mod]] # to maintain ordering
		robot_mod_list = []
		human_mod_list = []
		cause_mod_list = []
		env_mod_list = []
		robot_env_mod_list = []
		human_env_mod_list = []
		other_env_mod_list = []

		# this part is currently hardcoded
		self.solver_helper.fill_cause_effect_env_lists(mod_list,robot_mod_list,human_mod_list,cause_mod_list,env_mod_list,robot_env_mod_list,human_env_mod_list,other_env_mod_list)
		print("ROBOT MODS: {}".format(robot_mod_list))
		'''
		Process is now as follows:
			1 create "primary splits" based on event triggers that occur (e.g. create state segments with start and end times)
			2 find start points of robot behaviors (only the positive behaviors)
			3 create "secondary splits" based on a change within a single modality that occurs for seemingly no reason
				OR based on a change within different modalities that happens sequentially
			4 bin robot behaviors based on which state segment they fall under
		'''

		primary_splits = []
		start_points = {}
		for mod in robot_mod_list:
			start_points[mod] = []
		secondary_splits = [moments[0].start_time,curr_recording.end_time]

		def add_primary_splits(mod,moment,curr_state,primary_splits):
			if moment.tracks[mod] != curr_state[mod]:
				curr_state[mod] = moment.tracks[mod]
				if moment.tracks[mod] is not None and moment.tracks[mod] != "False":
					if moment.start_time not in primary_splits:
						primary_splits.append(moment.start_time)

		# 1
		curr_state = {}
		for mod in moments[0].tracks:
			if mod not in robot_mod_list:
				curr_state[mod] = None
		for moment in moments:
			for mod in cause_mod_list:
				add_primary_splits(mod,moment,curr_state,primary_splits)
		# debug
		
		print("primary splits")
		for split in primary_splits:
			print(split)

		# 2
		curr_state = {}
		for mod in robot_mod_list:
			curr_state[mod] = None
		for moment in moments:
			for mod in robot_mod_list:
				if moment.tracks[mod] != curr_state[mod]:
					curr_state[mod] = moment.tracks[mod]
					if moment.tracks[mod] is not None and moment.tracks[mod] != "False":
						start_points[mod].append(moment.start_time)

		# TODO: get rid of if this causes problems
		
		for mod in robot_mod_list:
			if moments[0].start_time not in start_points[mod]:
				start_points[mod].append(moments[0].start_time)

		# 3
		splits_so_far = primary_splits.copy()
		splits_so_far.extend(secondary_splits)
		splits_so_far.sort()
		# get changes within single behavioral modality
		for mod in start_points:
			for i in range(len(start_points[mod])-1):
				for j in range(len(splits_so_far)-1):

					if start_points[mod][i] >= splits_so_far[j] and start_points[mod][i+1] < splits_so_far[j+1]:
						secondary_splits.append(start_points[mod][i+1])

		# get sequential changes (a new behavioral modality begins that does not occur during concurrent execution of another modality)
		for i in range(1,len(moments)):
			moment = moments[i]
			prev_moment = moments[i-1]
			if moment.start_time not in secondary_splits and moment.start_time not in primary_splits:
				active_mods = []
				for mod in robot_mod_list:
					if moment.tracks[mod] is not None and moment.tracks[mod] != "False":
						if prev_moment.tracks[mod] is None or prev_moment.tracks[mod] == "False":
							active_mods.append(mod)
				if len(active_mods) == 1:
					pass
					secondary_splits.append(moment.start_time)
		
		print("secondary splits")
		for split in secondary_splits:
			print(split)

		# 4 
		all_splits = primary_splits.copy()
		all_splits.extend(secondary_splits)
		all_splits = sorted(all_splits)

		bins = []

		for i in range(len(all_splits)-1):
			bn = {"open": all_splits[i], "moments": []}
			for moment in moments:
				#if moment.start_time > all_splits[i+1]:
				#	break
				contains_start_mod = False
				for mod in robot_mod_list:
					if moment.start_time in start_points[mod]:
						contains_start_mod = True
				#if not contains_start_mod:
				#	pass
				if moment.start_time >= all_splits[i] and moment.start_time < all_splits[i+1]:
					bn["moments"].append(moment)
			bins.append(bn)

		print(bins)

		print("start points")
		for mod in start_points:
			for sp in start_points[mod]:
				print(sp - moments[0].start_time)

		print("primary splits")
		for split in primary_splits:
			print(split - moments[0].start_time)

		print("secondary splits")
		for split in secondary_splits:
			print(split - moments[0].start_time)
		
		print("bins:")
		for bn in bins:
			print("   open: {}, moment: {}".format(bn["open"]-moments[0].start_time,bn["moments"]))
		print("robot mod list: {}".format(robot_mod_list))

		# remove empty bins
		while len(bins) > 0 and len(bins[0]["moments"]) == 0:
			del bins[0]

		del bins[0]

		# if the first bin does not result in a change in behavior, discard it
		# e.g. if the first bin consists of the start and an end with no changes in behavior
		'''
		if len(bins) > 1:
			bin1 = bins[1]
			first_moment = bin1["moments"][0]
			if first_moment.start_time in primary_splits and first_moment.start_time not in secondary_splits:
				del bins[0]
		'''

		# create the uncompressed demos
		demo_array = []
		for bn in bins:
			if len(bn["moments"]) > 0:

				# get the cause
				cause = None
				cause_prior = None
				moment_idx = -1
				for i in range(len(moments)):
					moment = moments[i]
					if moment.start_time == bn["open"]:
						cause = moment
						moment_idx = i
						if i > 0:
							cause_prior = moments[i-1]
						else:
							cause_prior = moment
						break
				cause_dict = {}
				if cause.start_time in primary_splits:
					for item,val in cause.tracks.items():

						# get the causes
						# add to the cause dict ONLY when the state of the cause is new
						if item in cause_mod_list and not (val == "False" or val is None or val == False):
							if moment_idx == 0:
								cause_dict[item] = val
							else:
								if cause_prior.tracks[item] != val:
									cause_dict[item] = val
							described_moment = moment_copy2orig[cause]
							described_moment.description["h1"] += self.solver_helper.get_behavior_description(item,moment.tracks[item],"h1")

						# get the environmental state
						if item in env_mod_list:
							cause_dict[item] = val

				else:
					# get the self cause
					cause_dict[""] = "self"
					for item,val in cause_prior.tracks.items():
						# get the environmental state from the PRIOR moment
						if item in env_mod_list:
							cause_dict[item] = val
				# TODO: (below)

				effect_dict = {}

				# get the effect (combine them if multiple)
				effect_string = ""
				mod_set = {}
				for mod in robot_mod_list:
					for moment in bn["moments"]:
						'''
						if mod not in mod_set:
							mod_set[mod] = moment
						elif moment.tracks[mod] is not None and moment.tracks[mod] != "False":
							mod_set[mod] = moment
						'''
						if moment.tracks[mod] is not None and moment.tracks[mod] != "False":
							mod_set[mod] = moment
				for item,moment in mod_set.items():
					if item in robot_mod_list:
						effect_dict[item] = moment.tracks[item]
						if moment.tracks[item] is not None and moment.tracks[item] != "False":
							described_moment = moment_copy2orig[moment]
							described_moment.description["r"] += self.solver_helper.get_behavior_description(item,moment.tracks[item],"r")

				bin_start_moment = bn["moments"][0]
				inp = Input("",bin_start_moment)
				inp.input_dict = cause_dict
				out = Output("",bin_start_moment)
				out.output_dict = effect_dict

				demo_array.append((inp,out))

		# add a source state
		inp = Input("",moments[0])
		inp.input_dict = {"":"self"}
		out = Output("",moments[0])
		out.output_dict = {"sys":"ON"}
		demo_array.insert(0,(inp,out))

		# add a sink state
		inp = Input("",moments[-1])
		final_input_dict = {"":"self"}
		for item,val in moments[-1].tracks.items():
			if item in env_mod_list:
				final_input_dict[item] = val
		inp.input_dict = final_input_dict
		out = Output("",moments[-1])
		output_dict = {"sys":"OFF"}
		# get the final environmental state
		final_moment = moments[-1]
		for mod,val in final_moment.tracks.items():
			if mod in robot_mod_list and val is not None:
				output_dict[mod] = val
		out.output_dict = output_dict

		demo_array.append((inp,out))

		demo_id = self.trace_components.get_and_increment_demo_id()
		demo = Demo(demo_array,curr_recording,"Scene {}".format(demo_id),demo_id)
		demo.duration = int(round(curr_recording.end_time - curr_recording.start_time))

		# create a description for the scene


		# process the completed trace
		self.trace_processor.assign_movement_destination(demo)
		self.trace_processor.fill_pointless_movement(demo)
		self.trace_processor.sort_array_vals(demo)
		self.trace_processor.process_redundant_trace_states(demo, robot_mod_list)

		print(demo.pretty_string())

		'''
		if not self.flagflag and len(self.trace_components.demos) == 1:
			self.flagflag = True
			return None
		'''

		'''
		assemble the trace components
		'''
		self.trace_components.add_trace(demo,self.solver_helper.classify_event_behavior)
		# check to see whether there are any existing conflicts
		# TODO: there can now be multiple sets of conflicts per procedure
		print(len(self.trace_components.procedures))
		for event,event_val_dict in self.trace_components.procedures.items():
			for event_val,traces in event_val_dict.items():
				conflicts = self.glue_checker.check_for_conflicts(traces, self.modalities)
				print("is {} conflict free? {}".format(event,"True" if conflicts is None else False))

		return conflicts

	def get_trace_components(self):
		return self.trace_components

	def glue_traces(self):
		'''
		Debugging
		'''
		'''
		print(len(self.trace_components.demos))
		for trace in self.trace_components.demos:
			print(trace.pretty_string())
		'''

		# clear all xml files
		self.exporter.clear()

		# we define a program as a collection of procedures
		# the procedure beginning with a "self" action is defined as
		# the "main" procedure
		program = []
		
		for event,event_val_dict in self.trace_components.procedures.items():
			for event_val,traces in event_val_dict.items():

				glue = Glue(traces, self.modalities, self.glue_checker)
				glue.glue_traces()
				best_fold_storage = [glue.glued_traces]
				print("folding glue...")
				fold_start_time = time.time()
				glue.fold_glue(glue.glued_traces,best_fold_storage,fold_start_time)

				# get the best glue
				glue.glued_traces = best_fold_storage[0]
				program.append(glue.glued_traces)

				# generalize the glue
				main = False
				if event == "":
					main = True
				self.glue_generalizer.generalize(glue.glued_traces,self.solver_helper.classify_event_behavior,main=main)

				# vizualize the glue
				print(glue.glued_traces) 
				name = "{}_{}".format(event,event_val)
				glue.export_links(name)
				
				self.exporter.export_ts(glue.glued_traces,self.modalities,self.solver_helper.classify_event_behavior,name=name,filepath="")

		# export the speech categories
		self.exporter.export_speech(self.speech_db,filepath="")

		# build a transition system
		print("program has {} procedures".format(len(program)))
		for trace in self.trace_components.demos:
			print(trace.pretty_string())
		return program

class Glue:

	def __init__(self, traces, modalities, glue_checker):
		self.trace_list = traces
		self.modalities = modalities

		# dictionaries that map states and actions to their corresponding locations in the glue
		self.state2glued_state = {}
		self.action2glued_actions = {}

		# dictionaries that map glued states and actions to lists of their corresponding states and actions in the traces
		self.glued_state2states = {}
		self.glued_action2action = {}

		# the glued traces
		self.glued_traces = Glue.GluedTraces(self, self.modalities)

		# the checker
		self.glue_checker = glue_checker

	def fold_glue(self, curr_fold, best_fold_storage, fold_start_time):
		'''
		Return if more than 10 seconds have passed
		'''
		if time.time() - fold_start_time > 10.0:
			return

		best_fold = best_fold_storage[0]

		def merge(dict_lasting,dict_to_merge):
			for inp_tup,inp_val_dict in dict_to_merge.items():
				if inp_tup not in dict_lasting:
					dict_lasting[inp_tup] = {}

				for inp_val,action in inp_val_dict.items():
					if inp_val not in dict_lasting[inp_tup]:
						dict_lasting[inp_tup][inp_val] = action

		'''
		Find identital states to fold
		'''
		to_break = False
		#random.shuffle(curr_fold.gstates)
		for i in range(len(curr_fold.gstates)):
			for j in range(i+1,len(curr_fold.gstates)):
				gstate1 = curr_fold.gstates[i]
				gstate2 = curr_fold.gstates[j]
				if gstate1.compare_to(gstate2, self.modalities):
					copied_fold = curr_fold.copy()

					'''
					fold the states
					'''
					# find the relevant copied states
					copied_gstate1 = None
					copied_gstate2 = None
					for copied_gstate in copied_fold.gstates:
						if copied_gstate.id == gstate1.id:
							copied_gstate1 = copied_gstate
						if copied_gstate.id == gstate2.id:
							copied_gstate2 = copied_gstate

					# merge the states
					# what needs to be done is the following:
					# 1) all input traces from the merger gstate (2) must be given to the mergee gstate (1)
					# 2) all unique output traces from the merger gstate must be handed to the mergee state
					# 3) all gstates who had an input action from the merger gstate must have the source of their input action changed 
					# 4) remove the copied_gstate2 from the copied fold
					for source,inp_tup_dict in copied_gstate2.input_actions.items():
						for inp_tup,inp_val_dict in inp_tup_dict.items():
							for inp_val,action in inp_val_dict.items():
								action.target = copied_gstate1
								if source not in copied_gstate1.input_actions:
									copied_gstate1.input_actions[source] = {}
								if inp_tup not in copied_gstate1.input_actions[source]:
									copied_gstate1.input_actions[source][inp_tup] = {}
								copied_gstate1.input_actions[source][inp_tup][inp_val] = action

					for inp_tup,action_list in copied_gstate2.actions.items():
						if inp_tup not in copied_gstate1.actions:
							copied_gstate1.actions[inp_tup] = []
							for action in action_list:
								copied_gstate1.actions[inp_tup].append(action)
								action.source = copied_gstate1
						else:
							for action in action_list:
								action.source = copied_gstate1

								# need to check whether a duplicate of the action already exists
								exists_in_copied_gstate = False
								for existing_action in copied_gstate1.actions[inp_tup]:
									if action.compare_to(existing_action,self.glue_checker):
										exists_in_copied_gstate = True

								if exists_in_copied_gstate:
									copied_fold.gactions.remove(action)
								else:
									copied_gstate1.actions[inp_tup].append(action)

					# any other state who had an input from the state to be removed
					# that state must have its input source changed
					for gstate in copied_fold.gstates:
						if gstate != copied_gstate2:# and gstate != copied_gstate1:
							if copied_gstate2 in gstate.input_actions:
								if copied_gstate1 not in gstate.input_actions:
									gstate.input_actions[copied_gstate1] = gstate.input_actions[copied_gstate2]
								else:
									merge(gstate.input_actions[copied_gstate1],gstate.input_actions[copied_gstate2])
								del gstate.input_actions[copied_gstate2]

					copied_fold.gstates.remove(copied_gstate2)

					'''
					test if fold is better than the best fold
					'''	
					if len(best_fold.gstates) > len(copied_fold.gstates):
						best_fold = copied_fold	
						best_fold_storage[0] = best_fold

					to_break = True
					self.fold_glue(copied_fold,best_fold_storage,fold_start_time)
					#break
			#if to_break:
			#	break

		#if not to_break:
		#	print("nothing folded")

	def glue_traces_helper(self, remaining_traces, glued_traces, done_solutions):
		
		'''
		base case #0 -- we have already found a solution
		'''
		if len(done_solutions) > 0:
			return

		'''
		base case #1 -- there are no more traces to process -- we found a solution now
		'''
		if len(remaining_traces) == 0:
			done_solutions.append(glued_traces)
			return

		curr_trace = remaining_traces[0]
		trace = curr_trace.demo_array

		# stick the initial state
		unglued_init = trace[0][1]
		glued_traces.stick_to_init(unglued_init)

		# come up with the best locations to glue the trace
		state_match_dict = {unglued_init: glued_traces.initial_state}
		action_match_dict = {}
		matches = []
		curr_match = state_match_dict
		start_glued_state = glued_traces.initial_state
		start_unglued_index = 1
		self.find_candidate_matches(start_glued_state,start_unglued_index,trace,curr_match,matches,glued_traces=glued_traces)

		if len(matches) == 0:
			return

		sorted_matches = sorted(matches,key=len,reverse=True)
		for match in sorted_matches:
			if len(done_solutions) > 0:
				break

			deletable_actions,deletable_states,undoable_state_mods = self.assemble_glued_traces(trace,match,glued_traces=glued_traces)
			# add state ids to glued states for visualization

			copied_glued_traces = glued_traces.copy()
			self.glue_traces_helper(remaining_traces[1:],copied_glued_traces,done_solutions)
			self.undo_assemble_glued_traces(deletable_actions,deletable_states,undoable_state_mods,glued_traces=glued_traces)

	def glue_traces(self):
		unsorted_trace_list = self.trace_list

		# get a sorted trace list
		trace_list = sorted(unsorted_trace_list,key=len,reverse=True)
		for trace in trace_list:
			print(trace.pretty_string())

		done_solutions = []
		self.glue_traces_helper(trace_list,self.glued_traces,done_solutions)

		self.glued_traces = done_solutions[0]

		# add state ids to glued states for visualization
		counter = 0
		for state in self.glued_traces.gstates:
			state.id = counter
			counter += 1

		print(str(self.glued_traces))

	'''
	def glue_traces(self):
		unsorted_trace_list = self.trace_list

		# get a sorted trace list
		trace_list = sorted(unsorted_trace_list,key=len,reverse=True)
		
		# loop through traces
		counter = 0
		for trace_obj in trace_list:
			trace = trace_obj.demo_array

			# stick the initial state
			unglued_init = trace[0][1]
			self.glued_traces.stick_to_init(trace[0][1])

			# come up with the best locations to glue the trace
			state_match_dict = {unglued_init:self.glued_traces.initial_state}
			action_match_dict = {}
			matches = []
			curr_match = state_match_dict
			start_glued_state = self.glued_traces.initial_state
			start_unglued_index = 1
			print("FINDING CANDIDATE MATCH for TRACE {}".format(counter))
			self.find_candidate_matches(start_glued_state,start_unglued_index,trace,curr_match,matches)

			# determine the best possible match (the one that satisfies all properties and in which most states in the trace are matched up)
			best_candidate_match = None
			for match in matches:
				state_match_dict = match
				best_candidate_state_match_dict = best_candidate_match

				if best_candidate_match is None or (len(list(state_match_dict.keys())) > len(list(best_candidate_state_match_dict.keys()))):
					match_copy = copy(match)
					deletable_actions,deletable_states,undoable_state_mods = self.assemble_glued_traces(trace,match_copy)
					match_len=len(list(state_match_dict.keys()))
					#if self.glue_checker.check_properties(self.glued_traces):
					best_candidate_match = match
					self.undo_assemble_glued_traces(deletable_actions,deletable_states,undoable_state_mods)

			# now assemble the glued traces based on the best candidate match
			self.assemble_glued_traces(trace,best_candidate_match)
			counter += 1

		# add state ids to glued states for visualization
		counter = 0
		for state in self.glued_traces.gstates:
			state.id = counter
			counter += 1

		print(str(self.glued_traces))
	'''

	def assemble_glued_traces(self, trace, match, glued_traces=None):
		'''
		Stick a trace to the glue. Grow the glue. Be happy for the glue. Don't eat the glue.
		'''
		if glued_traces is None:
			glued_traces = self.glued_traces

		# create an undoable
		deletable_actions = {}
		deletable_states = []
		undoable_state_mods = {}

		def add_actions(prev_trace_output, trace_inputs, glue_output):
			'''
			Add the actions to a gstate
			'''

			# step 1: create the input and value tuples
			sorted_inputs = tuple(sorted(list(trace_inputs.keys())))
			sorted_vals = []
			for inp in sorted_inputs:
				sorted_vals.append(trace_inputs[inp])

			# create a hashable version of the sorted vals
			hashable_sorted_vals = self.glue_checker.hash_action_vals(sorted_vals)

			sorted_vals = tuple(sorted_vals)

			prev_glued_state = match[trace[i-1][1]]
			if prev_glued_state in glue_output.input_actions and sorted_inputs in glue_output.input_actions[prev_glued_state] and hashable_sorted_vals in glue_output.input_actions[prev_glued_state][sorted_inputs]:
				return # action is already present
			gaction = Glue.GluedAction()
			glued_traces.gactions.append(gaction)
			gaction.action_label = sorted_inputs
			gaction.action_val = sorted_vals
			gaction.source = prev_glued_state
			gaction.target = glue_output
			if gaction.source not in glue_output.input_actions:
				glue_output.input_actions[prev_glued_state] = {}
			if sorted_inputs not in glue_output.input_actions[prev_glued_state]:
				glue_output.input_actions[prev_glued_state][sorted_inputs] = {}
			glue_output.input_actions[prev_glued_state][sorted_inputs][hashable_sorted_vals] = gaction

			# add the undoable
			if gaction.source not in deletable_actions:
				deletable_actions[gaction.source] = {}
			if gaction.target not in deletable_actions[gaction.source]:
				deletable_actions[gaction.source][gaction.target] = []
			deletable_actions[gaction.source][gaction.target].append(gaction)

			'''
			NOTE: the code below could be buggy IF act_tup is already present in the prev_glued_state actions

			We currently solve this issue by doing a simple check to see whether the act_obj is already in the set
			of the prev-state's actions
			'''
			for input_act_tup,inp_val_dict in glue_output.input_actions[prev_glued_state].items():
				for inp_val,act_obj in inp_val_dict.items():
					if input_act_tup in prev_glued_state.actions:
						if act_obj not in prev_glued_state.actions[input_act_tup]:
							prev_glued_state.actions[input_act_tup].append(act_obj)
					else:
						prev_glued_state.actions[input_act_tup] = [act_obj]

		for i in range(len(trace)):
			item = trace[i]

			trace_input_obj = item[0]
			trace_inputs = trace_input_obj.input_dict

			trace_output_obj = item[1]
			glue_output = None
			if trace_output_obj in match:
				glue_output = match[trace_output_obj]

			prev_trace_output = trace[i-1][1]

			if item != trace[0]:

				# case where there is a match between the trace state and a glue state
				if glue_output is not None:
					# add input actions
					add_actions(prev_trace_output, trace_inputs, glue_output)

					undoable_state_mods[glue_output] = deepcopy(glue_output.behaviors)

					# glue the trace to the state
					glued_traces.glue_state(trace_output_obj,glue_output)

				# case where there is no match between the trace state and a glue state
				# we must create a match
				else:
					glued_state = Glue.GluedState()
					glued_traces.gstates.append(glued_state)
					
					for key,val in trace_output_obj.output_dict.items():
						# add behaviors
						glued_state.behaviors[key] = val

					# add input actions
					add_actions(prev_trace_output, trace_inputs, glued_state)

					# add the match to the match dictionary
					match[trace[i][1]] = glued_state

					# add the undoable
					deletable_states.append(glued_state)

		return deletable_actions,deletable_states,undoable_state_mods

	def undo_assemble_glued_traces(self,deletable_actions,deletable_states,undoable_state_mods,glued_traces=None):
		if glued_traces is None:
			glued_traces = self.glued_traces

		for source, target_dict in deletable_actions.items():
			for target, array in target_dict.items():
				for action in array:
					source.actions[action.action_label].remove(action)
					if len(source.actions[action.action_label]) == 0:
						del source.actions[action.action_label]
					del target.input_actions[source][action.action_label][self.glue_checker.hash_action_vals(action.action_val)]
					if len(target.input_actions[source][action.action_label].keys()) == 0:
						del target.input_actions[source][action.action_label]
					if len(target.input_actions[source].keys()) == 0:
						del target.input_actions[source]

					glued_traces.gactions.remove(action)

		for state in deletable_states:
			glued_traces.gstates.remove(state)

		for gstate,mods in undoable_state_mods.items():
			for beh,val in mods.items():
				gstate.behaviors[beh] = val

		# for all gactions, flip the active_guards to None
		for gaction in glued_traces.gactions:
			gaction.active_guards = None

	def find_candidate_matches(self, start_glued_state, start_unglued_index, trace, curr_match, matches, previous_glued_state=None, glued_traces=None):
		if glued_traces is None:
			glued_traces = self.glued_traces
		'''
		0) base case -- the trace has been iterated through
		'''
		if start_unglued_index >= len(trace):
			matches.append(curr_match)
			return

		'''
		1) match the action to glued actions
		'''
		# first get the unglued action
		unglued_actions = trace[start_unglued_index][0]    # THESE UNGLUED ACTIONS SHOULD BE TREATED AS "AND" STATEMENTS
		unglued_state = trace[start_unglued_index][1]

		# there are no corresponding glued state that was found for the current trace idx
		if start_glued_state is None and previous_glued_state is not None:
			# find a new matching state for the unglued state
			non_parental_candidate_states = []

			# 1) find non-parental states (also must be different states)
			for gstate in glued_traces.gstates:
				if not glued_traces.is_child_of_other_state(previous_glued_state,gstate):
					if glued_traces.check_if_state_gluable(unglued_state,gstate):
						non_parental_candidate_states.append(gstate)

			# 2) whether or not no non-parentals exist, continue new branch
			self.find_candidate_matches(None,start_unglued_index+1,trace,curr_match,matches,previous_glued_state,glued_traces=glued_traces)

			# 3) enumerate over all non-parental states that exist
			for candidate_state in non_parental_candidate_states:
				new_curr_match = copy(curr_match)
				new_curr_match[unglued_state] = candidate_state
				self.find_candidate_matches(candidate_state,start_unglued_index+1,trace,new_curr_match,matches,glued_traces=glued_traces)
		elif start_glued_state is None and previous_glued_state is None:
			'''
			This condition should never happen
			'''
			print("ERROR: there always needs to be a previous state tracked if the start glued state is None")
			exit(1)
		else:
			# case 1: if any matches actions were found that lead to non-matching states, the match should be discarded
			glued_target = None
			action_vals_same = True
			for action_cat,action_val_list in start_glued_state.actions.items():
				if self.input_dict_matches_actions(action_cat, unglued_actions.input_dict):  # found a category match
					for action_val in action_val_list:
						action_val_same = self.input_dict_vals_match_action_vals(action_val.action_val,unglued_actions.input_dict)
						if action_val_same: # found a val match

							# test to see if the target and unglued state are gluable
							if glued_traces.check_if_state_gluable(unglued_state,action_val.target):
								glued_target = action_val.target

							# else return
							else:
								print("returning")
								return

			# case 2: if some or all action matches were found, we record it and move to the next state and continue building the match
			if glued_target is not None:
				new_curr_match = copy(curr_match)
				new_curr_match[unglued_state] = glued_target
				self.find_candidate_matches(glued_target,start_unglued_index+1,trace,new_curr_match,matches,glued_traces=glued_traces)

			# case 2
			else:
				# if no action matches were found, we need to begin a new branch (match is good, but won't be continued on this branch)
				# 1) search for a non-parental state to the current glued state that matches the unglued state
				# 2) if no non-parental state exists, we need to begin a new branch
				# 3) for every non-parental state does exist, (1) go to those states with new candidate matches. Also begin a new branch anyway (new recursive branch)
				non_parental_candidate_states = []

				# 1) find non-parental states (also must be different states)
				for gstate in glued_traces.gstates:
					if not glued_traces.is_child_of_other_state(start_glued_state,gstate):
						if glued_traces.check_if_state_gluable(unglued_state,gstate):
							non_parental_candidate_states.append(gstate)

				# 2) whether or not no non-parentals exist, begin new branch
				self.find_candidate_matches(None,start_unglued_index+1,trace,curr_match,matches,start_glued_state,glued_traces=glued_traces)

				# 3) enumerate over all non-parental states that exist
				for candidate_state in non_parental_candidate_states:
					new_curr_match = copy(curr_match)
					new_curr_match[unglued_state] = candidate_state
					self.find_candidate_matches(candidate_state,start_unglued_index+1,trace,new_curr_match,matches,glued_traces=glued_traces)

	def input_dict_matches_actions(self, action_tuple, input_dict):
		input_list = sorted(list(input_dict.keys()))
		input_tuple = tuple(input_list)
		if input_tuple == action_tuple:
			return True
		return False

	def input_dict_vals_match_action_vals(self, action_val_tuple, input_dict):
		input_list = sorted(list(input_dict.keys()))
		vals_list = []
		for inp in input_list:
			vals_list.append(input_dict[inp])

		vals_tuple = tuple(vals_list)
		if vals_tuple == action_val_tuple:
			return True
		return False

	def export_links(self,name):
		self.glued_traces.export_links(name)

	class GluedTraces:

		def __init__(self, glue, modalities):
			self.glue = glue
			self.modalities = modalities
			self.gstates = []
			self.gactions = []
			self.initial_state = None

		def stick_to_init(self, state):
			'''
			Stick a trace to the initial glued trace
			'''

			if self.initial_state is None: # if the first trace that we are trying to stick
				self.initial_state = self.glue.GluedState()
				self.gstates.append(self.initial_state)

				for out_cat,out_val in state.output_dict.items():
					self.initial_state.behaviors[out_cat] = out_val
			else:
				if self.check_if_state_gluable(state,self.initial_state):
					self.glue_state(state,self.initial_state)
				else:
					# TODO: return the non-gluable components for review
					pass

		def glue_state(self,unglued_state,glued_state):
			'''
			Stick an unglued state to a glued state

			Preconditions are that this is possible
			'''
			for out_cat, out_val in unglued_state.output_dict.items():

				# the output must exist
				if out_cat not in glued_state.behaviors:
					print("ERROR: no output {} present in {}".format(out_cat,glued_state))
					exit(1)

				# the outputs must already be the same, regardless of whether it is an array or not
				if glued_state.behaviors[out_cat] != out_val:
					print("ERROR: glued outputs cannot match with unglued outputs")
					exit(1)

			return False

		def check_if_state_gluable(self,unglued_state,glued_state):
			'''
			Method for checking vals
			'''
			def check_vals(out_cat,out_val,glued_state):
				unglued_sorted = sorted(out_val)
				glued_sorted = sorted(glued_state.behaviors[out_cat])
				is_same = False
				if len(unglued_sorted) == len(glued_sorted):
					is_same = all([unglued_sorted[i]==glued_sorted[i] for i in range(len(glued_state.behaviors[out_cat]))])
				return is_same
			'''
			Check if a state is gluable to a glued state
			'''
			gluable = True
			for out_cat, out_val in unglued_state.output_dict.items():

				if out_cat not in glued_state.behaviors or glued_state.behaviors[out_cat] != out_val:
					gluable = False
					break

				# case 1: dealing with an array
				if self.modalities[out_cat]["array"]:

					if out_cat not in glued_state.behaviors:
						gluable = False
						break

					is_same = check_vals(out_cat,out_val,glued_state)
					if not is_same:
						gluable = False
						break

				# case 2: dealing with single-values
				else:

					if out_cat not in glued_state.behaviors or glued_state.behaviors[out_cat] != out_val:
						gluable = False
						break

			for out_cat, out_val in glued_state.behaviors.items():

				if self.modalities[out_cat]["array"]:
					if out_cat not in unglued_state.output_dict:
						gluable = False
						break

					is_same = check_vals(out_cat,out_val,glued_state)
					if not is_same:
						gluable = False
						break

				else:
					if out_cat not in unglued_state.output_dict or unglued_state.output_dict[out_cat] != out_val:
						gluable = False
						break

			return gluable

		def is_child_of_other_state(self, state, other):
			'''
			Recursive BFS method for finding of other is a parent of state
			'''

			# base case 1: if other is equal to the state
			if state == other:
				return True

			# base case 2: if we have enumerated through all of other's children
			if len(list(other.actions.keys())) == 0:
				return False

			action_leads_to_child = False
			for action,action_list in other.actions.items():
				for action_object in action_list:
					if action_object.target == state:
						action_leads_to_child = True
						break

			if action_leads_to_child:
				return True

			accumulated_result = False
			for action,action_list in other.actions.items():
				for action_object in action_list:
					accumulated_result = (accumulated_result or self.is_child_of_other_state(state,action_object.target))
					if accumulated_result == True:
						break

			return accumulated_result

		def __str__(self):
			string = "\nGLUED TRACE\n"

			for gstate in self.gstates:
				string += str(gstate)
			for gaction in self.gactions:
				string += str(gaction)

			return string

		def export_links(self,name):
			'''
			Export links: the source, source name, target, and input
			Export states: {"id": state.id, "name": state.name, "gesture": None, "reachble": True, "final": False, "prob": 0.0, "micro": state.micros[0]["name"]}
			'''

			def make_state_name(state):
				name = "{}.\n".format(str(state.id))

				for beh,val in state.behaviors.items():
					name += "{} - {}\n".format(beh,val)

				return name

			print("exporting json links")
			json_array = {}
			json_array["links"] = []
			state2name = {}
			print("exporting transitions")

			action_groups = {}
			for action in self.gactions:
				if action.source not in action_groups:
					action_groups[action.source] = {}
				if action.target not in action_groups[action.source]:
					action_groups[action.source][action.target] = []
				action_groups[action.source][action.target].append(action)

			for source, target_dict in action_groups.items():
				for target, action_list in target_dict.items():
					inp = ""
					for action in action_list:
						inp += "<"
						for i in range(len(action.action_label)):
							inp += ("{} - {}\n".format(action.action_label[i],action.action_val[i]))
						inp = inp.strip()
						inp += ">\n\n"
					inp = inp.strip() 

					# setup the state names
					state2name[source] = make_state_name(source)
					state2name[target] = make_state_name(target)
					link = {"source":source.id,"source_name":state2name[source],"target":target.id,"input":inp}
					json_array["links"].append(link)

			# in case any states did not have their names created
			for state in self.gstates:
				if state not in state2name:
					state2name[state] = make_state_name(state)

			print("exporting states")
			json_array["states"] = []
			for state in self.gstates:
				state_dict = {"id": state.id, "name": state2name[state], "gesture": None, "reachble": True, "final": False, "prob": 0.0, "micro": "None"}
				json_array["states"].append(state_dict)

			path = os.getcwd()
			idx = path.index("/Figaro/ctrl/ctrl")
			vispath = path[:idx] + "/Figaro/ctrl/ctrl/d3js/links_{}.json".format(name)

			with open(vispath,"w") as outfile:
				json.dump(json_array,outfile)

		def copy(self):

			# get each individual component
			copied_gstates = []
			gstate2copied = {}
			copied_gactions = []
			gaction2copied = {}
			for gstate in self.gstates:
				copied_gstate = gstate.copy_without_actions()
				copied_gstates.append(copied_gstate)
				gstate2copied[gstate] = copied_gstate
			for gaction in self.gactions:
				copied_gaction = gaction.copy_without_gstates()
				copied_gactions.append(copied_gaction)
				gaction2copied[gaction] = copied_gaction

			# perform linkages
			for gaction in self.gactions:
				source = gaction.source
				target = gaction.target

				copied_gaction = gaction2copied[gaction]
				copied_source = gstate2copied[source]
				copied_target = gstate2copied[target]
				copied_gaction.source = copied_source
				copied_gaction.target = copied_target

			for gstate in self.gstates:
				copied_gstate = gstate2copied[gstate]
				for inp_tup in gstate.actions:
					if inp_tup not in copied_gstate.actions:
						copied_gstate.actions[inp_tup] = []
					for gaction in gstate.actions[inp_tup]:
						copied_gstate.actions[inp_tup].append(gaction2copied[gaction])
				for source in gstate.input_actions:
					copied_source = gstate2copied[source]
					copied_gstate.input_actions[copied_source] = {}
					for inp_tup,inp_val_dict in gstate.input_actions[source].items():
						for inp_val in inp_val_dict:
							if inp_tup not in copied_gstate.input_actions[copied_source]:
								copied_gstate.input_actions[copied_source][inp_tup] = {}
							copied_gstate.input_actions[copied_source][inp_tup][inp_val] = gaction2copied[gstate.input_actions[source][inp_tup][inp_val]]

			copied_initial_state = gstate2copied[self.initial_state]
			new_glued_traces = Glue.GluedTraces(self.glue, self.modalities)
			new_glued_traces.initial_state = copied_initial_state
			new_glued_traces.gstates = copied_gstates
			new_glued_traces.gactions = copied_gactions

			return new_glued_traces

	class GluedState:
		
		def __init__(self):
			# id for visualization purposes
			self.id = None

			# a set of behaviors -- the absence of a behavior means "false"
			self.behaviors = {}

			# actions
			# structure is: actions[input_cat_tup] = [action1,action2,...]
			self.actions = {}

			# input actions
			# structure is: input_actions[source][input_tup][input_val] = action
			self.input_actions = {}

		def copy_without_actions(self):
			new_gstate = Glue.GluedState()

			new_gstate.id = self.id
			new_gstate.behaviors = deepcopy(self.behaviors)

			return new_gstate

		def compare_to(self, other, modalities):
			'''
			We care about the following:
			- behaviors are the same
			- for actions that are the same, they must lead to the same state 
			'''
			# check if the behavior keys are the same
			has_same_behavior_cats = True
			my_behaviors = tuple(sorted(list(self.behaviors.keys())))
			other_behaviors = tuple(sorted(list(other.behaviors.keys())))
			if my_behaviors != other_behaviors:
				return False

			# check if the behavior vals are the same
			has_same_behavior_vals = True
			for cat in my_behaviors:
				if not modalities[cat]["array"]:
					if self.behaviors[cat] != other.behaviors[cat]:
						has_same_behavior_vals = False
						break
				else:
					self_sorted = sorted(self.behaviors[cat])
					other_sorted = sorted(other.behaviors[cat])
					if len(self_sorted) != len(other_sorted):
						has_same_behavior_vals = False
						break
					array_is_same = True
					for i in range(len(self_sorted)):
						if self_sorted[i] != other_sorted[i]:
							array_is_same = False
					if not array_is_same:
						has_same_behavior_vals = False
						break
			if not has_same_behavior_vals:
				return False

			# check that the same actions lead to the same state
			actions_match = True
			for input_tup in self.actions:
				if input_tup in other.actions:
					for action_1 in self.actions[input_tup]:
						for action_2 in other.actions[input_tup]:
							if action_1.action_val == action_2.action_val:
								'''
								Destination should be the same
								'''
								if action_1.target != action_2.target:
									actions_match = False
							
			if not actions_match:
				return False

			return True

		def __str__(self):
			string = "Glued State {}: \n".format(self.id)
			for beh,val in self.behaviors.items():
				string += "  Behavior: {} - {}\n".format(beh,val)
			for inp_tup,act_list in self.actions.items():
				for action in act_list:
					string += "  Action from {} to {}  -----  {} - {}\n".format(action.source.id,action.target.id, action.action_label,action.action_val)
			for source,inp_tup_dict in self.input_actions.items():
				for inp_tup,inp_val_dict in inp_tup_dict.items():
					for inp_val,action in inp_val_dict.items():
						string += "  Input action from {} to {}  -----  {} - {}\n".format(action.source.id,action.target.id, action.action_label,action.action_val)
			return string

	class GluedAction:

		def __init__(self):
			self.action_label = None
			self.action_val = None

			self.source = None
			self.target = None

			self.env_vars = None

		def copy_without_gstates(self):
			new_gaction = Glue.GluedAction()

			new_gaction.action_label = deepcopy(self.action_label)
			new_gaction.action_val = deepcopy(self.action_val)
			new_gaction.env_vars = deepcopy(self.env_vars)

			return new_gaction

		def remove_label(self,label):
			action_label_list = list(self.action_label)
			action_val_list = list(self.action_val)

			idx = action_label_list.index(label)
			del action_label_list[idx]
			del action_val_list[idx]

			self.action_label = tuple(action_label_list)
			self.action_val = tuple(action_val_list)

		def remove_position(self,position):
			action_val_list = list(self.action_val)

			idx = action_label_list.index("position")

			position_list = list(action_val_list[idx])
			position_list.remove(position)

			position_tuple = tuple(position_list)
			action_val_list[idx] = position_tuple
			self.action_val = tuple(action_val_list)


		def compare_to(self, other, glue_checker):
			'''
			Precondition is that the action labels and values are sorted
			'''
			if other.source != self.source:
				return False

			if other.target != self.target:
				return False

			if len(self.action_label) != len(other.action_label):
				return False

			is_same = True
			for i in range(len(self.action_label)):
				if self.action_label[i] != other.action_label[i]:
					is_same = False
				if self.action_val[i] != other.action_val[i]:
					is_same = False

			if not is_same:
				return False

			return True



		def __str__(self):
			string = "Glued Action: \n"
			string += "  From {} to {}".format(self.source.id, self.target.id)
			string += "  {} - {}\n".format(self.action_label,self.action_val)
			if self.env_vars is not None:
				string += " {}\n".format(self.env_vars)
			return string
		
if __name__=="__main__":
	'''
	Testing the glue:
	1) Load a set of tests
	2) glue the traces together
	'''
	from os import listdir
	from os.path import isfile, join
	files = [f for f in listdir("glue_tests") if isfile(join("glue_tests", f))]
	files.sort()

	# load the input modalities
	import sys
	sys.path.append("/home/david/Documents/Figaro/ctrl/ctrl/controller")
	from input_modality_handler import *
	load_folder = "/home/david/Documents/Figaro/ctrl/ctrl/session_parameters/temi"
	input_modality_handler = InputModalityHandler(load_folder)
	modalities = input_modality_handler.get_input_modality_dict()
	
	# run the tests
	for fname in ["test8.json"]:
		f = open("glue_tests/{}".format(fname),"r")
		json_data = json.load(f)

		# create the traces
		traces = []
		scene_id = 0
		for trace_data in json_data:
			demo_array = []

			i=0
			while i+1 < len(trace_data):
				# handle the input
				new_input = Input("")
				new_input.input_dict = trace_data[i]

				# handle the output
				new_output = Output("")
				new_output.output_dict = trace_data[i+1]

				demo_array.append((new_input,new_output))
				i+=2

			# make the demo
			trace = Demo(demo_array,"trace",scene_id)
			traces.append(trace)

			scene_id += 1

		# run the test
		glue_checker = GlueChecker()
		glue = Glue(traces, modalities, glue_checker)
		#glue_checker.check_for_conflicts(traces,modalities)
		glue.glue_traces()
		best_fold_storage = [glue.glued_traces]
		glue.fold_glue(glue.glued_traces,best_fold_storage)

		# visualize the results in a links file
		glue.glued_traces = best_fold_storage[0]
		print(glue.glued_traces) 
		glue.export_links()