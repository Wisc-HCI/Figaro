class GlueChecker:

	def __init__(self):
		pass

	def hash_action_vals(self,sorted_val_list):
		hashable_val_list = []
		for item in sorted_val_list:
			if type(item) is list:
				hashable_item = tuple(item)
			else:
				hashable_item = item
			hashable_val_list.append(hashable_item)
		return tuple(hashable_val_list)

	def check_for_conflicts(self, traces, modalities):

		trace_list = []
		for trace_obj in traces:
			trace_list.append(trace_obj)

		max_trace_len = max([len(trace) for trace in trace_list])

		initial_cluster = [trace for trace in trace_list]
		clusters = [initial_cluster]

		def check_cluster_consistency(clusters, i):
			# perform an initial check to ensure that each initial state matches

			conflict_storage = None
			# assume each cluster has already been split by input
			# check that each cluster is homogenous
			for cluster in clusters:
				beh_dict = {}
				for trace_obj in cluster:
					trace = trace_obj.demo_array
					sorted_beh_keys = tuple(sorted([key for key in trace[i][1].output_dict]))
					sorted_beh_values = []
					for key in sorted_beh_keys:
						sorted_beh_values.append(trace[i][1].output_dict[key])
					sorted_beh_values = self.hash_action_vals(sorted_beh_values)
					if sorted_beh_keys not in beh_dict:
						beh_dict[sorted_beh_keys] = {}
					if sorted_beh_values not in beh_dict[sorted_beh_keys]:
						beh_dict[sorted_beh_keys][sorted_beh_values] = []
					beh_dict[sorted_beh_keys][sorted_beh_values].append(trace_obj)
				conflicts = []
				for beh_key,beh_val_dict in beh_dict.items():
					for beh_val,trace_obj in beh_val_dict.items():
						conflicts.append(trace_obj)
				if len(conflicts) > 1:
					print("found a conflict at position {}".format(i))
					print("behavioral differences are as follows:")
					for beh,beh_val_dict in beh_dict.items():
						print(beh)
						for beh_val, trace in beh_val_dict.items():
							print("  ---  {}".format(beh_val))
						print("")
					conflict_storage = Conflict(beh_dict,conflicts,i)

			return conflict_storage

		# check each cluster to ensure that the behaviors are the same
		conflict_storage = check_cluster_consistency(clusters,0)
		if conflict_storage is not None:
			return conflict_storage
		for i in range(1,max_trace_len):
			
			# iterate through clusters and make new clusters
			updated_cluster_list = []
			for cluster in clusters:

				# cluster based on inputs
				inp_dict = {}
				for trace_obj in cluster:
					trace = trace_obj.demo_array
					if i < len(trace):
						sorted_inp_keys = tuple(sorted([key for key in trace[i][0].input_dict]))
						sorted_inp_values = []
						for key in sorted_inp_keys:
							sorted_inp_values.append(trace[i][0].input_dict[key])
						sorted_inp_values = self.hash_action_vals(sorted_inp_values)
						if sorted_inp_keys not in inp_dict:
							inp_dict[sorted_inp_keys] = {}
						if sorted_inp_values not in inp_dict[sorted_inp_keys]:
							inp_dict[sorted_inp_keys][sorted_inp_values] = []
						inp_dict[sorted_inp_keys][sorted_inp_values].append(trace_obj)

				new_clusters = []
				for inp_key,inp_val_dict in inp_dict.items():
					for inp_val,trace_obj in inp_val_dict.items():
						new_clusters.append(trace_obj)

				for new_cluster in new_clusters:
					updated_cluster_list.append(new_cluster)

			clusters = updated_cluster_list

			# check each cluster to ensure that the behaviors are the same
			conflict_storage = check_cluster_consistency(clusters,i)
			if conflict_storage is not None:
				break

		return conflict_storage


	def check_properties(self, glued_trace):
		p1 = self.property_human_approaches_robot(glued_trace.initial_state,False)
		#print("P1 is {}".format(p1))
		#if not p1:
		#	exit(1)
		return p1

	'''
	Bounded Model Checking Properties
	'''
	def property_human_approaches_robot(self, gstate, h1_definitely_near):
		sat = True
		#if "close_to_human" in gstate.states:
		if h1_definitely_near:
			if gstate.states["close_to_human"] == "WILDCARD":
				sat = False
			elif gstate.states["close_to_human"]:
				sat = True

		targetted_states = []

		for act_label,action_list in gstate.actions.items():
			for action in action_list:

				if "h1_position" in act_label:
					h1_pos_idx = act_label.index("h1_position")

					if action.action_val[h1_pos_idx] == "robot":
						sat = (sat and self.property_human_approaches_robot(action.target,True))
						targetted_states.append(action.target)

		for act_label,action_list in gstate.actions.items():
			for action in action_list:
				if action.target not in targetted_states:
					sat = (sat and self.property_human_approaches_robot(action.target,False))
					targetted_states.append(action.target)

		return sat

class Conflict:

	def __init__(self,beh_dict,conflicts,index):
		self.beh_dict = beh_dict
		self.conflicts = conflicts
		self.index = index