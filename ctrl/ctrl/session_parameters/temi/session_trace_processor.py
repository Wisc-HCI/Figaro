class TraceProcessor:

	def __init__(self):
		pass		

	def process_redundant_trace_states(self, trace_obj, robot_mod_list):
		# look through each one
		# if next event is "self" but the conditionals are the same as previous conditionals AND nothing happens in the behavior in between
		# then remove the self transition and empty behavior

		flagged_idxs = []

		trace = trace_obj.demo_array

		for i in range(len(trace)-1):
			out = trace[i][0]
			next_out = trace[i+1][0]

			out_dict = out.input_dict
			next_dict = next_out.input_dict

			is_match = False
			if "" in next_dict: # there is a self command
				is_match = True
				for next_category in next_dict:
					if next_category is not "": # only look at conditionals
						output = next_dict[next_category]

						if next_category not in out_dict or out_dict[next_category] != output:
							is_match = False
							break


			if is_match:
				beh = trace[i][1]
				if not (len(beh.output_dict) == 1 and "movement" in beh.output_dict and len(beh.output_dict["movement"]) == 0):
					is_match = False

			# perform the surgery
			if is_match:
				flagged_idxs.append(i)

		flagged_idxs = sorted(flagged_idxs,reverse=True)

		for idx in flagged_idxs:
			beh = trace[idx][1]
			self_trans = trace[idx+1][0]

			replace_beh = trace[idx+1][1]

			trace[idx] = (trace[idx][0],replace_beh)

			del trace[idx+1]

	def assign_movement_destination(self, trace_obj):
		trace = trace_obj.demo_array

		for i in range(len(trace)-1):
			beh = trace[i][1]
			beh_out = trace[i+1][0]

			if "movement" in beh.output_dict and beh.output_dict["movement"] == ["True"]:

				# look ahead to environment of next state
				if i == len(trace) - 2:
					print("ERROR: movement should not occur on final state of trace")
					exit()
				env = trace[i+2][0]
				position = env.input_dict["position"]

				# assign position
				beh.output_dict["movement"] = []
				for pos in position:
					if pos != "movement":
						beh.output_dict["movement"].append(pos)

						# it is possible that this position has already been assigned
						if pos not in beh_out.input_dict["position"]:
							beh_out.input_dict["position"].append(pos)

				# it is possible that the robot reached the human before reaching
				# its destination
				if len(beh.output_dict["movement"]) == 0:
					beh.output_dict["movement"].append("True")

	def fill_pointless_movement(self,trace_obj):

		def test_position_equivalence(arr1,arr2):
			incl=True
			for item in arr1:
				if item is not "movement" and item not in arr2:
					incl=False
					break
			return incl

		trace = trace_obj.demo_array
		for i in range(len(trace) - 1):
			beh = trace[i][1]
			env = trace[i][0]
			curr_pos = env.input_dict["position"] if "position" in env.input_dict else []

			# we have a problem, because Temi cannot recognize "True" as a location
			if "movement" in beh.output_dict and beh.output_dict["movement"] == ["True"]:
				newVal = "True"
				#for j in range(i+1,len(trace)):
				j = i+1
				while newVal == "True" and j < len(trace) and test_position_equivalence(curr_pos,trace[j][0].input_dict["position"]):
					if "position" in trace[j][0].input_dict:
						for pos in trace[j][0].input_dict["position"]:
							if pos != "movement":
								newVal = pos
					j += 1

				beh.output_dict["movement"] = [newVal]

	def sort_array_vals(self,trace_obj):
		trace = trace_obj.demo_array

		for i in range(len(trace)):

			inp = trace[i][0]
			out = trace[i][1]

			for item,val in inp.input_dict.items():
				if isinstance(val,list):
					val.sort()