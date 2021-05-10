class TraceProcessor:

	def __init__(self):
		pass

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

	def sort_array_vals(self,trace_obj):
		trace = trace_obj.demo_array

		for i in range(len(trace)):

			inp = trace[i][0]
			out = trace[i][1]

			for item,val in inp.input_dict.items():
				if isinstance(val,list):
					val.sort()