class TraceProcessor:

	def __init__(self):
		pass

	def assign_movement_destination(self, trace_obj):
		pass

	def sort_array_vals(self,trace_obj):
		trace = trace_obj.demo_array

		for i in range(len(trace)):

			inp = trace[i][0]
			out = trace[i][1]

			for item,val in inp.input_dict.items():
				if isinstance(val,list):
					val.sort()