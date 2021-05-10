class RobotAxioms:

	def __init__(self):
		pass

	def ensure_position_and_movement_overlap(self,moments):
		# if movement is currently True and position is SOMETHING, and (1) the next movement is False and (2) the next position is ["X"], then add "movement" to the nexr position
		for i in range(len(moments)-1):
			curr_moment = moments[i]
			next_moment = moments[i+1]

			if curr_moment.tracks["movement"] == ["True"] and next_moment.tracks["movement"] != ["True"]:
				curr_position = curr_moment.tracks["position"]
				next_position = next_moment.tracks["position"]

				if set(curr_position) != set(next_position):
					next_moment.tracks["movement"] = ["True"]

	def process_position_movement(self, moments):
		for i in range(len(moments)):
			moment = moments[i]

			for human in ["h1"]:
				if moment.tracks["position"] is not None:
					for item in moment.tracks["position"]:
						if human in item:
							moment.tracks["close_to_human"] = True
							moment.tracks["position"].remove(item)

			# combine robot position and movement
			if moment.tracks["movement"] == ["True"]:
				if moment.tracks["position"] is None:
					moment.tracks["position"] = ["movement"]
				else:
					moment.tracks["position"].append("movement")

				'''
				# look ahead to see if the next position is not movement
				if i < len(moments) - 1:
					if moments[i+1].tracks["movement"] is None:
						lookahead_position = moments[i+1].tracks["position"]
						if lookahead_position is not None:
							for pos in lookahead_position:

								
								#Discard position from human (it is redundant info)
								
								detected_human_position = False
								for human in ["h1"]:
									if human in pos:
										detected_human_position = True

								if not detected_human_position:
									moment.tracks["position"].append(pos)
				'''

			# combine human position and movement
			# TODO: remove this
			for human in ["h1"]:
				if moment.tracks["{}_position".format(human)] is not None and any("robot" in string for string in moment.tracks["{}_position".format(human)]):
					moment.tracks["{}_near_rob".format(human)] = True
				moment.tracks["{}_movement".format(human)] = None
				moment.tracks["{}_position".format(human)] = None

	def remove_unrecognizable_objects_or_regions(self, moments, objects, regions):

		# objects and regions are lists of tuples at the moment containing both name and coordinate data
		# must extract only the name
		'''
		obj_name_list = []
		for obj in objects:
			obj_name_list.append(obj[0])
		print(obj_name_list)
		exit()
		'''
		####################################

		for moment in moments:
			if moment.tracks["position"] is not None:
				to_remove = []
				for pos in moment.tracks["position"]:
					#print("considering {}".format(pos))
					if pos in objects:
						#print("removing {}".format(pos))
						to_remove.append(pos)
				for pos in to_remove:
					moment.tracks["position"].remove(pos)

	def axiom_only_final_movement_destination_matters(self,moments):
		movement_started = False

		movement_moments = []
		for moment in moments:	
			if not movement_started and moment.tracks["movement"] == ["True"]:
				movement_started = True
				movement_moments.append(moment)
			elif movement_started and moment.tracks["movement"] != ["True"]:
				movement_started = False

				# process movement moments
				movement_moments.reverse()
				init_pos = movement_moments[0].tracks["position"]
				for mv in movement_moments:
					if mv.tracks["position"] != init_pos:
						if mv.tracks["position"] is not None:
							to_remove = []
							for item in mv.tracks["position"]:
								if "h1" not in item:
									to_remove.append(item) 
							for item in to_remove:
								mv.tracks["position"].remove(item)
							if len(mv.tracks["position"]) == 0:
								mv.tracks["position"] = None

				movement_moments = []
			elif movement_started:
				movement_moments.append(moment)