class RobotAxioms:

	def __init__(self):
		pass

	def process_position_movement(self, moments):
		for i in range(len(moments)):
			moment = moments[i]

			for human in ["h1"]:
				if moment.tracks["position"] is not None:
					for item in moment.tracks["position"]:
						if human in item:
							moment.tracks["close_to_human"] = True
							#if (i > 0 and moments[i-1].tracks["close_to_human"] is None) or i == 0:
							#	moment.tracks["close_to_human"] = item
							#else:
							#	moment.tracks["close_to_human"] = moments[i-1].tracks["close_to_human"]
							#moment.tracks["position"].remove(item)

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

		for i in range(len(moments)-1):
			cur = moments[i]
			nex = moments[i+1]

			if cur.tracks["close_to_human"] and not nex.tracks["close_to_human"]:
				nex.tracks["h1_not_near_rob"] = True

	def remove_unrecognizable_objects_or_regions(self, moments, objects, regions):
		for moment in moments:
			if moment.tracks["position"] is not None:
				to_remove = []
				for pos in moment.tracks["position"]:
					if pos in objects:
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

	def axiom_ultra_continuous_movement(self, moments, modalities):
		'''
		AXIOM: Continuous Movement
		Movement with < gapval second gaps must be treated as a single instance of movement
		'''
		# classify different modalities
		movement_modality_list = []
		for mod in modalities:
			if "movement" in mod:
				movement_modality_list.append(mod)

		gapval = 2

		# set of indexes for each movement modality that must be converted to "true"
		idxs_to_fill_in = {}
		for mod in movement_modality_list:
			idxs_to_fill_in[mod] = []

		# keep a list of non-movement indixes since the last movement was sensed
		idx_buffer = {}
		for mod in movement_modality_list:
			idx_buffer[mod] = []

		# keep track of how many seconds have passed since a movement was sensed
		movement_happened = {}
		for mod in movement_modality_list:
			movement_happened[mod] = -1

		for i in range(len(moments)):
			moment = moments[i]
			for mod in movement_modality_list:
				val = moment.tracks[mod]
				if val is not None and len(val) == 1 and val[0] == "True":					
					print("val is true, movement happened {}".format(movement_happened[mod]))
					curr_time = moment.start_time
					# first, determine if the time between now and the last movement sensed is above gapval
					# if so, dump the non-movement buffer into the set of indices to fill in
					if curr_time - movement_happened[mod] <= 0.5:
						print("filling in the following times for {}: {}".format(mod, idx_buffer[mod]))
						idxs_to_fill_in[mod].extend(idx_buffer[mod])

					# take the time of the movement
					movement_happened[mod] = curr_time

					# clear the non-movement indices
					idx_buffer[mod].clear()

				else: # val is False or None

					if movement_happened[mod] != -1:
					# store an updated time for how long it has been since movement happened
						idx_buffer[mod].append(i)

		# fill in the gaps of non-movement less than gapval
		for mod,idxs in idxs_to_fill_in.items():
			for idx in idxs:
				moments[idx].tracks[mod] = ["True"]