import time
import sys

class Axioms:

	def __init__(self, modality_info, design):
		self.modality_info = modality_info
		print("this is the original modality info")
		print(self.modality_info)
		self.modalities = list(modality_info.keys())

		# classify different modalities
		self.movement_modality_list = []
		for mod in self.modalities:
			if "movement" in mod:
				self.movement_modality_list.append(mod)

	def discard_non_modality_tracks(self, moments):
		tracks_to_discard = []
		print(self.modalities)
		for track in moments[0].tracks:
			if track not in self.modalities:
				tracks_to_discard.append(track)
		for moment in moments:
			for track in tracks_to_discard:
				del moment.tracks[track]

	def axiom_continuous_wait(self,moments):
		'''
		AXIOM: Continuous Movement
		Movement with < gapval second gaps must be treated as a single instance of movement
		'''
		gapval = 0.5

		# set of indexes for each movement modality that must be converted to "true"
		idxs_to_fill_in = []

		# keep a list of non-movement indixes since the last movement was sensed
		idx_buffer = []

		# keep track of how many seconds have passed since a movement was sensed
		wait_happened = -1

		for i in range(len(moments)):
			moment = moments[i]
			val = moment.tracks["wait"]
			print("searching for wait")
			if val == "True":
				print("\nWAIT\n")					
				#print("val is true, movement happened {}".format(movement_happened[mod]))
				curr_time = moment.start_time
				# first, determine if the time between now and the last movement sensed is above gapval
				# if so, dump the non-movement buffer into the set of indices to fill in
				if curr_time - wait_happened <= 0.5:
					print("filling in the following times for wait: {}".format(idx_buffer))
					idxs_to_fill_in.extend(idx_buffer)

				# take the time of the movement
				wait_happened = curr_time

				# clear the non-movement indices
				idx_buffer.clear()

			else: # val is False or None

				if wait_happened != -1:
				# store an updated time for how long it has been since movement happened
					idx_buffer.append(i)

		# fill in the gaps of non-movement less than gapval
		for idx in idxs_to_fill_in:
			print("filling wait")
			moments[idx].tracks["wait"] = "True"

	def axiom_continuous_movement(self, moments):
		'''
		AXIOM: Continuous Movement
		Movement with < gapval second gaps must be treated as a single instance of movement
		'''
		gapval = 0.5  # 2 for slow movement

		# set of indexes for each movement modality that must be converted to "true"
		idxs_to_fill_in = {}
		for mod in self.movement_modality_list:
			idxs_to_fill_in[mod] = []

		# keep a list of non-movement indixes since the last movement was sensed
		idx_buffer = {}
		for mod in self.movement_modality_list:
			idx_buffer[mod] = []

		# keep track of how many seconds have passed since a movement was sensed
		movement_happened = {}
		for mod in self.movement_modality_list:
			movement_happened[mod] = -1

		for i in range(len(moments)):
			moment = moments[i]
			for mod in self.movement_modality_list:
				val = moment.tracks[mod]
				if val is not None and len(val) == 1 and val[0] == "True":					
					#print("val is true, movement happened {}".format(movement_happened[mod]))
					curr_time = moment.start_time
					# first, determine if the time between now and the last movement sensed is above gapval
					# if so, dump the non-movement buffer into the set of indices to fill in
					if curr_time - movement_happened[mod] <= 0.5:  # 2.5 for slow movement
						#print("filling in the following times for {}: {}".format(mod, idx_buffer[mod]))
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

	def axiom_continuous_gesturing(self, moments):
		'''
		AXIOM: Continuous gesturing
		Movement with < gapval second gaps must be treated as a single instance of movement
		'''
		gapval = 0.2

		for mod in ["point","tilt","wait"]:
			# set of indexes for each gesture modality that must be converted to the current value
			idxs_to_fill_in = []

			# keep a list of non-matching indixes since the last movement was sensed
			idx_buffer = []

			# keep track of the current val
			curr_gesture_val = None

			# keep track of how many seconds have passed since a movement was sensed
			mod_happened = -1

			for i in range(len(moments)):
				moment = moments[i]
				val = moment.tracks[mod]

				# case 0: we JUST started. No gesture found yet
				if val is None and curr_gesture_val is None:
					pass

				# case 1: we are just starting and found a gesture
				elif val is not None and curr_gesture_val is None:
					curr_time = moment.start_time
					curr_gesture_val = val

					# take the time of the movement
					mod_happened = curr_time

				# case 2: none IN BETWEEN. This means we wait for gesture to re-occur
				elif val is None and curr_gesture_val is not None:
					idx_buffer.append((i,curr_gesture_val))

				# case 3: gesture occurs and it is the same gesture
				if val is not None and curr_gesture_val is not None and val == curr_gesture_val:
					curr_time = moment.start_time
					# first, determine if the time between now and the last movement sensed is above gapval
					# if so, dump the non-movement buffer into the set of indices to fill in
					if curr_time - mod_happened <= 0.5:
						#print("filling in the following times for {}: {}".format(mod, idx_buffer))
						idxs_to_fill_in.extend(idx_buffer)

					# discard everything
					idx_buffer.clear()

					# take the time of the movement
					mod_happened = curr_time

				# case 4: a different gesture occurs
				elif val is not None and curr_gesture_val is not None and val != curr_gesture_val:
					curr_time = moment.start_time
					# discard everything
					idx_buffer.clear()

					# set curr_gesture_val
					curr_gesture_val = val

					# take the time of the movement
					mod_happened = curr_time

				'''

				if val is not None and (curr_gesture_val is None or curr_gesture_val != val):
					curr_gesture_val = val

				if val is not None and val == curr_gesture_val:					
					print("val matches, gesture happened {}".format(mod_happened))
					curr_time = moment.start_time
					# first, determine if the time between now and the last movement sensed is above gapval
					# if so, dump the non-movement buffer into the set of indices to fill in
					if curr_time - mod_happened <= 0.5:
						print("filling in the following times for {}: {}".format(mod, idx_buffer))
						idxs_to_fill_in.extend(idx_buffer)

					# take the time of the movement
					mod_happened = curr_time

					# clear the non-movement indices
					idx_buffer.clear()

				else: # val is None

					if mod_happened != -1:
					# store an updated time for how long it has been since movement happened
						idx_buffer.append(i)
				'''

			# fill in the gaps of non-movement less than gapval
			for idx in idxs_to_fill_in:
				moments[idx[0]].tracks[mod] = idx[1]

	def helper_axiom_continuous_position(self,moments,position):
		gapval = 2
		i = 0
		while i < len(moments) - 2:
			curr_position = moments[i].tracks[position]
			next_position = moments[i+1].tracks[position]

			if curr_position != next_position:
				j = i+2
				fill_to = None
				while j < len(moments) and j < i+2+gapval:
					if moments[j].tracks[position] == curr_position:
						fill_to = j
						break
					j += 1

				if fill_to is not None:
					for k in range(i+1,j+1):
						moments[k].tracks[position] = curr_position

			i += 1

	def instantaneous_behaviors(self, moments):

		for i in range(len(moments)):

			curr = moments[i]
			if curr.tracks["wait"] == "True":
				j = i+1
				while j < len(moments):
					if moments[j].tracks["wait"] != "True":
						break
					moments[j].tracks["wait"] = None
					j += 1

			if curr.tracks["tilt"] is not None:
				j = i+1
				while j < len(moments):
					if moments[j].tracks["tilt"] is None:
						break
					moments[j].tracks["tilt"] = None
					j += 1

			if curr.tracks["point"] is not None:
				j = i+1
				while j < len(moments):
					if moments[j].tracks["point"] is None:
						break
					moments[j].tracks["point"] = None
					j += 1

			if curr.tracks["speech"] is not None:
				j = i+1
				while j < len(moments):
					if moments[j].tracks["speech"] is None:
						break
					moments[j].tracks["speech"] = None
					j += 1

	def axiom_continuous_position(self,moments):
		'''
		Prevent flickering that sometimes occurs with position
		'''

		self.helper_axiom_continuous_position(moments,"position")
		self.helper_axiom_continuous_position(moments,"h1_position")

	def axiom_constant_wait(self,moments):
		# cannot have waits under 1 second
		start_time = moments[0].start_time
		curr_wait = []
		in_wait = False
		for moment in moments:
			if moment.tracks["wait"] is not None and moment.tracks["wait"] == "True":
				if not in_wait:
					in_wait = True
					start_time = moment.start_time
				curr_wait.append(moment)
			else:
				if in_wait:
					in_wait = False
					if moment.start_time - start_time < 1:
						for non_wait in curr_wait:
							non_wait.tracks["wait"] = None
					curr_wait.clear()

	def axiom_only_tilt(self,moments):

		# get tilt idxs > 0.3 second
		# and also tilt idxs < that
		start_time = moments[0].start_time
		good_tilt_idxs = []
		bad_tilt_idxs = []
		in_tilt = False
		start = -1
		for i in range(len(moments)):
			for moment in moments:
				if moment.tracks["tilt"] is not None and moment.tracks["wait"] == "up" or moment.tracks["wait"] == "down":
					if not in_tilt:
						start = i
						in_tilt = True
						start_time = moment.start_time
				else:
					if in_tilt:
						in_tilt = False
						if moment.start_time - start_time < 0.3:
							bad_tilt_idxs.append((start,i))
						else:
							good_tilt_idxs.append((start,i))

		# for good tilts, clean up surroundings
		for interval in good_tilt_idxs:

			index = interval[0] - 1
			while index > 0:
				if moment[index].tracks["wait"] == "True":
					moment[index].tracks["wait"] = None
				else:
					break
				index -= 1

			index = interval[1]
			while index < len(moments):
				if moment[index].tracks["wait"] == "True":
					moment[index].tracks["wait"] = None
				else:
					break
				index += 1

		# is the tilt > 1 second? If so, keep. Otherwise, DISCARD EVERYTHING
		for interval in bad_tilt_idxs:

			for i in range(interval[0],interval[1]):
				moment[i].tracks["tilt"] = None


	def axiom_no_short_points(self, moments):
		print(moments[0].tracks)
		# cannot have points under 0.5
		# cannot have waits under 1 second
		start_time = moments[0].start_time
		curr_point = []
		curr_direction = None
		in_point = False

		# keep track of how many seconds have passed since a movement was sensed
		mod_happened = -1

		# idxs to remove
		idxs_to_remove = []

		# curr idxs
		curr_idxs = []

		for i in range(len(moments)):
			moment = moments[i]
			val = moment.tracks["point"]

			# case 0: we're just starting and point is None, and curr_direction is None
			if val is None and curr_direction is None:
				pass

			# case 1: we have found a point and curr_direction is None.
			elif val is not None and curr_direction is None:
				mod_happened = moment.start_time
				curr_direction = val
				curr_idxs.append(i)

			# case 2: we have found a point and the curr direction is NOT none. Need to check
			elif val is not None and curr_direction is not None:
				if curr_direction != val:
					# check if previous direction was long enough!
					curr_time = moment.start_time
					if curr_time - mod_happened < 0.5:
						idxs_to_remove.extend(curr_idxs)
					curr_idxs.clear()
					# reset time
					mod_happened = curr_time
				else:
					# continue incrementing time!
					curr_idxs.append(i)

				curr_direction = val

			# case 3: point is None and curr_direction is Not none. Need to check
			elif val is None and curr_direction is not None:
				# check if previous direction was long enough!
				curr_time = moment.start_time
				if curr_time - mod_happened < 0.5:
					idxs_to_remove.extend(curr_idxs)
				curr_idxs.clear()

				curr_direction = val

		for idx in idxs_to_remove:
			moments[idx].tracks["point"] = None

	def axiom_no_meaningless_movement(self, moments):
		curr_movement = []
		for moment in moments:
			if moment.tracks["movement"] is not None:
				curr_movement.append(moment)
			else:
				if len(curr_movement) > 0:  # we have a bout of movement
					if curr_movement[0].tracks["position"] != curr_movement[-1].tracks["position"]:
						curr_movement = []
						continue

					# the starting and ending position of the movement is the same!
					# this means we need to remove the movement
					destination = self.region_estimator()
					for movement_moment in curr_movement:
						movement_moment.tracks["movement"] = None
					curr_movement = []

	def region_estimator(self):
		pass

	def axiom_human_position_constant_between_movement(self, moments):
		self.helper_position_constant(moments,"h1_")
		#print("done with helper position constant for human")

	def axiom_robot_position_constant_between_movement(self, moments):
		self.helper_position_constant(moments,"")
		#print("done with helper position constant for robot")

	def helper_position_constant(self,moments,agent_ident):
		# set curr position
		curr_position = None
		for moment in moments:
			if moments[0].tracks["{}position".format(agent_ident)] is not None:
				curr_position = tuple(sorted(moments[0].tracks["{}position".format(agent_ident)]))
				break

		# set flag for is moving
		is_moving = True if moments[0].tracks["{}movement".format(agent_ident)] is not None else False

		i = 0
		while i < len(moments):
			moment = moments[i]
			if moment.tracks["{}movement".format(agent_ident)] == "False":
				if is_moving:

					# look ahead if necessary to find the curr position
					curr_position = ()
					j = i
					while j < len(moments) and moments[j].tracks["{}movement".format(agent_ident)] == "False":
						if moment.tracks["{}position".format(agent_ident)] is not None:		
							curr_position = tuple(sorted(moment.tracks["{}position".format(agent_ident)]))
							break
						j += 1
					is_moving = False
					i += 1
					continue

				# case 1: moment position is None
				if moment.tracks["{}position".format(agent_ident)] is None:
					moment.tracks["{}position".format(agent_ident)] == list(curr_position)

				# case 2: moment position is NOT none, but is unequal to the current position
				position = tuple(sorted(moment.tracks["{}position".format(agent_ident)]))
				if position != curr_position:
					moment.tracks["{}position".format(agent_ident)] == list(curr_position)
			else:
				is_moving = True

			i += 1

	def axiom_remove_redundancies(self, moments):
		'''
		Consolidate adjacent moments that are duplicates
		'''
		if len(moments) == 0:
			return
		moments_to_remove = []

		# search for what to remove
		curr_moment = moments[0]
		for moment in moments[1:]:
			tracks = moment.tracks
			curr_tracks = curr_moment.tracks

			is_same = True
			for mod in tracks:
				if tracks[mod] != curr_tracks[mod]:
					is_same = False
					break

			if is_same:
				moments_to_remove.append(moment)
			else:
				curr_moment = moment

		# perform the removal
		for moment in moments_to_remove:
			moments.remove(moment)

	def exclude_tracks(self, processed_moments):

		tracks_to_include = []
		'''
		Perform operations on all tracks
		'''

		'''
		Handle each track separately
		'''
		# handle position track
		curr_position = None
		curr_human_positions = {}
		for track in processed_moments[0].tracks:
			if "_position" in track:
				curr_human_positions[track[0:2]] = None
		curr_x = None
		curr_y = None
		for moment in processed_moments:
			if moment.tracks["position"] is not None:
				curr_position = moment.tracks["position"]
				curr_x = moment.x
				curr_y = moment.y
			for human in curr_human_positions:
				if moment.tracks["{}_position".format(human)] is not None:
					curr_human_positions[human] = moment.tracks["{}_position".format(human)]

			# determine if we can break
			if all(value is not None for value in curr_human_positions.values()) and curr_position != None:
				break

		for moment in processed_moments:
			if moment.x == -1:
				moment.x = curr_x
			else:
				curr_x = moment.x
			if moment.y == -1:
				moment.y = curr_y
			else:
				curr_y = moment.y
			for track in moment.tracks:

				# decide whether to include
				if moment.tracks[track] is not None:
					tracks_to_include.append(track)

				# handle the robot movement track
				if track == "movement":
					if moment.tracks[track] is None:
						moment.tracks[track] = []

				if track == "position":
					if moment.tracks[track] is None:
						moment.tracks[track] = []

				if track == "close_to_human":
					if moment.tracks[track] is None:
						moment.tracks[track] = False