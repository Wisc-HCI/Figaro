import math
import copy

class RobotAxioms:

	def __init__(self):
		pass

	def convert_strings_to_integers(self,moments):
		for moment in moments:
			moment.tracks["xpos"] = int(moment.tracks["xpos"]) if moment.tracks["xpos"] is not None else None
			moment.tracks["ypos"] = int(moment.tracks["ypos"]) if moment.tracks["ypos"] is not None else None
			moment.tracks["h_xpos"] = int(moment.tracks["h_xpos"]) if moment.tracks["h_xpos"] is not None else None
			moment.tracks["h_ypos"] = int(moment.tracks["h_ypos"]) if moment.tracks["h_ypos"] is not None else None
			moment.tracks["rotation"] = int(moment.tracks["rotation"]) if moment.tracks["rotation"] is not None else None

	def convert_degrees_to_radians(self,moments):
		pass

	def remove_absent_info(self,moments):
		i = 0
		while i < len(moments):
			moment = moments[i]
			tr = moment.tracks
			if tr["xpos"] is None or tr["ypos"] is None or tr["rotation"] is None:
				del moments[i]
			else:
				i += 1

	def remove_minute_movement(self,moments):
		for i in range(len(moments)):

			start_moment = moments[i]
			start_xpos = start_moment.tracks["xpos"]
			start_ypos = start_moment.tracks["ypos"]
			for j in range(i+1,len(moments)):
				curr_xpos = moments[j].tracks["xpos"]

				x_diff = abs(curr_xpos - start_xpos)
				if x_diff < 3:
					moments[j].tracks["xpos"] = start_xpos
				else:
					moments[j].tracks["movement_x"] = str(moments[j].tracks["xpos"])
					break

			for j in range(i+1,len(moments)):
				curr_ypos = moments[j].tracks["ypos"]

				y_diff = abs(curr_ypos - start_ypos)
				if y_diff < 3:
					moments[j].tracks["ypos"] = start_ypos
				else:
					moments[j].tracks["movement_y"] = str(moments[j].tracks["ypos"])
					break


	def remove_minute_angle_changes(self,moments):
		for i in range(len(moments)):

			start_moment = moments[i]
			start_rot = start_moment.tracks["rotation"]
			for j in range(i+1,len(moments)):
				curr_rot = moments[j].tracks["rotation"]

				diff = abs(curr_rot - start_rot)
				if diff < 15:
					moments[j].tracks["rotation"] = start_rot
				else:
					moments[j].tracks["movement_angle"] = str(moments[j].tracks["rotation"])
					break

	def interpolate_helper(self,moments,i,movement,pos):
		moment = moments[i]
		if moment.tracks[movement] is not None:
			curr_val = int(moment.tracks[movement])

			next_movement_idx = i + 1
			for j in range(i+1,i+5):
				if moments[j].tracks[movement] is not None:
					next_movement_idx = j
					break

			if next_movement_idx > i+1:
				next_val = int(moments[next_movement_idx].tracks[movement])
			else:
				return

			for j in range(i+1,next_movement_idx):
				to_fill = moments[j]
				to_fill.tracks[movement] = int(round(curr_val + (next_val - curr_val)*1.0*((j-i)*1.0/(next_movement_idx-i))))
				to_fill.tracks[pos] = int(round(curr_val + (next_val - curr_val)*1.0*((j-i)*1.0/(next_movement_idx-i))))


	def interpolate(self,moments):
		for i in range(len(moments)):

			# x
			self.interpolate_helper(moments,i,"movement_x","xpos")

			# y
			self.interpolate_helper(moments,i,"movement_y","ypos")

			# angle
			self.interpolate_helper(moments,i,"movement_angle","rotation")

			# now assemble the time
			tracks = moments[i].tracks
			if tracks["movement_x"] is not None or tracks["movement_y"] is not None or tracks["movement_angle"] is not None:
				tracks["movement_time"] = str(0.1)

	def add_moment_time_helper(self, rev_moments, mod, time_mod):
		i = 0
		while i < len(rev_moments):
			moment = rev_moments[i]

			# handle x
			if moment.tracks[mod] is not None:
				to_move_idx = i
				for j in range(i+1,i+5):
					if j >= len(rev_moments):
						break
					past_mom = rev_moments[j]
					if past_mom.tracks[mod] is None:
						to_move_idx = j
					else:
						break

				curr_move_status = moment.tracks[mod]
				moment.tracks[mod] = None
				rev_moments[to_move_idx].tracks[mod] = curr_move_status
				rev_moments[to_move_idx].tracks[time_mod] = 0.1*((to_move_idx+1)-i)

				i = to_move_idx + 1
				continue

			i += 1

	def add_moment_time(self,moments):
		for moment in moments:
			moment.tracks["movement_time_x"] = None
		rev_moments = moments.copy()
		rev_moments.reverse()
		
		# handle x
		self.add_moment_time_helper(rev_moments,"movement_x","movement_time_x")

		# handle y
		self.add_moment_time_helper(rev_moments,"movement_y","movement_time_y")

		# handle rotation
		self.add_moment_time_helper(rev_moments,"movement_angle","movement_time_angle")

	def interpolate_old(self,moments):
		for i in range(len(moments)):
			moment = moments[i]

			if moment.tracks["movement_time_x"] is not None and moment.tracks["movement_time_x"] > 0.1:
				curr_position = i
				curr_val = int(moment.tracks["movement_x"])
				lookahead_steps = int(round(moment.tracks["movement_time_x"]*10))
				lookahead_position = i + lookahead_steps
				next_val = int(moments[lookahead_position].tracks["movement_x"])

				for j in range(curr_position, lookahead_position):
					moment_to_modify = moments[j]
					modify_val = curr_val + ((next_val - curr_val)*1.0*(j/(lookahead_position-curr_position)))
					moment.tracks["movement_x"] = modify_val
					moment.tracks["movement_time_x"] = 0.1

			if moment.tracks["movement_time_y"] is not None and moment.tracks["movement_time_y"] > 0.1:
				pass

			if moment.tracks["movement_time_angle"] is not None and moment.tracks["movement_time_angle"] > 0.1:
				pass

	def convert_to_text(self,moments):
		for moment in moments:
			for track in moment.tracks:
				if moment.tracks[track] is not None:
					moment.tracks[track] = str(moment.tracks[track])