class SessionSignalProcessor:

	def __init__(self):
		pass

	def process_position_movement(self, moments):
		for moment in moments:

			for human in ["h1"]:
				if moment.tracks["position"] is not None:
					for item in moment.tracks["position"]:
						if human in item:
							moment.tracks["close_to_human"] = True
							moment.tracks["position"].remove(item)
			# combine robot position and movement
			if moment.tracks["movement"] == ["True"]:
				moment.tracks["position"] = ["movement"]

			# combine human position and movement
			# TODO: remove this
			for human in ["h1"]:
				if moment.tracks["{}_position".format(human)] is not None and any("robot" in string for string in moment.tracks["{}_position".format(human)]):
					moment.tracks["{}_near_rob".format(human)] = True
				moment.tracks["{}_movement".format(human)] = None
				moment.tracks["{}_position".format(human)] = None