class SpeechUtils:

	def __init__(self):
		self.categories = []

	def discretize_speech(self,speech_dicts,input_modalities):
		examples = {}

		counter = 0
		for speech in speech_dicts:
			if speech["speech"] == "UNKNOWN":
				input_modalities["speech"]["vals"].append("cat{}".format(counter))
				counter += 1
				examples[speech["speech"]] = "cat{}".format(counter)
			else:
				examples[speech["speech"]] = speech["speech"]

		cat_dicts = []
		for item in speech_dicts:
			cat_dicts.append({"cat":examples[item["speech"]],"start":item["start"],"end":item["end"]})

		return cat_dicts

