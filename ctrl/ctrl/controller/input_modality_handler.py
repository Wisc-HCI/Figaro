import json

class InputModalityHandler:

	def __init__(self,load_folder):
		self.input_modalities = {}
		self.load_input_modalities(load_folder)

	def load_input_modalities(self, load_folder):
		try:
			infile = open("{}/in_modes.json".format(load_folder), "r")
			modalities = json.load(infile)
			for databit in modalities:
				self.set_input_modality(databit["modality"],databit["type"],databit["array"],databit["behavior_or_state"])

		except:
			print("ERROR: loading input modalities")
			exit(1)
		return self.input_modalities

	def set_input_modality(self, modality, typ, array, behavior_or_state):
		self.input_modalities[modality] = {"type": typ, "array": array, "behavior_or_state": behavior_or_state}

	def get_input_modality_dict(self):
		return self.input_modalities