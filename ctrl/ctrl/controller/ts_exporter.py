import os
import json

class TSExporter:

	def __init__(self):
		self.cwd = os.getcwd()

	def export_links_json(self,sketch):
		'''
		Export links: the source, source name, target, and input
		Export states: {"id": state.id, "name": state.name, "gesture": None, "reachble": True, "final": False, "prob": 0.0, "micro": state.micros[0]["name"]}
		'''
		print("exporting json links")
		json_array = {}
		json_array["links"] = []
		state2name = {}
		print("exporting transitions")
		for transition in sketch.transitions:
			# setup the input
			inp = ""
			for lab in transition.transition_labels:
				inp += "{}\n".format(lab)
			inp = inp.strip()

			# setup the state names
			state2name[transition.source] = self.make_state_name(transition.source)
			state2name[transition.target] = self.make_state_name(transition.target)
			link = {"source":transition.source.id,"source_name":state2name[transition.source],"target":transition.target.id,"input":inp}
			json_array["links"].append(link)

		# in case any states did not have their names created
		for state in sketch.states:
			if state not in state2name:
				state2name[state] = self.make_state_name(state)

		print("exporting states")
		json_array["states"] = []
		for state in sketch.states:
			state_dict = {"id": state.id, "name": state2name[state], "gesture": None, "reachble": True, "final": False, "prob": 0.0, "micro": "None"}
			json_array["states"].append(state_dict)

		with open("{}/d3js/links.json".format(self.cwd),"w") as outfile:
			json.dump(json_array,outfile)

	def make_state_name(self, state):
		name = ""
		for prop,val in state.modalities.propositions.items():
			name += "{} - {}\n".format(prop,val)
		name = name.strip()
		return name