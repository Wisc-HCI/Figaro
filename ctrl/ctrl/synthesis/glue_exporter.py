import xml.etree.ElementTree as ET
from xml.dom import minidom
import os
import re

class GlueExporter:

	def __init__(self):
		pass

	def clear(self):
		for filename in os.listdir("."):
			if filename.endswith(".xml") and "interaction" in filename: 
				print("clearing interaction files...")
				os.remove(filename)

	def export_speech(self, speech_db,filepath=""):
		speech_data = ET.Element('speech')

		provided_intents = speech_db.provided_intents
		for cat in provided_intents:
			cat_data = ET.SubElement(speech_data,'speech_cat')
			cat_data.set('id',cat)
			for phrase in provided_intents[cat].speech:
				phrase_data = ET.SubElement(cat_data,'phrase')
				phrase_data.set('text',phrase)

		# create a new XML file with the results
		compiled = ET.tostring(speech_data)
		xmlstr = minidom.parseString(compiled).toprettyxml(indent="   ")
		with open("{}speech.xml".format(filepath), "w") as f:
			f.write(xmlstr)

	def export_ts(self, glue, input_modalities, conditional_classifier, name="main",filepath=""):

		def standardize_arg(arg):
			arg = arg.lower()
			re.sub('[^A-Za-z]+', '', arg)
			return arg

		# whole interaction
		interaction_data = ET.Element('interaction_{}'.format(name))

		# states
		for state in glue.gstates:
			state_data = ET.SubElement(interaction_data, 'state')
			state_data.set('id',str(state.id))
			all_beh_data = ET.SubElement(state_data,'behaviors')
			for beh_cat,beh_val in state.behaviors.items():
				beh_data = ET.SubElement(all_beh_data,'behavior')
				beh_data.set('cat',beh_cat)

				if not input_modalities[beh_cat]["array"]:
					beh_val_data = ET.SubElement(beh_data,'value')

					# postprocess movement -- locations need to be lowercased, all special chars removed, etc
					if beh_cat == "movement" or beh_cat == "position":
						beh_val = standardize_arg(beh_val)

					beh_val_data.set('val',beh_val)
				else:
					for item in beh_val:

						beh_val_data = ET.SubElement(beh_data,'value')

						# postprocess movement -- locations need to be lowercased, all special chars removed, etc
						if beh_cat == "movement" or beh_cat == "position":
							item = standardize_arg(item)

						beh_val_data.set('val',item)

		# init
		init_data = ET.SubElement(interaction_data,'init')
		init_data.set('id',str(glue.initial_state.id))

		# transitions
		for trans in glue.gactions:
			source = trans.source.id
			target = trans.target.id
			action_labels = trans.action_label
			action_vals = trans.action_val

			trans_data = ET.SubElement(interaction_data,'transition')
			trans_data.set("source_id",str(source))
			trans_data.set("target_id",str(target))

			event_data = ET.SubElement(trans_data,'events')
			env_state_data = ET.SubElement(trans_data,'env_state')
			for i in range(len(action_labels)):
				label = action_labels[i]
				val = action_vals[i]
				
				classified_label = conditional_classifier(label)
				if classified_label == "event":
					parent = event_data
				elif classified_label == "env":
					parent = env_state_data
				else:
					print("ERROR: action label must either be environmental state or an event")
					exit()

				label_data = ET.SubElement(parent,'label')
				label_data.set('label',label)

				if label == "" or not input_modalities[label]["array"]:
					val_data = ET.SubElement(label_data,"val")
					val_data.set('val',str(val) if val is not None else "False")
				else:
					for item in val:
						val_data = ET.SubElement(label_data,"val")
						val_data.set('val',item)

		# create the file structure
		'''
		data = ET.Element('data')
		items = ET.SubElement(data, 'items')
		item1 = ET.SubElement(items, 'item')
		item2 = ET.SubElement(items, 'item')
		item1.set('name','item1')
		item2.set('name','item2')
		item1.text = 'item1abc'
		item2.text = 'item2abc'
		'''

		# create a new XML file with the results
		compiled = ET.tostring(interaction_data)
		xmlstr = minidom.parseString(compiled).toprettyxml(indent="   ")
		with open("{}interaction_{}.xml".format(filepath,name), "w") as f:
			f.write(xmlstr)
