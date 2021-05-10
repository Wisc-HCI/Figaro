import json
import language_check
import random

class ExampleCreator:

	def __init__(self):
		try:
			file = open("intent_training_examples.json")
			self.intent_data = json.load(file)

			prefix_file = open("prefixes.json")
			self.prefix_data = json.load(prefix_file)
		except:
			print("ERROR: could not open intent training example file")
			exit(1)

	def create_pos_examples(self):
		pos_examples = []
		tool = language_check.LanguageTool('en-US')

		'''
		load a list of intent examples
		'''
		speech = []
		raw_examples = self.intent_data["rasa_nlu_data"]["common_examples"]
		for example in raw_examples:
			speech.append(example["text"])

		'''
		load the prefixes
		'''
		robot_speech_prefixes = []
		human_speech_prefixes = []
		raw_prefixes = self.prefix_data["rasa_nlu_data"]["common_examples"]
		for prefix in raw_prefixes:
			if prefix["intent"] == "robot_say_prefix":
				robot_speech_prefixes.append(prefix["text"])
			elif prefix["intent"] == "human_say_prefix":
				human_speech_prefixes.append(prefix["text"])

		for example in speech:
			'''
			assemble the example
			'''
			for subject in ["human","robot"]:
				prefixes = robot_speech_prefixes if subject == "robot" else human_speech_prefixes
				prefix = random.choice(prefixes)
				start_idx = len(prefix) + 1
				end_idx = start_idx + len(example)
				#suffix = random.choice(self.suffixes)
				whole_example = "{} {}".format(prefix,example)

				'''
				check the example for grammar
				'''
				matches = tool.check(whole_example)
				is_correct = True
				for match in matches:
					if match.ruleId != "UPPERCASE_SENTENCE_START" and match.ruleId != "MORFOLOGIK_RULE_EN_US":
						 is_correct = False
						 break
				if not is_correct:
					continue

				'''
				add the labels to the example
				'''
				# uncomment for tagging distinguishable parts of speech (e.g. robot v human, speech v actions)
				'''
				if subject == "robot":
					labeled_example_temp = whole_example[:end_idx] + " >" + whole_example[end_idx+1:]
					labeled_example = labeled_example_temp[:start_idx] + "< " + labeled_example_temp[start_idx:]
					labeled_example = labeled_example.strip()
				elif subject == "human":
					labeled_example_temp = whole_example[:end_idx] + " >>>" + whole_example[end_idx+1:]
					labeled_example = labeled_example_temp[:start_idx] + "<<< " + labeled_example_temp[start_idx:]
					labeled_example = labeled_example.strip()
				'''
				labeled_example_temp = whole_example[:end_idx] + " >" + whole_example[end_idx+1:]
				labeled_example = labeled_example_temp[:start_idx] + "< " + labeled_example_temp[start_idx:]
				labeled_example = labeled_example.strip()
				pos_examples.append(labeled_example)

		'''
		write the examples
		'''
		with open("pos_examples.txt","w") as outfile:
			for ex in pos_examples:
				outfile.write("{}\n".format(ex))

if __name__=="__main__":

	ec = ExampleCreator()
	ec.create_pos_examples()