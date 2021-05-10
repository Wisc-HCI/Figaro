from copy import copy

class SpeechDatabase:

	def __init__(self):

		'''
		Start with an empty list of intents
		'''
		self.learned_intents = []
		self.provided_intents = {}

	def get_all_intents(self):
		all_intents = copy(self.learned_intents)
		all_intents.extend(list(self.provided_intents.keys()))
		return all_intents

class ProvidedIntent:

	def __init__(self,name,speech):
		self.name = name
		self.speech = speech