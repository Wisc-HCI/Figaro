import sys

import os
import numpy as np
import joblib
import json
import re
from copy import copy

# google cloud stuff (dialogflow)
from google.oauth2 import service_account
import dialogflow_v2 as dialogflow

class NLU:

	def __init__(self):

		# get the working directory that nlu and audio_in sit inside of
		self.audio_dir = os.getcwd()

		# initialize speech and actions clients
		# SPEECH dialogflow initialization
		dialogflow_key = json.load(open('speech_key.json'))
		credentials = (service_account.Credentials.from_service_account_info(dialogflow_key))
		self.speech_d_client = dialogflow.SessionsClient(credentials=credentials)
		#self.speech_d_client = dialogflow.SessionsClient()
		self.speech_session = self.speech_d_client.session_path("verification-of-hri", "1")

		# initialize the intents clients (to access and modify intents remotely)
		self.speech_intents_client = dialogflow.IntentsClient(credentials=credentials)
		self.speech_intents_parent = self.speech_intents_client.project_agent_path("verification-of-hri")

		# ACTIONS dialogflow initialization
		dialogflow_key = json.load(open('actions_key.json'))
		credentials = (service_account.Credentials.from_service_account_info(dialogflow_key))
		self.action_d_client = dialogflow.SessionsClient(credentials=credentials)
		self.action_session = self.action_d_client.session_path("figaro-qwjpnp", "1")

	def get_speech_intents(self):
		return self.speech_intents_client.list_intents(self.speech_intents_parent)

	def get_wake_phrases(self,text,wake_phrases_idx):
		# the components of a wake phrase
		article = ["the","this"]
		robot_noun = ["robot","bot"]
		human_noun = ["human","human being","person","dude","guy","man","woman","kid","child","individual","fellow"]
		says = ["says", "responds", "announces"]
		does = ["does"]
		
		# find wake phrases
		for i in range(len(text)):
			word = text[i]
			word_idx = i
			phrase = word
			speech = False
			action = False

			# found a wake phrase
			if word in robot_noun or word in human_noun:

				# determine who the subject is
				if word in robot_noun:
					robot_or_human = "robot"
				elif word in human_noun:
					robot_or_human = "human"

				phrase = word

				# there are leading words
				# while i is not an adverb/adjective, until we reach an article or something that is not an adverb/adjective
				j = i-1
				if j >= 0:
					# if article
					if text[j] in article:
						word_idx = j
						phrase = text[j] + " " + phrase

				# there is a say or a does
				j=i+1
				if j < len(text):
					if text[j] in says:
						phrase = phrase + " " + text[j]
						speech = True
					elif text[j] in does:
						phrase = phrase + " " + text[j]
						action = True
				print(phrase)
				if speech or action:
					if word_idx + len(phrase.split()) < len(text):
						wake_phrases_idx[word_idx] = {"phrase":"{}".format(phrase),"robot_or_human":robot_or_human, "speech":speech}

	def extract_speech_from_keywords(self,text):
		'''
		step 0: remove all special characters from text
		'''
		text = re.sub('[^A-Za-z0-9 ]+', '', text)
		text=text.split()
		print(text)

		robot_speech = []
		robot_actions = []
		human_speech = []
		human_actions = []

		wake_phrases_idx = {}
		self.get_wake_phrases(text,wake_phrases_idx)

		if len(wake_phrases_idx) == 0:
			return [], [], [], []
		# array.append((pot_utt_str,utt_intent[pot_utt_str],start_idx,end_idx))

		# only extract the first wake word
		print(wake_phrases_idx)
		idx = min(list(wake_phrases_idx.keys()))
		wake_phrase = wake_phrases_idx[idx]
		phrase = wake_phrase["phrase"]
		robot_or_human = wake_phrase["robot_or_human"]
		speech = wake_phrase["speech"]
		wake_phrase_length = len(wake_phrases_idx[idx]["phrase"].split())
		idx += wake_phrase_length
		relevant_text = " ".join(text[idx:])

		# is it above a 0.7 threshold?
		if speech:
			parse_output = self.query_speech_intent(relevant_text)#self.speech_interpreter.parse(relevant_text)["intent"]
		else:
			parse_output = self.query_action_intent(relevant_text)#self.actions_interpreter.parse(relevant_text)["intent"]

		intent = parse_output["name"]
		confidence = float(parse_output["confidence"])

		# check if confidence is too low (only applies to speech)
		if confidence < 0.8 and speech:
			confidence = 1.0
			intent = "UNDEFINED"

		if intent is None or intent == "":
			intent = "UNDEFINED"

		start_idx = len(phrase.split()) - 1
		end_idx = len(text) - 1

		tup = (relevant_text, intent, start_idx, end_idx)
		if robot_or_human == "robot":
			if speech:
				robot_speech.append(tup)
			else:
				robot_actions.append(tup)
		else:
			if speech:
				human_speech.append(tup)
			else:
				human_actions.append(tup)

		return robot_speech, robot_actions, human_speech, human_actions

	def query_speech_intent(self,text):
		# otherwise we will return the intent
		text_input = dialogflow.types.TextInput(
			text=text, language_code='en-US')

		query_input = dialogflow.types.QueryInput(text=text_input)

		response = self.speech_d_client.detect_intent(
			session=self.speech_session, query_input=query_input)

		intent = response.query_result.intent.display_name
		confidence = response.query_result.intent_detection_confidence

		return {"name": intent,"confidence": confidence}

	def query_action_intent(self,text):
		text_input = dialogflow.types.TextInput(
			text=text, language_code='en-US')
		query_input = dialogflow.types.QueryInput(text=text_input)

		response = self.action_d_client.detect_intent(
			session=self.action_session, query_input=query_input)

		intent = response.query_result.intent.display_name
		confidence = response.query_result.intent_detection_confidence

		return {"name": intent,"confidence": confidence}