import io
import os
import json

# Imports the Google Cloud client library
from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
from google.oauth2 import service_account

class STT:

	def __init__(self):
		pass

	def interpret(self, filename):
		# Instantiates a client
		dialogflow_key = json.load(open('speech_key.json'))
		credentials = (service_account.Credentials.from_service_account_info(dialogflow_key))
		client = speech.SpeechClient(credentials=credentials)

		# The name of the audio file to transcribe
		file_name = os.path.join(
			os.path.dirname(__file__),
			'resources',
			'audio.raw')

		# Loads the audio into memory
		with io.open(filename, 'rb') as audio_file:
			content = audio_file.read()
			audio = types.RecognitionAudio(content=content)

		config = types.RecognitionConfig(
			encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
			sample_rate_hertz=44100,
			language_code='en-US',
			enable_word_time_offsets=True)

		# Detects speech in the audio file
		try:
			response = client.recognize(config, audio)
			return response.results
		except:
			print("Audio too long -- using long-running audio API call")
			return []
