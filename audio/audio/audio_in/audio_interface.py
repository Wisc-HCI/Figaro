#!/usr/bin/env python3

#from audio_in.audio_recorder import *
#from audio_in.stt import *
'''
Mycroft wake word necessary modules
'''
import sys
mycroft_path = sys.argv[1]
sys.path.append(mycroft_path)
sys.path.append("{}/runner".format(mycroft_path))
from precise.util import activate_notify
from precise_runner import PreciseRunner, PreciseEngine
try:
	from nlu.nlu_main import *
	from figaro_msgs.msg import DisplayContent
except:
	sys.path.append("/home/david/Documents/Figaro/audio/audio/")
	from nlu.nlu_main import *
from audio_in.single_stream_stt import *
from audio_in.deploy_single_stream_stt import *

import threading
import time
import threading

class AudioInterface:

	def __init__(self, display_publisher, highlight_publisher,deploy):
		#self.audio_recorder = AudioRecorder()
		#self.audio_recorder.set_path("audio_in/audio_bin/output.wav")
		#self.stt = STT()
		self.single_stream_stt = SingleStreamSTT()
		self.content_id = 0   # keeping track of unfinished content that belongs to the same finished content

		self.nlu = NLU()

		self.recorded_utterances = []
		self.is_recording = False
		self.display_publisher = display_publisher
		self.highlight_publisher = highlight_publisher

		self.paused = False

		'''
		Begin wake word listeners
			- robot say
			- robot do
			- human say
			- human do
		'''
		# semaphore for the stream callback
		self.stream_lock = threading.Lock()
		if deploy is None:
			# robot says
			thread = threading.Thread(target=self.listen_for_wake_words, args=("audio_in/wake_word_engines/robotsay.pb","robotsay",0.5))
			thread.daemon = True							# Daemonize thread
			thread.start()

			# human says
			thread = threading.Thread(target=self.listen_for_wake_words, args=("audio_in/wake_word_engines/humansay.pb","humansay",1))
			thread.daemon = True							# Daemonize thread
			thread.start()
			
		else:
			# hey temi

			import socket
			s = socket.socket()
			IP=""
			PORT=7779
			s.bind(('', PORT))         
			print("socket bound to {}:{}".format(IP,PORT))
			s.listen()

			c, addr = s.accept()
			self.deploy_stt = DeploySTT(c)

			thread = threading.Thread(target=self.listen_for_deploy_wakewords, args=("audio_in/wake_word_engines/heyrobot.pb",0.5))
			thread.daemon = True							# Daemonize thread
			thread.start()
			while True:
				pass


	def pause(self):
		self.paused = True

	def resume(self):
		self.paused = False

	def get_speech_intents(self):
		return self.nlu.get_speech_intents()

	def begin_recording(self):
		'''
		if nlu_initialized:
			thread = threading.Thread(target=self.audio_recorder.run)
			thread.daemon = True							# Daemonize thread
			thread.start()
		'''
		self.is_recording = True
		self.recording_start_time = time.time()
		self.recorded_utterances = []

	def process_utterance(self, text, timestamp_estimate):
		# do something here
		pass

	def flag_speech(self, speech):

		if not self.is_recording:
			print("not recording, so nothing to delete")
			return

		print("attempting to delete {}".format(speech))

		if len(self.recorded_utterances) < 1:
			print("ERROR: attempting to delete speech when there is none to delete")
			exit()

		print(self.recorded_utterances[-1][1])

		if self.recorded_utterances[-1][1] != speech:
			print("ERROR: speech to delete does not match the most recent speech added")
			exit()

		old_tup = self.recorded_utterances[-1]
		new_tup = (old_tup[0], old_tup[1], "UNDEFINED", old_tup[3], old_tup[4], old_tup[5])
		del self.recorded_utterances[-1]
		self.recorded_utterances.append(new_tup)
		print("successfully flagged speech")

	def throwaway_speech(self, speech):

		if not self.is_recording:
			print("not recording, so nothing to delete")
			return

		print("attempting to delete {}".format(speech))

		if len(self.recorded_utterances) < 1:
			print("ERROR: attempting to delete speech when there is none to delete")

		print(self.recorded_utterances[-1][1])

		if self.recorded_utterances[-1][1] != speech:
			print("ERROR: speech to delete does not match the most recent speech added")
			exit()

		del self.recorded_utterances[-1]
		print("successfully deleted speech")
		
	def end_recording(self):
		self.is_recording = False

		'''
		Can mayyybeee cross check the recorded utterances against the recorded ones
		and the additionally streamed ones just in case...
		'''

		robot_speech_dict = []
		robot_actions_dict = []
		human_speech_dict = []
		human_actions_dict = []
		for item in self.recorded_utterances:
			if item[5] == "robotsay":
				robot_speech_dict.append({"whole_text": item[0],
										  "relevant_text": item[1],
										  "speech":item[2], 
										  "start":item[3], 
										  "end":item[4]})
			elif item[5] == "humansay":
				human_speech_dict.append({"whole_text": item[0],
										  "relevant_text": item[1],
										  "speech":item[2], 
										  "start":item[3], 
										  "end":item[4]})
			elif item[5] == "humando":
				human_actions_dict.append({"whole_text": item[0],
										  "relevant_text": item[1],
										  "action":item[2], 
										  "start":item[3], 
										  "end":item[4]})
			elif item[5] == "robotdo":
				robot_actions_dict.append({"whole_text": item[0],
										  "relevant_text": item[1],
										  "action":item[2], 
										  "start":item[3], 
										  "end":item[4]})

		# return arrays of speech and actions
		print("returning {}".format(robot_speech_dict))
		print("returning {}".format(human_speech_dict))
		return robot_speech_dict, robot_actions_dict, human_speech_dict, human_actions_dict

	def remove_recordings_after_time(self, rtime):
		to_remove = []
		for tup in self.recorded_utterances:
			if tup[3] > rtime:
				to_remove.append(tup)
		for tup in to_remove:
			self.recorded_utterances.remove(tup)

	def process_streamed_utterance(self, text, start_time, end_time, category):

		# TODO: the need to get whole_text as it should be unnecessary
		whole_text = text
		if category == "robotsay":
			whole_text = "the robot says " + whole_text
			robot_speech,_,_,_=self.nlu.extract_speech_from_keywords(whole_text)
			intent = robot_speech[0][1]
		elif category == "humansay":
			whole_text = "the human says " + whole_text
			_,_,human_speech,_=self.nlu.extract_speech_from_keywords(whole_text)
			intent = human_speech[0][1]
		elif category == "robotdo":
			whole_text = "the robot does " + whole_text
			_,robot_actions,_,_=self.nlu.extract_speech_from_keywords(whole_text)
			intent = robot_actions[0][1]
		else:
			whole_text = "the human does " + whole_text
			_,_,_,human_actions=self.nlu.extract_speech_from_keywords(whole_text)
			intent = human_actions[0][1]

		# package the results
		tup = (whole_text, text, intent, start_time, end_time, category)
		self.recorded_utterances.append(tup)
		return tup

	def detected_text_callback(self, activation_time, text, activation_notifier):

		if self.is_recording  and not self.paused:
			end_time = time.time() - self.recording_start_time
			start_time = activation_time - self.recording_start_time
			info_tup = self.process_streamed_utterance(text,start_time,end_time,activation_notifier)
			intent = info_tup[2]
		else:
			if "say" in activation_notifier:
				intent = self.nlu.query_speech_intent(text)["name"]
			else:
				intent = self.nlu.query_action_intent(text)["name"]

		if intent is None or intent == "":
			intent = "UNDEFINED"

		self.send_text_to_interface(text, intent, activation_notifier)

	def send_text_to_interface(self, text, text_category, activation_notifier, in_progress=False):
		if activation_notifier == "robotsay":
			agent = "robot"
			typ = "speech"
		elif activation_notifier == "humansay":
			agent = "human"
			typ = "speech"
		elif activation_notifier == "robotdo":
			agent = "robot"
			typ = "actions"
		else:
			agent = "human"
			typ = "actions"

		# end the display of any previous text, display it with the new text
		msg = DisplayContent()
		msg.agent = agent
		msg.type = typ
		msg.content = text
		msg.content_category = text_category if text_category is not None else ""
		msg.unfinished = in_progress
		msg.content_id = self.content_id
		self.display_publisher.publish(msg)

	def start_stream_callback(self, receive_single_stream, receive_single_in_progress_stream, receive_agent_highlight, activation_time):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal, args=(receive_single_stream,receive_single_in_progress_stream,receive_agent_highlight,activation_time,self.content_id,self.stream_lock))
		thread.daemon = True							# Daemonize thread
		thread.start()

	def deploy_stream_callback(self):
		thread = threading.Thread(target=self.deploy_stt.run_terminal,args=(self.stream_lock,))
		thread.daemon = True							# Daemonize thread
		thread.start()

	def toggle_agent_highlight(self,activation_notifier,content_id):
		if activation_notifier == "robotsay":
			agent = "robot"
			typ = "say"
		elif activation_notifier == "humansay":
			agent = "human"
			typ = "say"
		elif activation_notifier == "robotdo":
			agent = "robot"
			typ = "do"
		else:
			agent = "human"
			typ = "do"
		# end the display of any previous text, display it with the new text
		msg = DisplayContent()
		msg.agent = agent
		msg.type = typ
		msg.content_id = content_id
		self.highlight_publisher.publish(msg)

	def listen_for_wake_words(self, word_library, activation_notifier, startdelay):
		def on_prediction(prob):
			pass

		def receive_single_stream(text,activation_time):
			print("text {}: {}".format(activation_time, text))
			self.detected_text_callback(activation_time, text, activation_notifier)

		def receive_single_in_progress_stream(text):
			self.send_text_to_interface(text,None,activation_notifier, in_progress=True)

		def receive_agent_highlight(content_id):
			self.toggle_agent_highlight(activation_notifier,content_id)

		def on_activation():
			self.content_id += 1
			print("ACTIVATION: {}".format(activation_notifier))
			activation_time = time.time()
			self.start_stream_callback(receive_single_stream, receive_single_in_progress_stream, receive_agent_highlight, activation_time)
			#self.detected_text_callback(activation_time, text, activation_notifier)
			#activate_notify()

		path = sys.argv[1]

		time.sleep(startdelay)

		engine = PreciseEngine('{}/.venv/bin/precise-engine'.format(path), word_library)
		PreciseRunner(engine, on_prediction=on_prediction, on_activation=on_activation,
					  trigger_level=0).start()
		#Event().wait()  # Wait forever

	def listen_for_deploy_wakewords(self,word_library, startdelay):
		def on_prediction(prob):
			pass

		def on_activation():
			self.content_id += 1
			print("deploy activated")
			activation_time = time.time()
			self.deploy_stream_callback()
			#self.detected_text_callback(activation_time, text, activation_notifier)
			#activate_notify()

		path = sys.argv[1]

		time.sleep(startdelay)

		engine = PreciseEngine('{}/.venv/bin/precise-engine'.format(path), word_library)
		PreciseRunner(engine, on_prediction=on_prediction, on_activation=on_activation,
					  trigger_level=0).start()

if __name__=="__main__":
	interface = AudioInterface(None,None,"deploy")