import time
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from figaro_msgs.msg import StringArray
from figaro_msgs.msg import Intent
from figaro_msgs.msg import IntentArray
from figaro_msgs.msg import Entity
from figaro_msgs.msg import Speech
from figaro_msgs.msg import SpeechArray
from figaro_msgs.msg import AudioContent
from figaro_msgs.msg import FigureAudioContent
from figaro_msgs.msg import DisplayContent

from audio_in.audio_interface import *
from editing.speech_command_interface import *

class Controller(Node):

	def __init__(self):
		'''
		setup publisher/subscribers for communicating with controller node
		'''
		super().__init__('audio_node')
		self.record_publisher = self.create_publisher(AudioContent, 'audio_record_pub_ctrl', 10)
		self.record_subscriber = self.create_subscription(String,'ctrl_pub_audio_record', self.record_subscriber_callback, 10)

		self.stream_publisher = self.create_publisher(String, 'audio_stream_pub_ctrl', 10)
		self.stream_subscriber = self.create_subscription(String, 'ctrl_pub_audio_stream', self.stream_subscriber_callback, 10)

		self.display_publisher = self.create_publisher(DisplayContent, 'audio_display_pub_ctrl', 10)
		self.highlight_agent_publisher = self.create_publisher(DisplayContent, 'audio_pub_ctrl_highlight_agent', 10)

		# passing/receiving intents, entities, and other speech data
		self.intent_publisher = self.create_publisher(StringArray, 'audio_intent_pub_ctrl', 10)

		# projector messages
		self.throwaway_speech_sub = self.create_subscription(String, 'ctrl_pub_audio_throwaway_speech', self.throwaway_speech_callback, 10)
		self.flag_speech_sub = self.create_subscription(String, 'ctrl_pub_audio_flag_speech', self.flag_speech_callback, 10)
		self.throwaway_speech_after_time = self.create_subscription(Float64, 'ctrl_pub_audio_rewind', self.rewind_audio_callback, 10)

		# for pausing and rewinding
		self.pause_sub = self.create_subscription(String, 'ctrl_pub_audio_pause', self.pause_audio_callback, 10)
		self.rewind_sub = self.create_subscription(Float64, 'ctrl_pub_audio_rewind', self.rewind_audio_callback, 10)

		# setup audio interfaces
		# special case if simulating on the robot
		deploy = None
		if len(sys.argv) > 2 and sys.argv[2] == "deploy":
			deploy = "deploy"
		self.record_interface = AudioInterface(self.display_publisher,self.highlight_agent_publisher,deploy)
		speech_intents = self.record_interface.get_speech_intents()
		#self.speech_command_interface = SpeechCommandInterface(self.publish_stream,self.record_interface.process_utterance, self.record_interface.check_pre_utterance)
		#self.speech_command_interface.begin_listening()

		# send message to ctrl node
		self.linker_pub = self.create_publisher(String, "node_link_ctrl", 10)
		name = String()
		name.data = "audio"
		print("sending ping to ctrl")
		self.linker_pub.publish(name)

		# receive ping from control node
		self.linker_sub = self.create_subscription(String, "ctrl_link_node", self.respond_to_ping, 10)

		# send intents to ctrl node
		intent_array = StringArray()
		for intent in speech_intents:
			intent_array.array.append(intent.display_name)
		self.intent_publisher.publish(intent_array)

	def rewind_audio_callback(self, msg):
		rtime = msg.data
		self.record_interface.remove_recordings_after_time(rtime)

	def pause_audio_callback(self, msg):
		data = msg.data
		if data == "pause":
			self.record_interface.pause()
		else:
			self.record_interface.resume()

	def record_subscriber_callback(self,msg):
		string = msg.data

		if string == "record":
			self.record_interface.begin_recording()
			#self.speech_command_interface.notify_recording()
		elif string == "end":
			#self.speech_command_interface.notify_not_recording()
			speech, actions, human_speech, human_actions = self.record_interface.end_recording()
			print(speech)

			robot_speech_msg = SpeechArray()
			for data in speech:
				indiv_speech_msg = Speech()
				indiv_speech_msg.text = data["relevant_text"]
				indiv_speech_msg.whole_text = data["whole_text"]
				indiv_speech_msg.speech = data["speech"]
				indiv_speech_msg.starttime = data["start"]
				indiv_speech_msg.endtime = data["end"]
				robot_speech_msg.array.append(indiv_speech_msg)

			robot_action_msg = IntentArray()
			for data in actions:
				indiv_action_msg = Intent()
				indiv_action_msg.text = data["relevant_text"]
				indiv_action_msg.whole_text = data["whole_text"]
				indiv_action_msg.intent = data["action"]
				indiv_action_msg.starttime = data["start"]
				indiv_action_msg.endtime = data["end"]
				robot_action_msg.array.append(indiv_action_msg)

			robot_msg = FigureAudioContent()
			robot_msg.figure = "robot"
			robot_msg.speech = robot_speech_msg
			robot_msg.actions = robot_action_msg

			human_speech_msg = SpeechArray()
			for data in human_speech:
				indiv_speech_msg = Speech()
				indiv_speech_msg.text = data["relevant_text"]
				indiv_speech_msg.whole_text = data["whole_text"]
				indiv_speech_msg.speech = data["speech"]
				indiv_speech_msg.starttime = data["start"]
				indiv_speech_msg.endtime = data["end"]
				human_speech_msg.array.append(indiv_speech_msg)

			human_action_msg = IntentArray()
			for data in human_actions:
				indiv_action_msg = Intent()
				indiv_action_msg.text = data["relevant_text"]
				indiv_action_msg.whole_text = data["whole_text"]
				indiv_action_msg.intent = data["action"]
				indiv_action_msg.starttime = data["start"]
				indiv_action_msg.endtime = data["end"]
				human_action_msg.array.append(indiv_action_msg)

			human_msg = FigureAudioContent()
			human_msg.figure = "h1"
			human_msg.speech = human_speech_msg
			human_msg.actions = human_action_msg

			msg = AudioContent()
			msg.figure_data.append(robot_msg)
			msg.figure_data.append(human_msg)
			self.record_publisher.publish(msg)

			'''
			new_data = self.record_interface.end_recording()

			msg = String()
			string_data = ""
			for data in new_data:
				string_data += data["speech"]
				string_data += "&&&"
				string_data += str(data["start"])
				string_data += "&&&"
				string_data += str(data["end"])
				string_data += "|||"
			msg.data = string_data
			self.record_publisher.publish(msg)
			'''
		'''
		string = msg.data

		if string == "record":
			self.record_interface.begin_recording()
		elif string == "end":
			new_data = self.record_interface.end_recording()

			msg = TimestampedSpeechArray()
			for data in new_data:
				msg_data = TimestampedSpeech()

				msg_data.speech = new_data["speech"]
				msg_data.start = new_data["start"]
				msg_data.end = new_data["end"]

				msg.array.append(msg_data)
			self.record_publisher.publish(msg)
		'''

	def stream_subscriber_callback(self,msg):
		pass

	def publish_stream(self,message):
		msg = String()
		msg.data = message
		self.stream_publisher.publish(msg)

	def throwaway_speech_callback(self, msg):
		speech = msg.data
		self.record_interface.throwaway_speech(speech)

	def flag_speech_callback(self, msg):
		speech = msg.data
		self.record_interface.flag_speech(speech)

	def respond_to_ping(self,message):
		print("responding to ping \"{}\"".format(message.data))
		name = String()
		name.data = "audio"
		self.linker_pub.publish(name)

def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)

if __name__ == '__main__':
    main()
