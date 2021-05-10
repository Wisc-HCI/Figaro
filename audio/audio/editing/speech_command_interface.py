from editing.stt import *

import threading

class SpeechCommandInterface:

	def __init__(self, publisher_callback, process_text, process_pre_utterance):
		self.stt = STT()
		self.publisher = publisher_callback
		self.process_text = process_text
		self.process_pre_utterance = process_pre_utterance

	def begin_listening(self):
		thread = threading.Thread(target=self.stt.run_terminal, args=(self.receive_text,self.process_text,self.process_pre_utterance))
		thread.daemon = True							# Daemonize thread
		thread.start()

	def notify_recording(self):
		self.stt.begin_record()

	def notify_not_recording(self):
		self.stt.end_record()

	def receive_text(self,text):
		text = text.strip()
		if text[:7].lower() == "figaro " or text[:9].lower() == "computer " or text[:6] == "table ":
			#need to adjust subsequent text according to length of each wake word
			if text[:7].lower() == "figaro ":
				text = text[7:]
			elif text[:9].lower() == "computer ":
				text = text[9:]
			elif text[:6].lower() == "table ":
				text = text[6:]

			#limited voice commands we are currently using:
			#only to cancel or end the scene
			if text == "end scene":
				self.publisher("end_scene")
			elif text == "cancel scene":
				#self.command_hub.cancel()
				self.publisher("cancel")
			elif text == "cancel":
				#self.command_hub.cancel()
				self.publisher("cancel")

			'''
			leaving remaining speech commands for future:
			if text == "sketch":
				self.publisher("begin_sketch")
			elif text == "end sketch":
				self.publisher("end_sketch")
			elif text == "begin scene":
				self.publisher("begin_scene")
			elif text == "end scene":
				self.publisher("end_scene")
			#Editing management commands
			#Cancel commands that have multiple parts, e.g. backing up, deleting, etc.
			#Undo and redo only have one part.
			elif text == "cancel":
				#self.command_hub.cancel()
				self.publisher("cancel")
			elif text == "undo":
				#self.command_hub.undo()
				self.publisher("undo")
			elif text == "redo":
				#self.command_hub.redo()
				self.publisher("redo")

			# Deployment commands
			elif text == "deploy":
				print("received deploy")
				self.publisher("begin_deploy")

			# recording commands
			elif text == "back up" or text == "backup":
				#self.command_hub.begin_go_backward()
				#self.publisher("back up")
				self.publisher("rewind")
			elif text == "go forward":
				#self.command_hub.begin_go_forward()
				#self.publisher("go forward")
				self.publisher("begin_fastforward")
			# editing commands
			elif text == "modify":
				#self.command_hub.begin_modify()
				self.publisher("begin_modify")
			elif text == "insert":
				pass
			elif text == "delete":
				#self.command_hub.begin_delete()
				self.publisher("delete")
			elif text == "add":
				pass
			'''
