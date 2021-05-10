from std_msgs.msg import String

import xml.etree.ElementTree as ET
import os

class Deployer:

	def __init__(self, deployment_pub, speech_pub):
		self.deployment_pub = deployment_pub
		self.speech_pub = speech_pub

	def deploy(self):
		'''
		1) First clear the interrupts
		'''
		print("asking Temi to wipe interrupts")
		msg = String()
		msg.data = "wipe_interrupts###wipe interrupts"
		self.deployment_pub.publish(msg)

		'''
		2) Send the speech
			1. first to Temi android
			2. second to helper speech node
		'''
		# get the speech content
		print("sending off speech XML")
		tree = ET.parse('speech.xml')
		speech_root = tree.getroot()
		speech_string = ET.tostring(speech_root, encoding='utf-8', method='xml').decode('utf-8')

		msg = String()
		msg.data = "speech_xml###{}".format(speech_string)
		self.deployment_pub.publish(msg)

		ros_msg = String()
		ros_msg.data = speech_string
		self.speech_pub.publish(ros_msg)

		'''
		3) Send the main interaction
		'''
		print("sending off main xml")
		print("sending main xml...")
		if "interaction__self.xml" in os.listdir("."):
			name = self.send_xml_content("interaction__self.xml","main")

		'''
		4) Send the interrupts
		'''
		print("sending interrupts")
		for filename in os.listdir("."):
			if filename.endswith(".xml") and filename != "interaction__self.xml" and filename != "speech.xml": 
				print("sending interrupt xml...")
				name = self.send_xml_content(filename,"interrupt")

	def send_xml_content(self, filename, is_interrupt):
		tree = ET.parse(filename)
		root = tree.getroot()

		# get the speech content
		content = ET.tostring(root, encoding='utf-8', method='xml').decode('utf-8')
		name = root.tag

		msg = String()
		msg.data = "{}###{}###{}".format(is_interrupt,content,name)
		self.deployment_pub.publish(msg)