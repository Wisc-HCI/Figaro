import time
import math
from datetime import datetime
import signal
import sys
import json
import sys
from google.cloud import pubsub_v1
from google.oauth2 import service_account
import time

class TestNode:

	def __init__(self, project_id, topic_name, act):
		# set the actor as human or robot
		self.actor = act

		# Authenticate google cloud services
		pubsub_key = json.load(open('pubsub_key.json'))
		credentials = (service_account.Credentials.from_service_account_info(pubsub_key))


		# Initialize a Publisher client.
		self.client = pubsub_v1.PublisherClient(credentials=credentials)

		# Create a fully qualified identifier in the form of
		# `projects/{project_id}/topics/{topic_id}`
		self.topic_path = self.client.topic_path('verification-of-hri', 'robot_topic')

	def get_callback(self, api_future, data, ref):
		"""Wrap message data in the context of the callback function."""

		def callback(api_future):
			try:
				print(
					"Published message {} now has message ID {}".format(
						data, api_future.result()
					)
				)
				ref["num_messages"] += 1
			except Exception:
				print(
					"A problem occurred when publishing {}: {}\n".format(
						data, api_future.exception()
					)
				)
				raise

		return callback

	def pub(self, direct, mag, but):
		# Keep track of the number of published messages.
		ref = dict({"num_messages": 0})
		
		while True:
			timestamp = datetime.utcnow()
			data = str(timestamp) + "#" + self.actor + "#" + str(direct) + "#" + str(mag) + "#" + str(but)
			print('sending ' + data)
			data = bytes(data, 'UTF-8')
			api_future = self.client.publish(self.topic_path, data)
			api_future.add_done_callback(self.get_callback(api_future, data, ref))

			time.sleep(1)

	def signal_handler(self, sig, frame):
	    print('Deleting topic before closing...')
	    self.client.delete_topic(self.topic_path)
	    print('Finished cleanup, exiting!')
	    sys.exit(0)

if __name__=="__main__":
	tn = TestNode('verification-of-hri', 'robot_topic', sys.argv[1])
	signal.signal(signal.SIGINT, tn.signal_handler)
	tn.pub('33000', '32300', '184')