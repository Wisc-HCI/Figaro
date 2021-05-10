from google.cloud import pubsub_v1
from google.oauth2 import service_account
import json
import threading
import traceback
import time
import sys

class BLEInput:

	def __init__(self,controller_callback):


		pubsub_key = json.load(open('pubsub_key.json'))
		credentials = (service_account.Credentials.from_service_account_info(pubsub_key))

		# Initialize a Subscriber client
		self.subscriber_client = pubsub_v1.SubscriberClient(credentials=credentials)
		self.publisher_client = pubsub_v1.PublisherClient(credentials=credentials)

		# Create a fully qualified identifier in the form of
		# `projects/{project_id}/subscriptions/{subscription_id}`
		self.subscription_path = self.subscriber_client.subscription_path("verification-of-hri", "robot_sub")
		self.topic_path = self.publisher_client.topic_path("verification-of-hri", "robot_topic")

		self.controller_callback = controller_callback

		self.thread_lock = threading.Lock()
		self.receiving = True

		thread = threading.Thread(target=self.message_listen)
		thread.daemon = True							# Daemonize thread
		thread.start()

	def message_listen(self):

		print("about to create sub.")
		success = False

		while not success:
			try:
				sub = self.subscriber_client.create_subscription(name=self.subscription_path, topic=self.topic_path)
				success = True
				print("successfully subscribed")
			except:
				time.sleep(1)

		streaming_pull_future = self.subscriber_client.subscribe(
			self.subscription_path, callback=self.internal_callback
		)

		print("Starting thread to listen for messages on {}..\n".format(self.subscription_path))
		print("thread starting...")

		with self.subscriber_client:
			try:
				# Calling result() on StreamingPullFuture keeps the main thread from
				# exiting while messages get processed in the callbacks.
				streaming_pull_future.result()
			except Exception as err:  # noqa
				streaming_pull_future.cancel()
				traceback.print_tb(err.__traceback__)

	def internal_callback(self,message):
		message.ack()
		str_data = message.data.decode('utf-8')

		split_idx = str_data.index("#")
		timestamp = float(str_data[:split_idx])
		str_data = str_data[split_idx+1:]

		split_idx = str_data.index("#")
		rh = str_data[:split_idx]
		str_data = str_data[split_idx+1:]

		split_idx = str_data.index("#")
		magnitude = float(str_data[:split_idx])
		str_data = str_data[split_idx+1:]

		split_idx = str_data.index("#")
		angle = float(str_data[:split_idx])
		str_data = str_data[split_idx+1:]

		voltage = float(str_data)

		#x_val = int(str_data[:split_idx])
		#y_val = int(str_data[split_idx:])

		#print("received\n  time: {}\n    rh: {}\n   mag: {}\n  angl: {}\n  volt: {}".format(timestamp, rh, magnitude, angle, voltage))

		# TODO: pushdown val?
		self.controller_callback(timestamp, rh, magnitude, angle, voltage)

	#cleanup function
	def signal_handler(self, sig, frame):
		# stop the receival loop
		print('Deleting subscriber before closing...')
		self.subscriber_client.delete_subscription(self.subscription_path)
		self.subscriber_client.close()
		print('Finished cleanup, exiting!')
		sys.exit(0)