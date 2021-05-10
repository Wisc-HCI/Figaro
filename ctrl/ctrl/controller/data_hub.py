from controller.speech_database import *

from figaro_msgs.msg import Query
from figaro_msgs.msg import DataPass
from figaro_msgs.msg import QueryArray
from figaro_msgs.msg import DataPassArray

class DataHub:
	'''
	Receive queries from various components of Figaro, and pass them to other components
	'''
	def __init__(self, controller):
		self.controller = controller

		'''
		SENDING QUERIES
		'''
		# tablet
		self.tablet_has_query_subscriber = self.controller.create_subscription(QueryArray, 'tablet_query_pub_ctrl', self.tablet_query_sub_callback, 10)

		# publishers
		self.ctrl_query_tablet_publisher = self.controller.create_publisher(QueryArray, 'ctrl_query_pub_tablet', 10)

		'''
		SENDING ANSWERS
		'''
		# tablet
		self.tablet_sends_answer_subscriber = self.controller.create_subscription(DataPassArray, 'tablet_datapass_pub_ctrl', self.tablet_answer_sub_callback, 10)

		# publishers
		self.ctrl_answer_tablet_pub = self.controller.create_publisher(DataPassArray, 'ctrl_datapass_pub_tablet', 10)

	def tablet_query_sub_callback(self, msg):
		queries = msg.queries
		for query in queries:
			if query.destination == "ctrl":
				self.controller.respond_to_query(msg)

	def tablet_answer_sub_callback(self, msg):
		print("received answer from tablet: {}".format(msg))
		stack_updated = self.controller.mode_stack.update(("query",msg))

	def process_unaccompanied_datapass(self,msg):
		data_bundle = msg.data_bundle

		for data in data_bundle:
			if data.type == "DataPassAllIntentCategories":

				# clear the current db
				self.controller.speech_db.provided_intents = {}

				# access the updated speech
				cats = data.all_intent_cats_data
				category_list = cats.edited_intent_categories.array
				phrases_list = cats.edited_speech_examples

				for i in range(len(category_list)):
					cat = category_list[i]
					phrases = phrases_list[i].array

					print("new cat: {}".format(cat))
					print("new phrases: {}".format(phrases))

					new_intent = ProvidedIntent(cat,phrases)
					self.controller.speech_db.provided_intents[cat] = new_intent

	'''
	Figure out where to publish the query
	'''
	def publish_query(self,msg,destination):
		print("sending off query")
		if destination == "tablet":
			print("sending to tablet")
			self.ctrl_query_tablet_publisher.publish(msg)

	def publish_answer(self, msg, destination):
		if destination == "tablet":
			self.ctrl_answer_tablet_pub.publish(msg)
