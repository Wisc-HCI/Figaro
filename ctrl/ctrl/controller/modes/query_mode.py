from controller.modes.mode import *
import time

class QueryMode(Mode):

	def __init__(self,name,done_callback,terminate_callback,query_publisher,received_answer,message,destination,data):
		super().__init__(name,done_callback,terminate_callback)
		self.message = message
		self.destination = destination
		self.data = data
		self.query_publisher_callback = query_publisher
		self.received_answer_callback = received_answer
		self.type = "query"
		self.query_answer = None

	def execute(self):
		# begin the determine state mode
		print("sending {} to {}".format(self.message, self.destination))
		self.query_publisher_callback(self.message,self.destination)

	def notify_child_terminated(self):
		self.done_callback(None)

	def check_done(self):
		if self.query_answer is not None:
			self.done_callback(None)
			self.received_answer_callback(self.query_answer,self.data)

	def update(self, arg):
		'''
		Received a query response
		'''
		print("{} QUERY is updating".format(self.name))
		if arg is not None and len(arg) > 0 and arg[0] == "query":
			self.query_answer = arg[1]
			self.check_done()