import socket
import select
import threading
import traceback
import os

class RobotConnector:

	def __init__(self):
		self.HOST = '192.168.0.169'
		self.PORT = 7780

	def send_interaction_files(self):
		try:
			self.s = socket.socket()
			self.s.bind((self.HOST, self.PORT))         
			self.s.listen()
			self.c, addr = self.s.accept()
			print("connected to robot")

			print("CONTROLLER >> sending interaction file to robot...")
			f = open("interaction_.xml", "rb")
			l = os.path.getsize("interaction_.xml")
			m = f.read(l)
			ready_to_read, ready_to_write, in_error = select.select([], [self.c], [])
			ready_to_write[0].sendall(m)
			print("done")
		except:
			print("sending interaction files failed")
			traceback.print_exc()