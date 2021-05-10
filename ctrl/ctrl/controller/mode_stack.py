class ModeStack:

	def __init__(self):
		self.stack = []

	def add_mode(self, mode):
		self.stack.append(mode)
		mode.execute()

	def peek_mode(self):
		if len(self.stack) > 0:
			return self.stack[-1]
		return None

	def peek_mode_type(self):
		if len(self.stack) > 0:
			return self.stack[-1].type
		return None

	def pop_mode(self,data=None):
		self.delete_mode()
		if len(self.stack) > 0:
			self.stack[-1].update(data)

	def cancel_mode(self):
		print("cancelling mode")
		if len(self.stack) > 0:
			# remove the first occurence without deleting from memory
			self.stack[-1].terminate()
			self.delete_mode()
			if len(self.stack) > 0:
				self.stack[-1].notify_child_terminated()

	def delete_mode(self):
		if len(self.stack) > 0:
			del self.stack[-1]

	def update(self, arg):
		if len(self.stack) > 0:
			self.stack[-1].update(arg)
			return True
		else:
			# what happens if data is passed back, but there is no mode to receive it?
			return False

	def is_empty(self):
		return len(self.stack) == 0