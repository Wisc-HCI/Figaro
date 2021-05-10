def concat(string1, string2):
	return "{}-{}".format(string1,string2)

class BiMap:

	def __init__(self, dic):
		self.key = dic
		self.val = {}

		self.update()

	def add(self, key, val):
		self.key[key] = val
		self.val[val] = key

	def update(self):
		for key,val in self.key.items():
			self.val[val] = key

	def __str__(self):
		string = ""
		for key,val in self.key.items():
			string += " <{} : {}> ".format(key, val)
		return string