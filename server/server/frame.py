import sys
import os
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5.QtCore import QUrl

class TabletFrame(QMainWindow):

	def __init__(self):
		super().__init__()
		self.title = 'Figaro Tablet Control'
		self.setWindowTitle(self.title)
		self.setGeometry(200,200,800,600)
		html_view = HTMLVis(self)
		self.show()

class HTMLVis(QWebEngineView):

	def __init__(self, parent):
		super().__init__(parent)
		self.cwd = os.getcwd()
		self.setGeometry(0,0,800,600)
		url = QUrl.fromLocalFile("{}/index.html".format(self.cwd))
		self.load(url)