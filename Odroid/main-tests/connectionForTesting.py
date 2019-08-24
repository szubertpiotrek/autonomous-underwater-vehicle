from server import *

from threading import Thread
from time import sleep

# ip = '192.168.137.200' #adres odroida

class Connection(Thread):
	def __init__(self, ip):
		Thread.__init__(self)
		self.server = Server(ip)
		self.dataFrame = []
	def run(self):
		while True:
			self.server.sendData(self.dataFrame)
			sleep(0.1)
	def setDataFrame(self, dataFrame):
		self.dataFrame = dataFrame
	def getDataFrame(self):
		return self.dataFrame
