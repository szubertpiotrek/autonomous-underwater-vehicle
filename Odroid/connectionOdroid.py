from server import *
from time import sleep
from threading import Thread

#ip = '192.168.137.147' #adres odroida 

class Connection(Thread):
	def __init__(self, ip):
		Thread.__init__(self)
		self.server = Server(ip)
		 

	def run(self):
		while True:
			#odbiera ramki i zapisuje je w zmiennej 
			self.dataFrame = self.server.receiveData()
			#przekazuje ramki do wykorzystania
			self.dataUser.setDataFrame(self.dataFrame)
			
	def getDataFrame(self):
		return self.dataFrame
			


		
		

	


