from client import *

from threading import Thread
from time import sleep
import random

#ip = '192.168.137.147' #adres odroida

class Connection(Thread):
	def __init__(self, ip):
		Thread.__init__(self)
		self.client = Client(ip) 
		self.flag = self.client.flag		 
		self.dataFrame = []
	def run(self):
		while True:
			self.client.sendData(self.dataFrame)
			sleep(1) #na przykład 1s opóźnienia 

	def setDataFrame(self, dataFrame):
		self.dataFrame = dataFrame
		
connFlag = True
while connFlag: 
	#Jetson próbuje połączyć się z odroidem przez ethernet
	conn = Connection('192.168.137.147')  
	connFlag = not conn.flag

conn.start() #rozpoczyna wysyłanie ramek danych do odroida przez ethernet
		