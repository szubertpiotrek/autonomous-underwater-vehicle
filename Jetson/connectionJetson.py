from client import *

from threading import *
from time import sleep

#ip = '192.168.137.42' #adres odroida

class Connection(Thread):
	def __init__(self, ip):
		Thread.__init__(self)
		self.client = Client(ip) 

	def run(self):
		while True:
			self.client.sendData('Hello server') #Hello server to przykładowe dane
			sleep(1) #na przykład 1s opóźnienia 
			
conn = Connection('192.168.137.42')  #to docelowo dać do Main w jetsonie tak jak w Odroidzie, 
conn.start()