from server import *

from threading import *

#ip = '192.168.137.42' #adres odroida 

class Connection(Thread):
	def __init__(self, ip):
		Thread.__init__(self)
		self.server = Server(ip)
		

	def run(self):
		while True:
			data = self.server.receiveData()
			print(data)
			 

			
#conn = Connection('192.168.137.42') #w MainOdroid
#conn.start()

		

	


