from client import *

from threading import *
from time import sleep

#ip = '192.168.137.147' #adres odroida

class Connection(Thread):
	def __init__(self, ip, data):
		Thread.__init__(self)
		self.client = Client(ip) 
		self.flag = self.client.flag		 
		self.data = data #jeżeli w przypiszemy polu data referencje do zmieniającej się ramki to zadziała?
	def run(self):
		while True:
			self.client.sendData(self.data) #Hello server to przykładowe dane
			sleep(1) #na przykład 1s opóźnienia 

connFlag = True
while connFlag: #Jetson próbuje połączyć się z odroidem przez ethernet
	conn = Connection('192.168.137.147', 'Ramka Danych!')  #to docelowo dać do wątku w jetsonie	?
	connFlag = not conn.flag

conn.start() #rozpoczyna wysyłanie ramek danych do odroida przez ethernet
		#mozna tez nie przypisywać od razu referecji tylko poprostu zmieniac pole conn.data = 'ramka' w Mainie
		#czyli to jest do przekminy ;p 