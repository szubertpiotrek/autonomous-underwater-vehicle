import socket
from time import sleep
class Server:
	def __init__(self, ip):	#konstruktor tworzy socket oraz łączy i testuje połączenie z serverem
		self.server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		port = 8080
		address = (ip, port)
		self.server.bind(address)
		self.server.listen(20)
		self.statusFlag = False
		self.client, addr = self.server.accept()
		msg ='Connection succesful'
		self.client.send(msg.encode('utf-8'))
					
				
	
	def receiveData(self): 	#metoda, którą odpalimy w wątku i będzie odbierać napływające dane z severa
		data = self.client.recv(1024)
		return data.decode('utf-8')
	
	def sendData(self, data):
		self.client.send(data.encode('utf-8'))