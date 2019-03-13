import socket
class Client:
	def __init__(self, ip):	#konstruktor tworzy socket oraz łączy i testuje połączenie z serverem
		self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		port = 8080
		
		try:
			self.client.connect((ip,port))
			msg1 = self.client.recv(1024)
			self.flag = True
		except Exception as e:
			msg1 = 'Connection error'.encode('utf-8')
			self.flag = False
		finally:
			print("Connection test: ", msg1.decode('utf-8'))
	
	def receiveData(self): 	#metoda, którą odpalimy w wątku i będzie odbierać napływające dane z severa
		data = self.client.recv(1024)
		return data.decode('utf-8')
	
	def sendData(self, data):
		self.client.send(data.encode('utf-8'))