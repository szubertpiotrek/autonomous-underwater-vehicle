import socket
import pickle
class Client:
	def __init__(self, ip):	#konstruktor tworzy socket oraz łączy i testuje połączenie z serverem
		self.client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
		port = 8080
		
		try:
			self.client.connect((ip,port))
			msg1 = self.client.recv(4096)
			self.flag = True
		except Exception as e:
			msg1 = pickle.dumps('Connection error')
			self.flag = False
		finally:
			print("Connection test: ", pickle.loads(msg1))
	
	def receiveData(self): 	#metoda, którą odpalimy w wątku i będzie odbierać napływające dane z severa
		data = self.client.recv(4096)
		return pickle.loads(data)
	
	def sendData(self, data):
		self.client.send(pickle.dumps(data))
