# from server import *
# from time import sleep
# from threading import Thread
#
#
# # ip = '192.168.137.200' #adres odroida
#
# class Connection(Thread):
#     def __init__(self, ip):
#         Thread.__init__(self)
#         self.server = Server(ip)
#
#     def run(self):
#         while True:
#             # odbiera ramki i zapisuje je w zmiennej
#             self.dataFrame = self.server.receiveData()
#
#     def getDataFrame(self):
#         return self.dataFrame

from client import *

from threading import Thread
import time

# ip = '192.168.137.68' #adres odroida


class Connection(Thread):
    def __init__(self, ip):
        Thread.__init__(self)
        self.client = Client(ip)
        self.flag = self.client.flag
        self.dataFrame = []

    def run(self):
        while True:
            # odbiera ramki i zapisuje je w zmiennej
            self.dataFrame = self.client.receiveData()


    def setDataFrame(self, dataFrame):
        self.dataFrame = dataFrame

    def getDataFrame(self):
        return self.dataFrame
