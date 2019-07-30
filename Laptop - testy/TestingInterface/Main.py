from GUI import *
from connectionLaptop import Connection
import time
import threading

IP_ADDRESS = '192.168.137.68'

def sampleAngles(conn, sim):
    while sampleFlag:
        sim.setCurrentAngles(conn.getDataFrame())
        time.sleep(0.002)
        #sim.setCurrentAngles([conn.getDataFrame()[i] for i in range(3)])
        #sim.setCurrentDepth(conn.getDataFrame()[3])
        #sim.setCurrentMotorsPower([conn.getDataFrame()[i] for i in range(4, 9)])


sampleFlag = True
s = Simulation3D()
connFlag = True  # flaga -> opuszczanie pętli od razu po połączeniu
while connFlag:
    connThread = Connection(IP_ADDRESS)
    connFlag = not connThread.flag

connThread.start()
s.start()
time.sleep(1)
threading.Thread(target=sampleAngles, args=[connThread, s]).start()


# def readSamples(connectionObj):
#     while True:
#         s1.catchallsamples(sv, 1.0)
#         if c % 100 == 0:
#             print(hs.format(*statevars))
#         print(fs.format(s1.state))
#         c += 1
#         connectionObj.setDataFrame(s1.state)
#
#
# connFlag = True  # flaga -> opuszczanie pętli od razu po połączeniu
# while connFlag:
#     connThread = Connection('192.168.137.1')
#     connFlag = not connThread.flag
#
# threading.Thread(target=readSamples, args=[connThread]).start()
# connThread.start()
