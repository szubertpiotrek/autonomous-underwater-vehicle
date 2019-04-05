from connectionOdroid import *
from PID import PID

conn = Connection('192.168.137.147') #docelowo inny adres po podłączeniu z Jetsonem?
conn.start()

steerX = PID(0.1, 0.05, 0.05)
steerY = PID(0.1, 0.05, 0.05)

while True:
    # object center coordinates received from Jetson
    objX, objY = 2, 2 # instead of 2, 2, some function returning current object coordinates
    xEnginesDiff = steerX.update(objX)
    yEnginesDiff = steerY.update(objY)