from Camera import Camera
import threading
import time

import subprocess

#subprocess.call(["cd ~/bin", "sudo jetson_clocks", "sudo nvpmodel -m 0"])

from connectionJetson import Connection


label = ''
lock=threading.Lock()

class MyThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.cam = Camera()

    def run(self):
        global label

        self.cam.openCamera()
        label = self.cam.getLabel()
        time.sleep(0.01)

    def getCamera(self):
        return self.cam

class LabelThread(threading.Thread):

    def run(self):
        global label
        # while 1:
        #     print(label)

class FrameMakerThread(threading.Thread):
    singleDataFrame = []
    multiDataFrame = []

    def __init__(self, cam, connection):
        threading.Thread.__init__(self)
        self.cam = cam
        self.connection = connection

        for item in range(0,6):
            self.singleDataFrame.append(None)

    def makeSingleFrame(self,objectNum):
        # objectNum to index obiektu ktorego dane chcemy przepisac w danej iteracji funkcji
        # stereoMonoFlag = True jeżeli chcemy wysłąć ramkę z stereoDist, =False kiedy z monoDist
        print('SingleFrame zrobioned11!')
        #lock.acquire() #tworzy zamek do czasu uruchomienia metody release
        self.singleDataFrame[0] = self.cam.getObjDistances()[objectNum]
        print(self.cam.getObjCenterDeltasXY(), objectNum)
        self.singleDataFrame[1] = self.cam.getObjCenterDeltasXY()[objectNum][0] # x
        self.singleDataFrame[2] = self.cam.getObjCenterDeltasXY()[objectNum][1] # y
        self.singleDataFrame[3] = self.cam.getObjectsFillLevel[objectNum]
        self.singleDataFrame[4] = self.cam.getDetectImages[objectNum]
        self.singleDataFrame[5] = stereoMonoFlag # flaga
        print('SingleFrame zrobioned222!')
        #lock.release()
        return self.singleDataFrame


    def makeMultiFrame(self, numOfObjects):
        # numOfObjects to liczba wykrytych przez kamerę obiektów, a co za tym idzie ilość potrzebnych wierszy w multi ramce
        # stereoMonoFlag = True jeżeli chcemy wysłąć ramkę z stereoDist, =False kiedy z monoDist
        self.multiDataFrame.clear()
        for objectNum in range(0, numOfObjects):
            #lock.acquire()
            self.multiDataFrame.append(self.makeSingleFrame(objectNum))
            #lock.release()
        return self.multiDataFrame

    def run(self):
        while True:
            lock.acquire()
            self.connection.setDataFrame(self.makeMultiFrame(len(self.cam.getDetectImages())))
            lock.release()

            time.sleep(0.2) # przykładowe opóźnienie
           


connFlag = True # flaga -> opuszczanie pętli od razu po połączeniu
connThread = ''
while connFlag:
    # Jetson próbuje połączyć się z odroidem przez ethernet od razu bo zbootowaniu się Jetsona
    connThread = Connection('10.42.0.158')
    connFlag = not connThread.flag

mythread1 = MyThread()
mythread2 = LabelThread()
mythread1.start()
mythread2.start()

frameMaker = FrameMakerThread(mythread1.getCamera(), connThread)
connThread.start() #rozpoczyna wysyłanie ramek danych do odroida przez ethernet
frameMaker.start()
