from Camera import Camera
import threading
import time
import subprocess

subprocess.call(["cd ~/bin", "sudo jetson_clocks", "sudo nvpmodel -m 0"])


label=''

class MyThread(threading.Thread):

    def run(self):
        global label
        cam = Camera()
        cam.openCamera()
        label = cam.getLabel()

class LabelThread(threading.Thread):

    def run(self):
        global label
        #while 1:
            #print (label)


mythread1 = MyThread()
mythread2 = LabelThread()
mythread1.start()
mythread2.start()
