import cv2
from .darkflow.net.build import TFNet
import numpy as np
import time
import tensorflow as tf
 

class Camera:
    label=''

    def openCamera(self):
        config = tf.ConfigProto(log_device_placement=True)
        config.gpu_options.allow_growth = True
        with tf.Session(config=config) as sess:
            options = {
                'model': 'cfg/yolov2-tiny-voc-1c.cfg',
                'load': 15000,
                'threshold': 0.1,
                'gpu': 1.0
            }
            tfnet = TFNet(options)

        colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]

        capture = cv2.VideoCapture(0)
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)

        while True:
            stime = time.time()
            ret, frame = capture.read()
            if ret:
                results = tfnet.return_predict(frame)
                for result in results:
                    tl = (result['topleft']['x'], result['topleft']['y'])
                    br = (result['bottomright']['x'], result['bottomright']['y'])
                    self.label = result['label']
                    confidence = result['confidence']
                    text = '{}: {:.0f}%'.format(self.label, confidence * 100)
                    frame = cv2.rectangle(frame, tl, br, (0, 0, 255), 5)
                    frame = cv2.putText(frame, text, tl, cv2.FONT_ITALIC, 1, (0, 0, 0), 2)
                cv2.imshow('frame', frame)
            print('FPS {:.1f}'.format(1 / (time.time() - stime)))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        capture.release()
        cv2.destroyAllWindows()

    def getLabel(self):
        return self.label

    def setLabel(self,value):
        self.label=value
