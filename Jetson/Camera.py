import os
import cv2
import numpy as np
import time
import darknet


class Camera:
    label=''
    # list of centers (x, y) of currently detected objects
    objCenters = []
    # list contain lists of zones of currently detected objects
    objZones = []

    def openCamera(self):
        
        metaMain = None
        netMain = None
        altNames = None
        
        configPath = "cfg/yolov3-tiny-obj.cfg"
        weightPath = "backup/yolov3-tiny-obj_2000.weights"
        metaPath = "data/r2d2.data"
        
        if netMain is None:
            netMain = darknet.load_net_custom(configPath.encode("ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
        if metaMain is None:
            metaMain = darknet.load_meta(metaPath.encode("ascii"))
        if altNames is None:
            try:
                with open(metaPath) as metaFH:
                    metaContents = metaFH.read()
                    import re
                    match = re.search("names *= *(.*)$", metaContents,
                                      re.IGNORECASE | re.MULTILINE)
                    if match:
                        result = match.group(1)
                    else:
                        result = None
                    try:
                        if os.path.exists(result):
                            with open(result) as namesFH:
                                namesList = namesFH.read().strip().split("\n")
                                altNames = [x.strip() for x in namesList]
                    except TypeError:
                        pass
            except Exception:
                pass

        capture = cv2.VideoCapture(0)
        # capture.set(cv2.CAP_PROP_FRAME_WIDTH, 800)
        # capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 400)
        
        darknet_image = darknet.make_image(darknet.network_width(netMain), darknet.network_height(netMain), 3)

        while True:
            stime = time.time()
            ret, frame = capture.read()
            if ret:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_resized = cv2.resize(frame_rgb,
                          (darknet.network_width(netMain),
                           darknet.network_height(netMain)),
                          interpolation=cv2.INTER_LINEAR)

                heightFrame = np.size(frame_resized, 0)
                widthFrame = np.size(frame_resized, 1)

                darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())
                
                detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.25)

                frame, xmin, ymin, xmax, ymax = self.cvDrawBoxes(detections, frame_resized)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

                self.saveObjectsCenters(detections)
                objectsFillLevel = self.getObjectsFillLevel(detections, heightFrame, widthFrame)
                print("Wypelnienie:", round(objectsFillLevel, 2), "%")
                cv2.imshow('frame', frame)
                self.saveObjectsZones(detections, heightFrame, widthFrame)
                print(self.getObjectsZones())
            print('FPS {:.1f}'.format(1 / (time.time() - stime)))
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        capture.release()
        cv2.destroyAllWindows()

    def getObjectCenter(self, detection):
        x, y = detection[2][0], detection[2][1]
        return x, y

    def getObjectDimensions(self, detection):
        width, height = detection[2][2], detection[2][3]
        return width, height

    def getObjectsNum(self, detections):
        return len(detections)

    def saveObjectsCenters(self, detections):
        objNum = self.getObjectsNum(detections)
        for detection in detections:
            if detections.index(detection) < len(self.objCenters):
                self.objCenters[detections.index(detection)] = self.getObjectCenter(detection)
            else:
                self.objCenters.append(self.getObjectCenter(detection))

        # pop all surplus elements
        for i in range(objNum, len(self.objCenters)):
            self.objCenters.pop(objNum)

    def getObjectsVertexes(self, detections):
        objVertexes = []
        for detection in detections:
            x, y = self.getObjectCenter(detection)
            w, h = self.getObjectDimensions(detection)
            xmin, ymin, xmax, ymax = self.convertBack(
                float(x), float(y), float(w), float(h))
            tl = [xmin, ymin]
            tr = [xmax, ymin]
            br = [xmax, ymax]
            bl = [xmin, ymax]
            rect = [tl, tr, br, bl]
            objVertexes.append(rect)
        return objVertexes

    def getObjectsFillLevel(self, detections, heightFrame, widthFrame):
        objVertexesArr = np.array(self.getObjectsVertexes(detections), dtype=np.int32)
        im = np.zeros([heightFrame, widthFrame], dtype=np.uint8)
        cv2.fillPoly(im, objVertexesArr, 1)
        objectsArea = cv2.countNonZero(im)
        frameArea = heightFrame * widthFrame
        objFillLvl = objectsArea / frameArea * 100
        return objFillLvl

    def convertBack(self, x, y, w, h):
        xmin = int(round(x - (w / 2)))
        xmax = int(round(x + (w / 2)))
        ymin = int(round(y - (h / 2)))
        ymax = int(round(y + (h / 2)))
        return xmin, ymin, xmax, ymax

    def cvDrawBoxes(self, detections, img):
        xmin=0
        ymin=0
        xmax=0
        ymax=0
        for detection in detections:
            x, y = self.getObjectCenter(detection)
            w, h = self.getObjectDimensions(detection)
            xmin, ymin, xmax, ymax = self.convertBack(
                float(x), float(y), float(w), float(h))
            tl = (xmin, ymin)
            br = (xmax, ymax)
            cv2.rectangle(img, tl, br, (0, 255, 0), 1)
            cv2.putText(img,
                        detection[0].decode() +
                        " [" + str(round(detection[1] * 100, 2)) + "]",
                        (tl[0], tl[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        [0, 255, 0], 2)
        return img, xmin, ymin, xmax, ymax
    def getDetectionObjectZones(self, detection, heightFrame, widthFrame):
        detectionObjZones = []
        x, y = self.getObjectCenter(detection)
        w, h = self.getObjectDimensions(detection)
        xmin, ymin, xmax, ymax = self.convertBack(float(x), float(y), float(w), float(h))
        x_zonemin = int(xmin/(widthFrame/3))
        y_zonemin = int(ymin/(heightFrame/3))
        x_zonemax = int(xmax/(widthFrame/3))
        y_zonemax = int(ymax/(heightFrame/3))
        for i in range(x_zonemin, x_zonemax+1):
            for j in range(y_zonemin, y_zonemax+1):
                detectionObjZones.append(i+3*j+1)
        return detectionObjZones

    def saveObjectsZones(self, detections, heightFrame, widthFrame):
       objNum = self.getObjectsNum(detections)
       for detection in detections:
           if detections.index(detection) < len(self.objZones):
               self.objZones[detections.index(detection)] = self.getDetectionObjectZones(detection, heightFrame, widthFrame)
           else:
               self.objZones.append(self.getDetectionObjectZones(detection, heightFrame, widthFrame))
       # pop all surplus elements
       for i in range(objNum, len(self.objZones)):
           self.objZones.pop(objNum)

    def getObjectsZones(self):
        return self.objZones

    def getLabel(self):
        return self.label

    def setLabel(self, value):
        self.label = value
