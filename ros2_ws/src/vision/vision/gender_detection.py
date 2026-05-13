import os
import cv2
import numpy as np

_BASE = os.path.dirname(os.path.abspath(__file__))

# Model Paths
FACE_PROTO   = os.path.join(_BASE, "opencv_face_detector.pbtxt")
FACE_MODEL   = os.path.join(_BASE, "opencv_face_detector_uint8.pb")
GENDER_PROTO = os.path.join(_BASE, "gender_deploy.prototxt")
GENDER_MODEL = os.path.join(_BASE, "gender_net.caffemodel")

# Adience Dataset constants from the repo
MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
GENDER_LIST = ['Male', 'Female']

class GenderDetector:
    def __init__(self):
        # Load the TensorFlow Face model
        self.face_net = cv2.dnn.readNetFromTensorflow(FACE_MODEL, FACE_PROTO)
        # Load the Caffe Gender model
        self.gender_net = cv2.dnn.readNet(GENDER_MODEL, GENDER_PROTO)
        print("[GenderDetector] Models initialized successfully.")

    def detect(self, frame):
        if frame is None: return None, 0.0
        h, w = frame.shape[:2]
        
        # 1. Detect Face
        blob = cv2.dnn.blobFromImage(frame, 1.0, (300, 300), [104, 117, 123], True, False)
        self.face_net.setInput(blob)
        detections = self.face_net.forward()

        best_gender = None
        best_conf = 0.0

        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.7:  # Confidence threshold for face
                x1 = int(detections[0, 0, i, 3] * w)
                y1 = int(detections[0, 0, i, 4] * h)
                x2 = int(detections[0, 0, i, 5] * w)
                y2 = int(detections[0, 0, i, 6] * h)

                face_crop = frame[max(0,y1):min(h,y2), max(0,x1):min(w,x2)]
                if face_crop.size == 0: continue

                # 2. Predict Gender from face crop
                gender_blob = cv2.dnn.blobFromImage(face_crop, 1.0, (227, 227), 
                                                   MODEL_MEAN_VALUES, swapRB=False)
                self.gender_net.setInput(gender_blob)
                preds = self.gender_net.forward()
                
                idx = preds[0].argmax()
                if preds[0][idx] > best_conf:
                    best_conf = preds[0][idx]
                    best_gender = GENDER_LIST[idx]
        
        return best_gender, best_conf
