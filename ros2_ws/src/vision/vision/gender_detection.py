"""
gender_detection.py

Standalone gender detection using:
  - smahesh29 TensorFlow face detector
  - Gil Levi / Tal Hassner Caffe gender classifier

Run directly to test:
    python3 gender_detection.py
"""

import os
import cv2
import numpy as np

_BASE = os.path.dirname(os.path.abspath(__file__))

FACE_PROTO   = os.path.join(_BASE, "opencv_face_detector.pbtxt")
FACE_MODEL   = os.path.join(_BASE, "opencv_face_detector_uint8.pb")
GENDER_PROTO = os.path.join(_BASE, "gender_deploy.prototxt")
GENDER_MODEL = os.path.join(_BASE, "gender_net.caffemodel")

MODEL_MEAN_VALUES = (78.4263377603, 87.7689143744, 114.895847746)
GENDER_LIST       = ["Male", "Female"]
FACE_CONFIDENCE   = 0.7


class GenderDetector:
    def __init__(self):
        if not all(os.path.exists(p) for p in
                   [FACE_PROTO, FACE_MODEL, GENDER_PROTO, GENDER_MODEL]):
            raise FileNotFoundError(
                "Model files missing — run the wget commands to download them"
            )
        self.face_net   = cv2.dnn.readNetFromTensorflow(FACE_MODEL, FACE_PROTO)
        self.gender_net = cv2.dnn.readNet(GENDER_MODEL, GENDER_PROTO)
        print("[GenderDetector] Models loaded successfully")

    def detect(self, frame):
        """
        Returns (gender, confidence):
            gender = "Male" | "Female" | None
        """
        if frame is None:
            return None, 0.0

        h, w = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(
            frame, 1.0, (300, 300), [104, 117, 123], True, False
        )
        self.face_net.setInput(blob)
        detections = self.face_net.forward()

        best_gender = None
        best_conf   = 0.0

        for i in range(detections.shape[2]):
            face_conf = float(detections[0, 0, i, 2])
            if face_conf < FACE_CONFIDENCE:
                continue

            x1 = max(0, int(detections[0, 0, i, 3] * w))
            y1 = max(0, int(detections[0, 0, i, 4] * h))
            x2 = min(w, int(detections[0, 0, i, 5] * w))
            y2 = min(h, int(detections[0, 0, i, 6] * h))

            face_crop = frame[y1:y2, x1:x2]
            if face_crop.size == 0:
                continue

            gender_blob = cv2.dnn.blobFromImage(
                face_crop, 1.0, (227, 227),
                MODEL_MEAN_VALUES, swapRB=False
            )
            self.gender_net.setInput(gender_blob)
            preds = self.gender_net.forward()

            idx  = int(preds[0].argmax())
            conf = float(preds[0][idx])

            if conf > best_conf:
                best_conf   = conf
                best_gender = GENDER_LIST[idx]

        return best_gender, best_conf
