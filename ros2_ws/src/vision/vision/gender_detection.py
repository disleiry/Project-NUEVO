"""
gender_detection.py

Standalone gender detection using:
  - smahesh29 TensorFlow face detector
  - InsightFace genderage.onnx gender classifier (replaces Caffe model)
"""

import os
import cv2
import numpy as np
import onnxruntime as ort

_BASE = os.path.dirname(os.path.abspath(__file__))

FACE_PROTO   = os.path.join(_BASE, "opencv_face_detector.pbtxt")
FACE_MODEL   = os.path.join(_BASE, "opencv_face_detector_uint8.pb")
GENDER_MODEL = os.path.join(_BASE, "genderage.onnx")

FACE_CONFIDENCE = 0.7


class GenderDetector:
    def __init__(self):
        if not all(os.path.exists(p) for p in
                   [FACE_PROTO, FACE_MODEL, GENDER_MODEL]):
            raise FileNotFoundError(
                "Model files missing — check FACE_PROTO, FACE_MODEL, GENDER_MODEL paths"
            )
        self.face_net      = cv2.dnn.readNetFromTensorflow(FACE_MODEL, FACE_PROTO)
        self.gender_sess   = ort.InferenceSession(
            GENDER_MODEL,
            providers=['CPUExecutionProvider']
        )
        self._gender_input = self.gender_sess.get_inputs()[0].name
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

            # genderage.onnx: input [1,3,96,96], RGB, float32, /255
            resized = cv2.resize(face_crop, (96, 96))
            rgb     = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            inp     = (rgb.transpose(2, 0, 1).astype(np.float32) / 255.0)[np.newaxis]

            preds  = self.gender_sess.run(None, {self._gender_input: inp})[0][0]
            gender = "Female" if preds[0] > 0.34 else "Male"
            print(f"[DEBUG] face_conf={face_conf:.2f} preds={preds} → {gender}")


            if face_conf > best_conf:
                best_conf   = face_conf
                best_gender = gender

        return best_gender, best_conf
