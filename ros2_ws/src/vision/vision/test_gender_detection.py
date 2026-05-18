"""
test_gender_detection.py

Standalone test for gender detection.
No ROS2, no robot, no bridge needed.

Run from Docker terminal:
    python3 /ros2_ws/src/vision/vision/test_gender_detection.py
"""

import sys
import time
import cv2

sys.path.insert(0, '/ros2_ws/src/vision')
from vision.gender_detection import GenderDetector


def main():
    detector = GenderDetector()

    cap = cv2.VideoCapture('/dev/video10')
    if not cap.isOpened():
        print("ERROR: Cannot open /dev/video10")
        return

    print("\n--- GENDER DETECTION TEST ---")
    print("Stand in front of the camera.")
    print("Customer A = Female")
    print("Customer B = Male")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Frame capture failed")
                time.sleep(0.5)
                continue

            gender, conf = detector.detect(frame)

            if gender:
                customer = "Customer A" if gender == "Female" else "Customer B"
                print(f"[DETECTED] {gender} ({conf:.2f}) → deliver to {customer}")
            else:
                print("[WAITING]  No face detected")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()