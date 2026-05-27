"""
test_gender_detection.py
Standalone gender detection test.
Rules:
  - Male at 1.00 confidence → Customer B
  - Female at >= 0.95 confidence → Customer A
  - Anything else → keep scanning
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
    print("Customer A = Female (conf >= 0.95)")
    print("Customer B = Male   (conf = 1.00)")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[ERROR] Frame capture failed")
                time.sleep(0.5)
                continue

            gender, conf = detector.detect(frame)

            if gender is None:
                print("[WAITING]  No face detected")

            elif gender == "Male" and conf >= 1.00:
                print(f"[DETECTED] Male ({conf:.2f}) → Customer B")

            elif gender == "Female" and conf >= 0.95:
                print(f"[DETECTED] Female ({conf:.2f}) → Customer A")

            else:
                print(f"[UNSURE]   {gender} ({conf:.2f}) — confidence too low, keep scanning")

            time.sleep(0.5)

    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
