"""
test_gender_detection.py
Standalone gender detection test with majority vote for reliability.
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
            votes = {"Male": 0, "Female": 0}

            # take 15 frames and vote
            for i in range(15):
                ret, frame = cap.read()
                if not ret:
                    continue
                gender, conf = detector.detect(frame)
                if gender:
                    votes[gender] += 1
                time.sleep(0.1)

            # result
            if votes["Male"] == 0 and votes["Female"] == 0:
                print("[WAITING]  No face detected")
            else:
                result = "Male" if votes["Male"] > votes["Female"] else "Female"
                customer = "Customer A" if result == "Female" else "Customer B"
                print(f"[DETECTED] {result} "
                      f"(Female:{votes['Female']} Male:{votes['Male']}) "
                      f"→ deliver to {customer}")

    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
