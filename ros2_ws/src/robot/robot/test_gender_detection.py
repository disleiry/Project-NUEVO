"""
test_gender_detection.py
Standalone gender detection test with majority vote.
Takes 10 frames, counts votes, picks the winner.
"""

import sys
import time
import cv2

sys.path.insert(0, '/ros2_ws/src/vision')
from vision.gender_detection import GenderDetector

FRAMES       = 10     # number of frames to sample
FRAME_DELAY  = 0.2    # seconds between frames
MIN_CONF     = 0.70   # ignore detections below this confidence


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

            for i in range(FRAMES):
                ret, frame = cap.read()
                if not ret:
                    continue
                gender, conf = detector.detect(frame)
                if gender and conf >= MIN_CONF:
                    votes[gender] += 1
                    print(f"  frame {i+1}/{FRAMES}: {gender} ({conf:.2f})")
                else:
                    print(f"  frame {i+1}/{FRAMES}: no detection")
                time.sleep(FRAME_DELAY)

            total = votes["Male"] + votes["Female"]
            if total == 0:
                print("[WAITING]  No face detected — trying again\n")
                continue

            result   = "Male" if votes["Male"] > votes["Female"] else "Female"
            customer = "Customer B" if result == "Male" else "Customer A"
            print(f"\n[RESULT] {result} "
                  f"(Female:{votes['Female']} Male:{votes['Male']}) "
                  f"→ {customer}\n")

    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        cap.release()


if __name__ == "__main__":
    main()
