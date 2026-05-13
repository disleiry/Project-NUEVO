import cv2
import time
from robot.robot import Robot
from vision.gender_detection import GenderDetector

def run_test():
    robot = Robot()
    robot.enable_vision()
    detector = GenderDetector()

    print("\n--- VISION TEST STARTING ---")
    print("1. Stand in front of the camera.")
    print("2. Wait for 'person' detection (YOLO).")
    print("3. System will then classify gender.")

    while True:
        frame = robot.get_camera_frame()
        if frame is None: continue

        # Check YOLO for person detection
        person_seen = robot.get_detections("person")
        
        display_frame = frame.copy()
        
        if person_seen:
            # We found a person, now look for gender
            gender, conf = detector.detect(frame)
            
            if gender:
                color = (0, 255, 0) if gender == "Female" else (255, 0, 0)
                label = f"CUSTOMER: {gender} ({conf:.2f})"
                cv2.putText(display_frame, label, (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)
                print(f"[LIVE] {gender} detected - Confidence: {conf:.2f}")
        else:
            cv2.putText(display_frame, "STATUS: No Person in view", (50, 50), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow("Vision Pipeline Test", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_test()
