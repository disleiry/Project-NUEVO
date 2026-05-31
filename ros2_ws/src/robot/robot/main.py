import time
from robot import Robot  # Adjust this import based on your exact codebase structure

# IMPORTANT: Paste your get_front_wall_distance_mm() function right here 
# if you were previously defining it inside your main script rather than importing it.

def main():
    print("[INIT] Connecting to robot...")
    robot = Robot()
    
    # Give the Lidar sensor time to physically spin up and populate initial tracks
    print("[INIT] Waiting 2 seconds for Lidar to spin up...")
    time.sleep(2.0)
    
    print("=========================================")
    print("  Lidar Distance Monitor Active")
    print("  Press Ctrl+C to stop")
    print("=========================================")
    
    try:
        while True:
            # If your Robot class requires manual ticking for ROS callbacks, 
            # you may need to call something like robot.update() or robot.spin() here.
            
            distance_to_wall = get_front_wall_distance_mm(robot)
            
            print(f"[LIDAR] Front wall distance: {distance_to_wall} mm")
            
            # Pause for 2 seconds before measuring again
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        print("\n[EXIT] Ctrl+C detected. Shutting down monitor.")
        robot.stop()

if __name__ == "__main__":
    main()
