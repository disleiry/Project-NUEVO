import time
import math
import statistics

from robot.robot import Robot
from robot.hardware_map import (
    POSITION_UNIT,
    LIDAR_MOUNT_X_MM,
    LIDAR_MOUNT_Y_MM,
    LIDAR_MOUNT_THETA_DEG,
    LIDAR_RANGE_MIN_MM,
    LIDAR_FOV_DEG
)

def get_front_wall_distance_mm(robot: Robot) -> float:
    """
    Finds the closest obstacle directly in front of the robot using existing tracks.
    Returns distance in mm, or -1.0 if the path is clear.
    """
    obstacle_tracks = robot.get_obstacle_tracks()
    if not obstacle_tracks:
        return -1.0

    # Use raw odometry for the standalone script
    rx, ry, rtheta_deg = robot.get_odometry_pose()
    rtheta_rad = math.radians(rtheta_deg)
    
    front_distances = []
    
    # Check every tracked obstacle
    for track in obstacle_tracks:
        ox = float(track["x"])
        oy = float(track["y"])
        radius = float(track["radius"])
        
        # Calculate delta from robot to obstacle
        dx = ox - rx
        dy = oy - ry
        
        # Rotate coordinates to the robot's local perspective (X is forward, Y is left)
        local_x = dx * math.cos(rtheta_rad) + dy * math.sin(rtheta_rad)
        local_y = -dx * math.sin(rtheta_rad) + dy * math.cos(rtheta_rad)
        
        # If the obstacle is in front (local_x > 0) and within a +/- 150mm lateral corridor
        if local_x > 0 and abs(local_y) < (150.0 + radius):
            # Distance to the edge of the obstacle
            dist = local_x - radius
            if dist > 0:
                front_distances.append(dist)
                
    if not front_distances:
        return -1.0
        
    return min(front_distances)


def get_robust_wall_distance(robot: Robot, num_samples: int = 5, delay_s: float = 0.1) -> float:
    """
    Takes multiple Lidar frames, throws away invalid (-1.0) readings, 
    and returns the median of the valid readings to reject outliers.
    """
    valid_distances = []
    
    print(f"\n[NAV] Taking {num_samples} Lidar samples...")
    for i in range(num_samples):
        dist = get_front_wall_distance_mm(robot)
        
        # Only keep the reading if it actually saw a wall
        if dist > 0.0:
            valid_distances.append(dist)
            
        # Keep the ROS heartbeat alive and wait for the Lidar to spin again
        robot.wait_for_pose_update(timeout=delay_s)
        
    # If every single frame was empty, the path is genuinely clear
    if not valid_distances:
        print("[NAV] All samples returned empty (-1.0).")
        return -1.0
        
    # The median ignores extreme high/low sensor glitches
    final_distance = statistics.median(valid_distances)
    print(f"[NAV] Valid samples: {valid_distances}")
    print(f"[NAV] Robust median distance: {final_distance:.0f} mm")
    
    return final_distance


# This is the exact entry point your ROS node looks for
def run(robot: Robot) -> None:
    print("[INIT] Configuring Lidar...")
    robot.set_unit(POSITION_UNIT)
    
    # We must enable the Lidar and give it the filter settings
    robot.enable_lidar()
    robot.set_lidar_mount(
        x_mm=LIDAR_MOUNT_X_MM,
        y_mm=LIDAR_MOUNT_Y_MM,
        theta_deg=LIDAR_MOUNT_THETA_DEG,
    )
    
    # CRITICAL FIX: Set max range to 2000.0 so we can see far away walls!
    robot.set_lidar_filter(
        range_min_mm=LIDAR_RANGE_MIN_MM,
        range_max_mm=2000.0,  
        fov_deg=LIDAR_FOV_DEG,
    )
    
    robot.start_lidar_world_publisher()
    robot.reset_odometry()
    
    print("[INIT] Waiting 2 seconds for Lidar to spin up...")
    time.sleep(2.0)
    
    print("=========================================")
    print("  Robust Lidar Distance Monitor Active")
    print("  Press Ctrl+C to stop")
    print("=========================================")
    
    # Continuous monitoring loop
    while True:
        # Call the multi-frame robust function
        distance_to_wall = get_robust_wall_distance(robot, num_samples=5, delay_s=0.1)
        
        if distance_to_wall > 0:
            print(f"[LIDAR] FINAL CONFIRMED DISTANCE: {distance_to_wall:.0f} mm")
        else:
            print(f"[LIDAR] Path clear (-1.0)")
            
        time.sleep(1.5)
