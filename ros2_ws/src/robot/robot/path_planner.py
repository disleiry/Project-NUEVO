"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions. The Robot class calls compute_velocity() from its own
navigation thread.

To use a planner in your navigation code, just instantiate it and call
compute_velocity() with the current pose and remaining waypoints.
"""

from __future__ import annotations

from collections.abc import Callable
import math
import numpy as np

# =============================================================================
# Base class
# =============================================================================

class PathPlanner:
    """
    Abstract base for path planning algorithms.

    Subclasses implement compute_velocity() and optionally get_obstacles().
    """

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        """
        Return (linear, angular) velocity command.

          pose       — (x, y, theta_rad) in any consistent unit
          waypoints  — remaining waypoints in the same unit, nearest first
          max_linear — maximum forward speed in that unit/s

        Returns (linear, angular_rad_s).
        """
        raise NotImplementedError

    def get_obstacles(self) -> list:
        """
        Return a list of obstacle positions in the robot's frame.
        Override when a 2D lidar topic is available.
        """
        return []


# =============================================================================
# Pure Pursuit
# =============================================================================

class PurePursuitPlanner(PathPlanner):
    """
    Pure-pursuit path follower for differential drive.

    Steers toward a lookahead point on the path. Works well for smooth
    curves. The lookahead_dist controls the trade-off between responsiveness
    (small) and smoothness (large).

    Parameters:
        lookahead_dist — how far ahead on the path to aim at (same units as pose)
        max_angular    — maximum angular rate (rad/s)
    """

    def __init__(
        self,
        lookahead_dist: float = 150,
        max_angular: float = 2.0,
        goal_tolerance: float = 20.0,
    ) -> None:
        self._lookahead  = lookahead_dist
        self._max_angular = max_angular
        self.goal_tolerance = goal_tolerance

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        dx = tx - x
        dy = ty - y

        # Transform the lookahead point into the robot frame.
        x_r = math.cos(theta) * dx + math.sin(theta) * dy
        y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        dist = math.hypot(x_r, y_r)

        if dist < 1e-6:
            return 0.0, 0.0

        # Standard pure-pursuit curvature for a differential-drive robot.
        curvature = 2.0 * y_r / (dist * dist)

        # Slow down for high-curvature turns. The lookahead-scaled term is
        # dimensionless and gives a smooth transition between straight driving
        # and tight cornering.
        forward_scale = max(0.0, x_r / dist)
        curvature_scale = 1.0 + abs(curvature) * self._lookahead
        linear = max_linear * forward_scale / curvature_scale

        if linear <= 1e-6:
            angular = self._max_angular * math.tanh(y_r / max(self._lookahead, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self._max_angular:
            angular = math.copysign(self._max_angular, angular)
            linear = min(linear, abs(angular / curvature)) if abs(curvature) > 1e-6 else linear

        return linear, angular

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        """
        Return the first ordered waypoint beyond the lookahead distance.

        The caller is expected to pass the remaining path in route order. That
        avoids Euclidean nearest-point jumps around corners, which otherwise
        make the lookahead target chatter between the incoming and outgoing
        path segments.
        """
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]
    
    def CurrentTargetReached(self, target_x, target_y, x, y):
        dist_to_target = np.hypot(target_x - x, target_y - y)
        return dist_to_target < self.goal_tolerance


# =============================================================================
# APF
# =============================================================================

class APFPlanner(PathPlanner):
    """
    Artificial Potential Fields planner.

    Combines an attractive force toward the goal with repulsive forces from
    obstacles. Obstacle data comes from get_obstacles(), which reads the
    lidar topic once it is available.

    This first version is usable today with caller-provided robot-frame
    obstacles. A future lidar/object-detection node can feed live obstacles
    through the Robot obstacle-provider API without changing the planner.
    """

    def __init__(
        self,
        lookahead_dist: float = 200,
        max_linear: float = 200,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        heading_gain: float = 2.0,
        obstacle_provider: Callable[[], list[tuple[float, float]]] | None = None,
    ) -> None:
        self._lookahead = lookahead_dist
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._rep_gain = repulsion_gain
        self._rep_range = repulsion_range
        self.goal_tolerance = goal_tolerance
        self._attr_gain = attraction_gain
        self._heading_gain = heading_gain
        self._obstacle_provider = obstacle_provider

    def compute_velocity(
        self,
        pose: tuple[float, float, float],
        waypoints: list[tuple[float, float]],
        max_linear: float,
    ) -> tuple[float, float]:
        x, y, theta = pose
        tx, ty = self._lookahead_point(x, y, waypoints)
        dx = tx - x
        dy = ty - y

        goal_x_r = math.cos(theta) * dx + math.sin(theta) * dy
        goal_y_r = -math.sin(theta) * dx + math.cos(theta) * dy
        goal_dist = math.hypot(goal_x_r, goal_y_r)
        if goal_dist < 1e-6:
            return 0.0, 0.0

        attr_x = self._attr_gain * goal_x_r / goal_dist
        attr_y = self._attr_gain * goal_y_r / goal_dist

        rep_x = 0.0
        rep_y = 0.0
        nearest_obstacle = self._rep_range
        for obs_x_r, obs_y_r in self.get_obstacles():
            dist = math.hypot(obs_x_r, obs_y_r)
            if dist < 1e-6 or dist >= self._rep_range:
                continue
            nearest_obstacle = min(nearest_obstacle, dist)
            repulse = self._rep_gain * ((1.0 / dist) - (1.0 / self._rep_range)) / (dist * dist)
            rep_x += repulse * (-obs_x_r / dist)
            rep_y += repulse * (-obs_y_r / dist)

        force_x = attr_x + rep_x
        force_y = attr_y + rep_y
        if math.hypot(force_x, force_y) < 1e-6:
            return 0.0, 0.0

        heading_error = math.atan2(force_y, force_x)
        forward_scale = max(0.0, math.cos(heading_error))
        linear_limit = min(float(max_linear), self._max_linear)
        goal_scale = min(1.0, goal_dist / max(self.goal_tolerance * 2.0, 1e-6))
        linear = linear_limit * forward_scale * goal_scale
        if nearest_obstacle < self._rep_range:
            linear *= max(0.0, min(1.0, nearest_obstacle / self._rep_range))
        if force_x <= 0.0:
            linear = 0.0

        angular = self._heading_gain * heading_error
        angular = max(-self._max_angular, min(self._max_angular, angular))
        return linear, angular

    def get_obstacles(self) -> list[tuple[float, float]]:
        if self._obstacle_provider is None:
            return []
        return list(self._obstacle_provider())

    def _lookahead_point(
        self, x: float, y: float, waypoints: list[tuple[float, float]]
    ) -> tuple[float, float]:
        for wx, wy in waypoints:
            if math.hypot(wx - x, wy - y) >= self._lookahead:
                return wx, wy
        return waypoints[-1]


class PurePursuitPlannerWithAvoidance(PathPlanner):
    def __init__(self,
            lookahead_distance: float=100.0,
            max_linear_speed: float=130.0,
            max_angular_speed: float=1.0,
            goal_tolerance: float=20.0,
            obstacles_range: float=400.0,
            view_angle: float=np.pi/2,
            safe_dist: float=150.0,
            avoidance_delay: int=200,
            offset: float=120.0,
            x_L: float=0.0,
            lane_width: float=500.0,
            alpha_Ld: float=0.8,
            obstacle_avoidance: bool = True,
            ):
        self.Ld = lookahead_distance
        self.raw_LD = lookahead_distance
        self.v_max = max_linear_speed  # mm/s
        self.w_max = max_angular_speed  # rad/s
        self.goal_tolerance = goal_tolerance
        self.obstacles_range = obstacles_range
        self.view_angle = view_angle
        self.safe_dist = safe_dist
        self.alpha_Ld = alpha_Ld
        self.obstacle_avoidance = obstacle_avoidance
        self.x_L = x_L
        self.lane_width = lane_width
        self.offset = offset

        self.avoidance_active = False
        self.avoidance_counter = 0
        self.avoidance_delay = avoidance_delay

        # self.current_lane = 'Center'
        self.current_lane = 'Left'

    def set_path(self, path: list[tuple[float, float]]):
        self.raw_path = path.copy()
        if self.current_lane == 'Center':
            self.remaining_path = path.copy()
        elif self.current_lane == 'Left':
            self.remaining_path = []
            for i in range(len(self.raw_path)):
                x_, y_ = self.raw_path[i]
                self.remaining_path.append((x_-self.offset, y_))
        elif self.current_lane == 'Right':
            self.remaining_path = []
            for i in range(len(self.raw_path)):
                x_, y_ = self.raw_path[i]
                self.remaining_path.append((x_+self.offset, y_))

    def _advance_remaining_path(self,
        x: float,
        y: float,
    ) -> list[tuple[float, float]]:
        
        while len(self.remaining_path) > 1:
            next_x_mm, next_y_mm = self.remaining_path[0]
            if np.hypot(x-next_x_mm, y-next_y_mm) > self.Ld:
                break
            self.remaining_path.pop(0)
            self.raw_path.pop(0)

            if self.avoidance_active:
                self.avoidance_active = False
                self.Ld = self.raw_LD
                self.avoidance_counter = 0

        return self.remaining_path

    def _lookahead_point(self, path, x, y):
        path = np.array(path)
        position = np.array([x, y])

        for i in range(len(path)):
            dist = np.linalg.norm(path[i,:] - position)
            if dist >= self.Ld:
                return path[i]

        return path[-1]
    
    def TargetReached(self, path, x, y):
        if self.avoidance_active:
            return False # in avoidance mode, we don't check goal reached condition to prevent the robot from stopping before reaching the goal due to the added waypoints for obstacle avoidance, which may cause the robot to think it's close enough to the goal when it's actually still far away.
        goal_x, goal_y = path[0]
        dist_to_goal = np.hypot(goal_x - x, goal_y - y)
        return (dist_to_goal < self.goal_tolerance)

    def gen_obstacle_waypoint(self, pose, obstacles_r):
        # Step 1: Obtain current state: Obtain pose and obstacles in robot frame based on your lidar and robot configurations.
        x, y, theta = pose
        if len(obstacles_r) > 0:
            # lidar orientation due to installation is 180 deg rotated from robot forward, so rotate obstacles accordingly.
            obstacles_r = (np.array([[np.cos(np.pi), -np.sin(np.pi)], [np.sin(np.pi), np.cos(np.pi)]]) @ obstacles_r.T).T 
            
            # since some robot parts (e.g., the arm) may cause obstacles to be detected, we can filter out those obstacles behind the lidar.
            obstacles_r = obstacles_r[np.abs(np.arctan2(obstacles_r[:,1],obstacles_r[:,0])) <= self.view_angle,:] # only consider obstacles in front of the robot within 180 deg FOV, which can help prevent the robot from being too conservative by reacting to obstacles behind it that are not in its path.

            # consider the lidar offset from the robot center
            # lidar_offset_mm = 100.0
            # obstacles_r = obstacles_r + np.array([[lidar_offset_mm, 0],])

            # Filter out obstacles outside of detecting range.
            dists = np.linalg.norm(obstacles_r, axis=1)
            obstacles_r = obstacles_r[(dists < self.obstacles_range)]

            # Step 2: Obstacle filtering and path modification
            # Transform obstacles from robot frame to world frame
            obstacles = (np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]) @ obstacles_r.T).T + np.array([[x, y],])

            # Obstacle filtering by lane width.
            obstacles_r = obstacles_r[np.abs(obstacles[:,0]-self.x_L)<self.lane_width,:]
            obstacles = obstacles[np.abs(obstacles[:,0]-self.x_L)<self.lane_width,:]

            # Modify path waypoints that are too close to the obstacles to prevent the robot from trying to track those waypoints and colliding with the obstacles.
            if np.any(np.sqrt(np.sum((np.float64([self.remaining_path[0]])-obstacles)**2, 1)) < self.safe_dist):
                self.remaining_path[0] = ((self.remaining_path[0][0]+self.remaining_path[1][0])/2, (self.remaining_path[0][1]+self.remaining_path[1][1])/2)
                self.raw_path[0] = ((self.raw_path[0][0]+self.raw_path[1][0])/2, (self.raw_path[0][1]+self.raw_path[1][1])/2)

            if (len(obstacles_r) > 0)  and (self.avoidance_counter <= 0):
                # Step 3: Find the cloest obstacle, and decide which lane to switch.
                dists = np.linalg.norm(obstacles_r, axis=1)
                # min_dist = np.min(dists)
                arg_dist = np.argmin(dists)
                closest_pt = obstacles[arg_dist,:] # closest obstacle point in world frame

                change_lane = False
                if (closest_pt[0] < self.x_L and self.current_lane!='Right') or (closest_pt[0] > self.x_L and self.current_lane!='Left'):
                    change_lane = True
                    # reduce lookahead distance to track added waypoints more precisely.
                    self.Ld = self.raw_LD * self.alpha_Ld

                    # keep avoidance active for a few cycles to ensure the robot reacts to the obstacle.
                    self.avoidance_counter = self.avoidance_delay
                    self.avoidance_active = True

                # Generate new waypoints based on the desired waypoints on the center lane.
                if change_lane:
                    self.remaining_path = []
                    for i in range(len(self.raw_path)):
                        x_, y_ = self.raw_path[i]
                        if closest_pt[0] < self.x_L:
                            self.remaining_path.append((x_+self.offset, y_))
                            self.current_lane = 'Right'
                        else:
                            self.remaining_path.append((x_-self.offset, y_))
                            self.current_lane = 'Left'
                    print('Change Lane!!! Current lane is:', self.current_lane)
                    if np.hypot(x-closest_pt[0], y-closest_pt[1]) < (self.safe_dist+self.obstacles_range)/2:
                        print('Too Closed!!!')
                        if self.current_lane == 'Right':
                            self.remaining_path.insert(0, (x+self.offset, y+self.offset/2))
                            self.raw_path.insert(0, (x+self.offset, y+self.offset))
                        elif self.current_lane == 'Left':
                            self.remaining_path.insert(0, (x-self.offset, y+self.offset/2))
                            self.raw_path.insert(0, (x-self.offset, y+self.offset))

        if self.avoidance_counter > 0:
            self.avoidance_counter -= 1

    def compute_velocity(self, pose, obstacles_r: np.nparray):
        # Note that the input obstacle point cloud is in robot frame
        x, y, theta = pose
        self._advance_remaining_path(x,y)
        
        if self.TargetReached(self.remaining_path,x,y):
            return 0.0, 0.0  # Stop if within goal tolerance

        if self.obstacle_avoidance:
            self.gen_obstacle_waypoint(pose, obstacles_r)

        target = self._lookahead_point(self.remaining_path, x, y)
        tx, ty = target

        dx = tx - x
        dy = ty - y

        # Transform to robot frame
        x_r = np.cos(theta) * dx + np.sin(theta) * dy
        y_r = -np.sin(theta) * dx + np.cos(theta) * dy
        dist = math.hypot(x_r, y_r)

        if dist < 1e-6:
            return 0.0, 0.0

        # Standard pure-pursuit curvature for a differential-drive robot.
        curvature = 2.0 * y_r / (dist * dist)

        # Slow down for high-curvature turns. The lookahead-scaled term is
        # dimensionless and gives a smooth transition between straight driving
        # and tight cornering.
        forward_scale = max(0.0, x_r / dist)
        curvature_scale = 1.0 + abs(curvature) * self.Ld
        linear = self.v_max * forward_scale / curvature_scale

        if linear <= 1e-6:
            angular = self.w_max * math.tanh(y_r / max(self.Ld, 1e-6))
            return 0.0, angular

        angular = curvature * linear
        if abs(angular) > self.w_max:
            angular = math.copysign(self.w_max, angular)
            linear = min(linear, abs(angular / curvature)) if abs(curvature) > 1e-6 else linear

        return linear, angular

    def motion(self, pose, v, w, dt):
        x, y, theta = pose
        pose[0] += v * np.cos(theta) * dt
        pose[1] += v * np.sin(theta) * dt
        pose[2] += w * dt
        return pose


# =============================================================================
# Helper
# =============================================================================

def _wrap_angle(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi
