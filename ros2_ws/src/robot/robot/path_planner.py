"""
path_planner.py — pure-algorithm path planning library
=======================================================
These classes are stateless algorithm helpers. They do NOT own threads or
ROS subscriptions.

``PurePursuitPlanner`` — waypoint-following via lookahead point.
``APFPlanner``         — single-goal Artificial Potential Fields with a
                         live world-frame obstacle cloud (from lidar).
"""

from __future__ import annotations

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

class APFPlanner:
    """
    Artificial Potential Fields planner — single-goal variant.

    Combines an attractive force toward one goal point with repulsive forces
    from a world-frame obstacle cloud supplied by the caller (e.g. from
    ``LidarScan.to_world_frame()``).  No waypoints, no lookahead buffer, no
    obstacle callbacks — the caller decides what obstacles are relevant.
    """

    def __init__(
        self,
        max_linear: float = 200.0,
        max_angular: float = 2.0,
        repulsion_gain: float = 500.0,
        repulsion_range: float = 300.0,
        goal_tolerance: float = 20.0,
        attraction_gain: float = 1.0,
        heading_gain: float = 2.0,
        force_ema_alpha: float = 0.35,
        stop_dist: float = 80.0,
    ) -> None:
        self._max_linear  = max_linear
        self._max_angular = max_angular
        self._rep_gain    = repulsion_gain
        self._rep_range   = repulsion_range
        self.goal_tolerance = goal_tolerance
        self._attr_gain   = attraction_gain
        self._heading_gain = heading_gain
        self._force_alpha = float(force_ema_alpha)
        self._stop_dist   = float(stop_dist)

        # EMA state for desired heading — None until first call so initial
        # heading is set directly rather than blended from 0.0.
        self._desired_heading: float | None = None

    def navigate_to_goal(
        self,
        pose: tuple[float, float, float],
        goal: tuple[float, float],
        obstacles: np.ndarray,
    ) -> tuple[float, float]:
        """
        Return ``(linear_mm_s, angular_rad_s)`` toward *goal*, avoiding *obstacles*.

        Parameters
        ----------
        pose      : ``(x_mm, y_mm, theta_rad)`` in world frame
        goal      : ``(x_mm, y_mm)`` target position in world frame
        obstacles : ``(N, 2)`` point array or ``(N, 3)`` disk array
                    in world-frame millimetres. For disks, columns are
                    ``x_mm, y_mm, radius_mm``.
        """
        px, py, theta = pose
        gx, gy = goal

        dx = gx - px
        dy = gy - py
        dist_to_goal = math.hypot(dx, dy)

        if dist_to_goal <= self.goal_tolerance:
            return 0.0, 0.0

        # Attractive force — capped so gain stays bounded far from goal
        attr_scale = self._attr_gain * min(dist_to_goal, self._rep_range)
        attr_x = attr_scale * dx / dist_to_goal
        attr_y = attr_scale * dy / dist_to_goal

        # Repulsive force — vectorised over obstacle cloud.
        #
        # The classical inverse-distance APF gradient becomes vanishingly small
        # when applied directly in millimetres. Use a bounded proximity falloff
        # instead so repulsion remains meaningful at robot-scale distances.
        rep_x = 0.0
        rep_y = 0.0
        tan_x = 0.0
        tan_y = 0.0
        obstacle_scale = 1.0
        obs = np.asarray(obstacles)
        if obs.ndim == 2 and obs.shape[0] > 0:
            centers = obs[:, :2]
            radii = obs[:, 2] if obs.shape[1] >= 3 else np.zeros(obs.shape[0], dtype=float)
            fx = px - centers[:, 0]
            fy = py - centers[:, 1]
            center_dists = np.sqrt(fx * fx + fy * fy)
            boundary_dists = np.maximum(center_dists - radii, 0.0)
            in_range = (center_dists > 1e-6) & (boundary_dists < self._rep_range)
            if np.any(in_range):
                d = np.maximum(boundary_dists[in_range], 1e-6)
                proximity = 1.0 - (d / self._rep_range)
                mag = self._rep_gain * proximity * proximity
                rep_x = float(np.sum(mag * fx[in_range] / center_dists[in_range]))
                rep_y = float(np.sum(mag * fy[in_range] / center_dists[in_range]))

                # Add a tangential escape component around each obstacle.
                # This breaks the head-on APF local minimum where a centered
                # obstacle only produces a backward force and the robot stalls.
                ux = fx[in_range] / center_dists[in_range]
                uy = fy[in_range] / center_dists[in_range]
                left_tx = -uy
                left_ty = ux
                right_tx = uy
                right_ty = -ux
                goal_ux = dx / dist_to_goal
                goal_uy = dy / dist_to_goal
                choose_left = (left_tx * goal_ux + left_ty * goal_uy) >= (right_tx * goal_ux + right_ty * goal_uy)
                tangent_x = np.where(choose_left, left_tx, right_tx)
                tangent_y = np.where(choose_left, left_ty, right_ty)
                tangent_mag = 0.50 * self._rep_gain * proximity * proximity
                tan_x = float(np.sum(tangent_mag * tangent_x))
                tan_y = float(np.sum(tangent_mag * tangent_y))

                nearest_boundary = float(np.min(boundary_dists[in_range]))
                obstacle_scale = min(1.0, nearest_boundary / self._rep_range)

        force_x = attr_x + rep_x + tan_x
        force_y = attr_y + rep_y + tan_y

        if math.hypot(force_x, force_y) < 1e-6:
            return 0.0, 0.0

        # EMA on the desired heading angle (not force components) so opposite
        # avoidance and attractive vectors can't cancel inside the filter.
        desired_raw = math.atan2(force_y, force_x)
        if self._desired_heading is None:
            self._desired_heading = desired_raw
        else:
            delta = (desired_raw - self._desired_heading + math.pi) % (2.0 * math.pi) - math.pi
            self._desired_heading = (self._desired_heading + self._force_alpha * delta + math.pi) % (2.0 * math.pi) - math.pi
        desired = self._desired_heading
        ang_err  = (desired - theta + math.pi) % (2.0 * math.pi) - math.pi

        forward_scale = max(0.0, math.cos(ang_err))
        goal_scale    = min(1.0, dist_to_goal / max(2.0 * self.goal_tolerance, 1.0))
        linear  = self._max_linear * forward_scale * goal_scale * obstacle_scale

        # Hard stop: within stop_dist of an obstacle surface, freeze forward motion.
        # Angular is still allowed so the robot can steer clear before resuming.
        if obstacle_scale < (self._stop_dist / max(self._rep_range, 1e-6)):
            linear = 0.0

        angular = self._heading_gain * ang_err
        angular = max(-self._max_angular, min(self._max_angular, angular))

        return linear, angular


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
