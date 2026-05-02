from __future__ import annotations

from collections.abc import Callable
import math
import time
import threading

from rclpy.node import Node

import time as _time

from robot.sensor_fusion import OrientationComplementaryFilter, PositionComplementaryFilter, SensorFusion
try:
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except (ImportError, ModuleNotFoundError):
    class ReliabilityPolicy:
        BEST_EFFORT = "best_effort"

    class HistoryPolicy:
        KEEP_LAST = "keep_last"

    class QoSProfile:
        def __init__(self, reliability=None, history=None, depth=1) -> None:
            self.reliability = reliability
            self.history = history
            self.depth = depth

import numpy as np

from bridge_interfaces.msg import (
    DCEnable,
    DCHome,
    DCPid,
    DCPidReq,
    DCPidSet,
    DCResetPosition,
    DCSetPosition,
    DCSetPwm,
    DCSetVelocity,
    DCStateAll,
    IOSetLed,
    IOSetNeopixel,
    IOOutputState,
    SensorKinematics,
    ServoEnable,
    ServoSet,
    ServoStateAll,
    StepConfig,
    StepConfigReq,
    StepConfigSet,
    StepEnable,
    StepHome,
    StepMove,
    StepStateAll,
    SysOdomParamReq,
    SysOdomParamRsp,
    SysOdomParamSet,
    SysOdomReset,
    SystemConfig,
    SystemDiag,
    SystemInfo,
    SystemPower,
    SystemState,
    IOInputState,
    FusedPose,
    LidarWorldPoints,
    SensorImu,
    TagDetectionArray,
)
try:
    from bridge_interfaces.msg import VisionDetectionArray
except (ImportError, ModuleNotFoundError):
    class VisionDetectionArray:  # test-stub fallback
        detections = ()
        image_width = 0
        image_height = 0

from bridge_interfaces.srv import SetFirmwareState
try:
    from sensor_msgs.msg import LaserScan
except (ImportError, ModuleNotFoundError):
    class LaserScan:  # test-stub fallback
        ranges = ()
        angle_min = 0.0
        angle_max = 0.0
        angle_increment = 0.0
        range_min = 0.0
        range_max = 0.0

from robot.hardware_map import (
    BUTTON_COUNT,
    DCMotorMode,
    DCPidLoop,
    DEFAULT_NAV_HZ,
    LEDMode,
    LIMIT_COUNT,
    Motor,
    StepMoveType,
    StepperMotionState,
)

from robot.robot_impl.hardware import FirmwareState, HardwareMixin
from robot.robot_impl.sensors import SensorsMixin
from robot.robot_impl.navigation import Unit, MotionHandle, NavigationMixin
from robot.robot_impl.legacy import LegacyMixin


# =============================================================================
# Robot
# =============================================================================

class Robot(HardwareMixin, SensorsMixin, NavigationMixin, LegacyMixin):
    """
    Layer-1 abstraction over all bridge ROS topics and services.

    Construct this inside a running ROS node and pass the node reference in.
    All length/velocity inputs and outputs respect the unit system set at
    construction time (default: mm). Angles are always in degrees for the
    public API; internally radians are used.

    Motor IDs are 1-based (1–4). Stepper IDs are 1-based (1–4).
    Servo channels are 1-based (1–16).
    Button IDs are 1-based (1–10). Limit IDs are 1-based (1–8).

    Sensor subscriptions are opt-in — call the relevant enable_*() method
    before using the corresponding API:
        robot.enable_lidar()   — subscribe to /scan, populate get_obstacles()
        robot.enable_gps()     — subscribe to /tag_detections, populate GPS fusion
        robot.enable_imu()     — subscribe to /sensor_imu, populate orientation fusion
        robot.enable_vision()  — subscribe to /vision/detections, populate get_detections()

    Hardware defaults match firmware/arduino/src/config.h:
        WHEEL_DIAMETER_MM = 74.0
        WHEEL_BASE_MM     = 333.0
        ENCODER_PPR       = 1440  (4× mode)
        DEFAULT_LEFT_WHEEL_MOTOR  = 1
        DEFAULT_RIGHT_WHEEL_MOTOR = 2
        INITIAL_THETA_DEG = 90.0
        IMU_Z_DOWN        = False  (Z-axis points up)
    """

    WHEEL_DIAMETER_MM: float = 74.0
    WHEEL_BASE_MM:     float = 333.0
    ENCODER_PPR:       int   = 1440
    INITIAL_THETA_DEG: float = 90.0
    IMU_Z_DOWN:        bool  = False
    DEFAULT_LEFT_WHEEL_MOTOR:  int = int(Motor.DC_M1)
    DEFAULT_RIGHT_WHEEL_MOTOR: int = int(Motor.DC_M2)
    DEFAULT_LEFT_WHEEL_DIR_INVERTED:  bool = False
    DEFAULT_RIGHT_WHEEL_DIR_INVERTED: bool = True
    POSITION_ALPHA    = 0.10   # complementary filter GPS weight for position fusion
    ORIENTATION_ALPHA = 0.0    # complementary filter IMU weight for orientation fusion
    TAG_X_OFFSET_MM   = 0.0   # ArUco tag x in robot body frame (mm, +x = forward)
    TAG_Y_OFFSET_MM   = 0.0   # ArUco tag y in robot body frame (mm, +y = left)

    LIDAR_MOUNT_X_MM:      float = 0.0    # lidar x in robot body frame (mm, +x = forward)
    LIDAR_MOUNT_Y_MM:      float = 0.0    # lidar y in robot body frame (mm, +y = left)
    LIDAR_MOUNT_THETA_DEG: float = 0.0   # lidar heading offset relative to robot forward (CCW+)

    LIDAR_RANGE_MIN_MM:  float = 150.0   # discard lidar returns closer than this (mm)
    LIDAR_RANGE_MAX_MM:  float = 6000.0  # discard lidar returns farther than this (mm)
    LIDAR_FOV_MIN_DEG:   float = -180.0  # FOV window min (deg, 0° = robot +x forward, CCW+)
    LIDAR_FOV_MAX_DEG:   float =  180.0  # FOV window max

    _SERVO_MIN_US: int = 1000
    _SERVO_MAX_US: int = 2000
    _SHUTDOWN_SETTLE_S: float = 0.10

    def __init__(
        self,
        node: Node,
        unit: Unit = Unit.MM,
        wheel_diameter_mm: float = WHEEL_DIAMETER_MM,
        wheel_base_mm: float = WHEEL_BASE_MM,
        encoder_ppr: int = ENCODER_PPR,
    ) -> None:
        self._node             = node
        self._unit             = unit
        self._wheel_diameter   = wheel_diameter_mm
        self._wheel_base       = wheel_base_mm
        self._encoder_ppr      = encoder_ppr
        self._ticks_per_mm     = encoder_ppr / (math.pi * wheel_diameter_mm)
        self._initial_theta_deg = self.INITIAL_THETA_DEG
        self._left_wheel_motor  = self.DEFAULT_LEFT_WHEEL_MOTOR
        self._right_wheel_motor = self.DEFAULT_RIGHT_WHEEL_MOTOR
        self._left_wheel_dir_inverted  = self.DEFAULT_LEFT_WHEEL_DIR_INVERTED
        self._right_wheel_dir_inverted = self.DEFAULT_RIGHT_WHEEL_DIR_INVERTED
        self._odom_user_configured = False
        self._odom_confirm_event = threading.Event()
        self._lock             = threading.Lock()

        # ── Cached firmware state ─────────────────────────────────────────────
        self._sys_state:  int            = 0
        self._sys_power:  SystemPower    = None
        self._sys_info:   SystemInfo     = None
        self._sys_config: SystemConfig   = None
        self._sys_diag:   SystemDiag     = None
        self._dc_state:   DCStateAll     = None
        self._dc_pid_cache: dict[tuple[int, int], DCPid] = {}
        self._step_state: StepStateAll   = None
        self._step_config_cache: dict[int, StepConfig] = {}
        self._servo_state: ServoStateAll = None
        self._io_output_state: IOOutputState = None
        self._imu:        SensorImu      = None
        self._imu_z_down:         bool        = self.IMU_Z_DOWN
        self._ahrs_heading:       float | None = None
        self._odom_reset_pending: bool       = False
        self._fused_theta:        float      = math.radians(self.INITIAL_THETA_DEG)
        self._orientation_fusion: OrientationComplementaryFilter = OrientationComplementaryFilter(alpha=self.ORIENTATION_ALPHA)
        self._fusion: SensorFusion = self._orientation_fusion
        self._pose:    tuple = (0.0, 0.0, 0.0)   # x_mm, y_mm, theta_rad (raw odometry)

        # ── GPS position fusion ───────────────────────────────────────────────
        self._tracked_tag_id:      int   = -1
        self._gps_x_mm:            float = 0.0
        self._gps_y_mm:            float = 0.0
        self._gps_last_time:       float = 0.0
        self._gps_timeout_s:       float = 1.0
        self._gps_offset_x_mm:     float = 0.0
        self._gps_offset_y_mm:     float = 0.0
        self._tag_body_offset_x_mm: float = self.TAG_X_OFFSET_MM
        self._tag_body_offset_y_mm: float = self.TAG_Y_OFFSET_MM

        # ── Lidar ─────────────────────────────────────────────────────────────
        self._lidar_mount_x_mm:      float = self.LIDAR_MOUNT_X_MM
        self._lidar_mount_y_mm:      float = self.LIDAR_MOUNT_Y_MM
        self._lidar_mount_theta_rad: float = math.radians(self.LIDAR_MOUNT_THETA_DEG)
        self._lidar_range_min_mm:    float = self.LIDAR_RANGE_MIN_MM
        self._lidar_range_max_mm:    float = self.LIDAR_RANGE_MAX_MM
        self._lidar_fov_min_rad:     float = math.radians(self.LIDAR_FOV_MIN_DEG)
        self._lidar_fov_max_rad:     float = math.radians(self.LIDAR_FOV_MAX_DEG)
        self._lidar_world_pub        = None   # set by start_lidar_world_publisher
        self._lidar_world_timer      = None   # ROS timer, fires on spin thread

        # ── Fused position ────────────────────────────────────────────────────
        self._fused_x_mm:    float = 0.0
        self._fused_y_mm:    float = 0.0
        self._pos_fusion:    PositionComplementaryFilter = PositionComplementaryFilter(alpha=self.POSITION_ALPHA)
        self._vel:     tuple = (0.0, 0.0, 0.0)   # vx_mm_s, vy_mm_s, vtheta_rad_s

        # ── IO ────────────────────────────────────────────────────────────────
        self._buttons: int  = 0
        self._limits:  int  = 0
        self._button_edges: int  = 0
        self._limit_edges:  int  = 0
        self._have_io_input: bool = False

        # ── Obstacles / vision ────────────────────────────────────────────────
        self._obstacles_mm: np.ndarray = np.float64([])
        self._obstacle_provider: Callable[[], list[tuple[float, float]]] | None = None
        self._obstacle_avoidance_planner = None   # legacy
        self._vision_detections: list[dict[str, object]] = []
        self._vision_image_size: tuple[int, int] = (0, 0)
        self._vision_last_time: float = 0.0

        # ── Trajectory ────────────────────────────────────────────────────────
        self._odom_traj:  list[tuple[float, float]] = []
        self._fused_traj: list[tuple[float, float]] = []

        # ── Events ────────────────────────────────────────────────────────────
        self._pose_event:       threading.Event = threading.Event()
        self._odom_reset_event: threading.Event = threading.Event()
        self._button_events: dict[int, threading.Event] = {}
        self._limit_events:  dict[int, threading.Event] = {}

        # ── Navigation ────────────────────────────────────────────────────────
        self._nav_thread: threading.Thread = None
        self._nav_cancel: threading.Event  = threading.Event()
        self._nav_done:   threading.Event  = threading.Event()

        # ── Publishers ────────────────────────────────────────────────────────
        self._dc_vel_pub   = node.create_publisher(DCSetVelocity,  '/dc_set_velocity',  10)
        self._dc_pwm_pub   = node.create_publisher(DCSetPwm,       '/dc_set_pwm',       10)
        self._dc_pos_pub   = node.create_publisher(DCSetPosition,  '/dc_set_position',  10)
        self._dc_en_pub    = node.create_publisher(DCEnable,       '/dc_enable',        10)
        self._dc_home_pub  = node.create_publisher(DCHome,         '/dc_home',          10)
        self._dc_rst_pub   = node.create_publisher(DCResetPosition,'/dc_reset_position',10)
        self._dc_pid_set   = node.create_publisher(DCPidSet,       '/dc_pid_set',       10)
        self._dc_pid_req   = node.create_publisher(DCPidReq,       '/dc_pid_req',       10)
        self._step_en_pub  = node.create_publisher(StepEnable,     '/step_enable',      10)
        self._step_mv_pub  = node.create_publisher(StepMove,       '/step_move',        10)
        self._step_hm_pub  = node.create_publisher(StepHome,       '/step_home',        10)
        self._step_cfg_req_pub = node.create_publisher(StepConfigReq, '/step_config_req', 10)
        self._step_cfg_pub = node.create_publisher(StepConfigSet,  '/step_config_set',  10)
        self._srv_en_pub   = node.create_publisher(ServoEnable,    '/servo_enable',     10)
        self._srv_set_pub  = node.create_publisher(ServoSet,       '/servo_set',        10)
        self._led_pub      = node.create_publisher(IOSetLed,       '/io_set_led',       10)
        self._neo_pub      = node.create_publisher(IOSetNeopixel,  '/io_set_neopixel',  10)
        self._odom_param_req_pub = node.create_publisher(SysOdomParamReq, '/sys_odom_param_req', 10)
        self._odom_param_pub     = node.create_publisher(SysOdomParamSet, '/sys_odom_param_set', 10)
        self._odom_pub     = node.create_publisher(SysOdomReset,   '/sys_odom_reset',   10)
        self._pub_fused_pose = node.create_publisher(FusedPose,    '/fused_pose',       10)

        # ── Always-on subscriptions ───────────────────────────────────────────
        node.create_subscription(SystemState,      '/sys_state',          self._on_sys_state,      10)
        node.create_subscription(SystemPower,      '/sys_power',          self._on_sys_power,      10)
        node.create_subscription(SystemInfo,       '/sys_info_rsp',       self._on_sys_info,       10)
        node.create_subscription(SystemConfig,     '/sys_config_rsp',     self._on_sys_config,     10)
        node.create_subscription(SystemDiag,       '/sys_diag_rsp',       self._on_sys_diag,       10)
        node.create_subscription(DCPid,            '/dc_pid_rsp',         self._on_dc_pid,         10)
        node.create_subscription(DCStateAll,       '/dc_state_all',       self._on_dc_state,       10)
        node.create_subscription(StepConfig,       '/step_config_rsp',    self._on_step_config,    10)
        node.create_subscription(StepStateAll,     '/step_state_all',     self._on_step_state,     10)
        node.create_subscription(ServoStateAll,    '/servo_state_all',    self._on_servo_state,    10)
        node.create_subscription(SensorKinematics, '/sensor_kinematics',  self._on_kinematics,     10)
        node.create_subscription(IOInputState,     '/io_input_state',     self._on_io_input,       10)
        node.create_subscription(IOOutputState,    '/io_output_state',    self._on_io_output,      10)
        node.create_subscription(SysOdomParamRsp,  '/sys_odom_param_rsp', self._on_odom_param_rsp, 10)

        # Sensor subscriptions are opt-in — call enable_lidar(), enable_gps(),
        # enable_imu(), or enable_vision() to activate them.

        # ── Service clients ───────────────────────────────────────────────────
        self._set_state_client = node.create_client(SetFirmwareState, '/set_firmware_state')

        # Sync local odometry cache with live firmware snapshot.
        self.request_odometry_parameters()
