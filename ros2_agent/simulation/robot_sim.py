"""2-D differential-drive physics simulation.

No external dependencies — only the Python standard library and asyncio.

Features
--------
* Differential-drive kinematics at 20 Hz
* Simple obstacle-avoidance (collision detection stops the robot)
* P-controller waypoint follower (used when a navigation goal is sent)
* 32-ray LiDAR simulation via ray-casting against axis-aligned rectangles
* Battery drain proportional to movement
* Thread-safe singleton accessed via ``get_sim()``
"""

from __future__ import annotations

import asyncio
import math
import time
from collections import deque
from dataclasses import dataclass, field
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass

# ── Map definition (10 m × 10 m room) ────────────────────────────────────────

MAP_SIZE: float = 10.0  # metres, square room
ROBOT_RADIUS: float = 0.22  # metres

# Obstacles as (x, y, width, height) in metres
OBSTACLES: list[tuple[float, float, float, float]] = [
    (2.0, 1.5, 1.2, 0.4),
    (5.5, 1.0, 0.4, 2.0),
    (1.5, 4.5, 2.0, 0.4),
    (6.5, 3.5, 1.2, 1.2),
    (3.8, 6.5, 0.4, 1.8),
    (7.0, 7.0, 1.5, 0.4),
    (0.5, 7.5, 0.4, 1.5),
    (4.5, 3.5, 1.0, 0.4),
]

# ── State dataclass ───────────────────────────────────────────────────────────


@dataclass
class RobotState:
    """Mutable snapshot of the simulated robot's physical and sensor state."""

    x: float = 1.5
    y: float = 1.5
    theta: float = 0.0  # radians, CCW from +X
    vx: float = 0.0  # m/s forward
    omega: float = 0.0  # rad/s angular
    battery: float = 85.0  # percent
    goal_x: float | None = None
    goal_y: float | None = None
    goal_id: int | None = None
    nav_active: bool = False
    nav_succeeded: bool = False
    path: list[tuple[float, float]] = field(default_factory=list)
    full_path: list[tuple[float, float]] = field(default_factory=list)
    lidar: list[float] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)
    waypoints: list[tuple[float, float]] = field(default_factory=list)
    waypoint_index: int = 0
    velocity_history: list[float] = field(default_factory=list)
    occupancy_grid: list[list[float]] = field(default_factory=list)
    # Camera / detection
    camera_detections: list[dict] = field(default_factory=list)
    camera_depth_row: list[float] = field(default_factory=list)
    # SLAM
    slam_covariance: float = 0.02  # metres² positional uncertainty
    slam_map_cells_seen: int = 0
    # Timing (seconds, measured each tick)
    timing: dict = field(default_factory=dict)


# ── Simulator ─────────────────────────────────────────────────────────────────


class RobotSim:
    """Async differential-drive robot simulation."""

    DT: float = 0.05  # seconds per tick (20 Hz)
    LIDAR_RAYS: int = 90  # 90 rays ≈ realistic budget RPLIDAR
    LIDAR_MAX: float = 6.0  # metres
    NAV_K_ANGLE: float = 2.5  # P-gain for heading error
    NAV_K_SPEED: float = 0.55  # max forward speed
    GOAL_TOLERANCE: float = 0.20

    # Motor dynamics (acceleration limits — makes motion feel physical)
    _MAX_ACCEL: float = 1.5  # m/s² forward acceleration limit
    _MAX_ALPHA: float = 4.0  # rad/s² angular acceleration limit
    _MOTOR_DEADZONE_VX: float = 0.03  # static friction deadzone (m/s command)
    _MOTOR_DEADZONE_OMEGA: float = 0.08  # static friction deadzone (rad/s command)

    # Sensor noise
    _LIDAR_NOISE_SIGMA: float = 0.018  # Gaussian σ on LiDAR ranges (metres)
    _LIDAR_DROPOUT_PROB: float = 0.012  # probability of a ray returning max range (glass/dropout)

    # Obstacle avoidance
    _FRONT_HALF_ANGLE: float = math.pi / 3  # ±60° cone treated as "front"
    _DANGER_DIST: float = 0.55  # slow down below this clearance
    _ESCAPE_TICKS: int = 60  # ticks to run escape manoeuvre (~3 s)
    _STUCK_TICKS: int = 25  # ticks without movement = stuck (~1.25 s)

    # Realism knobs
    _CONTROL_LATENCY_S: float = 0.09  # command transport+controller latency
    _ROLLING_RESISTANCE: float = 0.55  # linear damping coefficient
    _ANGULAR_DAMPING: float = 0.95  # angular damping coefficient
    _SLIP_BASE: float = 0.025  # baseline longitudinal slip
    _SLIP_TURN_GAIN: float = 0.06  # extra slip while turning

    def __init__(self) -> None:
        """Initialise the simulator with default state and a fresh occupancy grid."""
        self.state = RobotState()
        self._cmd_vx: float = 0.0
        self._cmd_omega: float = 0.0
        self._manual_override: bool = False
        self._running: bool = False
        self._task: asyncio.Task | None = None  # type: ignore[type-arg]
        self._cmd_queue: deque[tuple[float, float, float]] = deque()
        self._current_slip: float = 0.0
        self._physics_profile_name: str = "default"
        self._physics_profile_raw: dict = {}
        self._loaded_policy_name: str | None = None
        self._loaded_policy_raw: dict | None = None
        self._robot_profile_name: str | None = None
        self._robot_profile_raw: dict | None = None
        self._robot_profile_payload: dict = {}
        self._robot_visual_style: dict = {}

        # Escape / stuck tracking
        self._escape_countdown: int = 0  # ticks remaining in escape mode
        self._escape_omega: float = 1.5  # direction of escape turn
        self._escape_phase: int = 0  # 0=turn, 1=forward-slide past obstacle
        self._no_move_ticks: int = 0  # consecutive ticks with near-zero movement
        self._last_x: float = self.state.x
        self._last_y: float = self.state.y
        self._consec_escapes: int = 0  # how many escapes from same spot

        # Event log
        self._events: list[dict] = []
        self._log_event("system", "Simulation started")

        # Initialize occupancy grid (100x100 for 10m map = 0.1m resolution)
        self.state.occupancy_grid = [[0.0] * 100 for _ in range(100)]
        self._init_occupancy_grid()

    # ── Public control API ────────────────────────────────────────────────────

    def set_cmd_vel(self, vx: float, omega: float) -> None:
        """Handle a cmd_vel command published by mock ROS 2."""
        target_vx = max(-0.5, min(0.5, vx))
        target_omega = max(-2.0, min(2.0, omega))

        # Static friction / deadzone: tiny commands generally do not move the platform.
        if abs(target_vx) < self._MOTOR_DEADZONE_VX:
            target_vx = 0.0
        if abs(target_omega) < self._MOTOR_DEADZONE_OMEGA:
            target_omega = 0.0

        # Apply control latency so response feels closer to real robot stacks.
        apply_at = time.time() + self._CONTROL_LATENCY_S
        self._cmd_queue.append((apply_at, target_vx, target_omega))
        self._manual_override = True
        self.state.nav_active = False
        self._escape_countdown = 0

    def set_goal(self, x: float, y: float, goal_id: int | None = None) -> None:
        """Handle a navigation goal sent by mock ROS 2."""
        s = self.state
        s.goal_x = max(0.5, min(MAP_SIZE - 0.5, x))
        s.goal_y = max(0.5, min(MAP_SIZE - 0.5, y))
        s.goal_id = goal_id
        s.nav_active = True
        s.nav_succeeded = False
        self._manual_override = False
        self._escape_countdown = 0
        self._escape_phase = 0
        self._no_move_ticks = 0
        self._consec_escapes = 0
        self._log_event("nav_goal", f"Navigation goal set to ({s.goal_x:.1f}, {s.goal_y:.1f})")

    def set_waypoints(self, waypoints: list[tuple[float, float]]) -> None:
        """Set a queue of waypoints to visit sequentially."""
        s = self.state
        clamped = [
            (max(0.5, min(MAP_SIZE - 0.5, wx)), max(0.5, min(MAP_SIZE - 0.5, wy)))
            for wx, wy in waypoints
        ]
        s.waypoints = clamped
        s.waypoint_index = 0
        if clamped:
            self.set_goal(clamped[0][0], clamped[0][1])
            self._log_event("waypoints", f"Waypoint path set with {len(clamped)} points")

    def stop(self) -> None:
        """Kill the async loop."""
        self._running = False
        if self._task is not None:
            self._task.cancel()

    def apply_physics_profile(self, profile_name: str, config: dict) -> dict:
        """Apply a physics profile from uploaded JSON-style config."""
        self._physics_profile_name = profile_name
        self._physics_profile_raw = dict(config)

        def _set_float(key: str, attr: str, lo: float, hi: float) -> None:
            if key not in config:
                return
            try:
                value = float(config[key])
            except (TypeError, ValueError):
                return
            setattr(self, attr, max(lo, min(hi, value)))

        _set_float("max_accel", "_MAX_ACCEL", 0.1, 5.0)
        _set_float("max_alpha", "_MAX_ALPHA", 0.2, 10.0)
        _set_float("control_latency_s", "_CONTROL_LATENCY_S", 0.0, 0.6)
        _set_float("rolling_resistance", "_ROLLING_RESISTANCE", 0.0, 2.0)
        _set_float("angular_damping", "_ANGULAR_DAMPING", 0.0, 3.0)
        _set_float("slip_base", "_SLIP_BASE", 0.0, 0.4)
        _set_float("slip_turn_gain", "_SLIP_TURN_GAIN", 0.0, 0.3)
        _set_float("motor_deadzone_vx", "_MOTOR_DEADZONE_VX", 0.0, 0.2)
        _set_float("motor_deadzone_omega", "_MOTOR_DEADZONE_OMEGA", 0.0, 0.5)

        return {
            "profile": self._physics_profile_name,
            "applied": {
                "max_accel": self._MAX_ACCEL,
                "max_alpha": self._MAX_ALPHA,
                "control_latency_s": self._CONTROL_LATENCY_S,
                "rolling_resistance": self._ROLLING_RESISTANCE,
                "angular_damping": self._ANGULAR_DAMPING,
                "slip_base": self._SLIP_BASE,
                "slip_turn_gain": self._SLIP_TURN_GAIN,
                "motor_deadzone_vx": self._MOTOR_DEADZONE_VX,
                "motor_deadzone_omega": self._MOTOR_DEADZONE_OMEGA,
            },
        }

    def load_policy_profile(self, policy_name: str, config: dict) -> dict:
        """Load a policy profile config for later execution."""
        self._loaded_policy_name = policy_name
        self._loaded_policy_raw = dict(config)
        return {
            "policy": self._loaded_policy_name,
            "type": str(config.get("type", "cmd_vel_hold")),
            "loaded": True,
        }

    def load_universal_profile(self, profile: dict) -> dict:
        """Load universal robot profile (URDF/USD-inspired envelope).

        Supported immediately:
        - profile.physics.* -> applied to simulator dynamics
        - profile.policy.*  -> loaded as rollout policy
        - profile.visual.*  -> stored for renderer/UI consumption

        Additional sections (kinematics/sensors/controllers/collision/assets)
        are accepted and preserved for future use.
        """
        if not isinstance(profile, dict):
            raise ValueError("Profile must be an object")

        profile_name = str(profile.get("name", "universal-profile"))
        schema_version = str(profile.get("schema_version", "1.0.0"))
        root = dict(profile)

        # Some users may nest fields under "profile", support both layouts.
        payload = root.get("profile", root)
        if not isinstance(payload, dict):
            raise ValueError("Profile payload must be an object")

        physics = payload.get("physics", {})
        policy = payload.get("policy", {})
        visual = payload.get("visual", {})

        if isinstance(physics, dict):
            self.apply_physics_profile(profile_name, physics)
        if isinstance(policy, dict) and policy:
            self.load_policy_profile(profile_name + "::policy", policy)
        if isinstance(visual, dict):
            self._robot_visual_style = dict(visual)

        self._robot_profile_name = profile_name
        self._robot_profile_raw = root
        self._robot_profile_payload = dict(payload)

        sections = sorted([k for k in payload if isinstance(k, str)])
        return {
            "profile": profile_name,
            "schema_version": schema_version,
            "sections": sections,
            "physics_applied": isinstance(physics, dict) and bool(physics),
            "policy_loaded": isinstance(policy, dict) and bool(policy),
            "visual_loaded": isinstance(visual, dict) and bool(visual),
        }

    async def run_loaded_policy(self, duration_s: float = 5.0) -> dict:
        """Run currently loaded policy for a bounded duration."""
        if not self._loaded_policy_raw:
            raise ValueError("No policy loaded")

        policy = self._loaded_policy_raw
        ptype = str(policy.get("type", "cmd_vel_hold"))
        run_for = max(0.2, min(60.0, float(duration_s)))

        if ptype == "waypoints":
            raw_wps = policy.get("waypoints", [])
            waypoints: list[tuple[float, float]] = []
            if isinstance(raw_wps, list):
                for item in raw_wps:
                    if isinstance(item, (list, tuple)) and len(item) >= 2:
                        try:
                            waypoints.append((float(item[0]), float(item[1])))
                        except (TypeError, ValueError):
                            continue
            if waypoints:
                self.set_waypoints(waypoints)
            await asyncio.sleep(run_for)
            return {
                "ok": True,
                "mode": "waypoints",
                "waypoints": len(waypoints),
                "duration_s": run_for,
            }

        # Default: hold a fixed cmd_vel, useful for baseline policy tests
        vx = float(policy.get("vx", 0.3))
        omega = float(policy.get("omega", 0.0))
        step_s = max(0.05, min(0.5, float(policy.get("step_s", 0.12))))
        elapsed = 0.0
        while elapsed < run_for:
            self.set_cmd_vel(vx, omega)
            await asyncio.sleep(step_s)
            elapsed += step_s
        self.set_cmd_vel(0.0, 0.0)
        return {"ok": True, "mode": "cmd_vel_hold", "vx": vx, "omega": omega, "duration_s": run_for}

    # ── Async run loop ────────────────────────────────────────────────────────

    async def run(self) -> None:
        """Run the simulation until ``stop()`` is called."""
        self._running = True
        while self._running:
            await asyncio.sleep(self.DT)
            self._tick(self.DT)

    def start_background(self) -> None:
        """Schedule ``run()`` as an asyncio task on the current event loop."""
        loop = asyncio.get_event_loop()
        self._task = loop.create_task(self.run())

    # ── Physics tick ──────────────────────────────────────────────────────────

    def _tick(self, dt: float) -> None:
        s = self.state

        # Advance delayed manual commands to emulate real control pipeline latency.
        now = time.time()
        while self._cmd_queue and self._cmd_queue[0][0] <= now:
            _, delayed_vx, delayed_omega = self._cmd_queue.popleft()
            self._cmd_vx = delayed_vx
            self._cmd_omega = delayed_omega

        # ── Stuck detection ────────────────────────────────────────────────
        moved = math.hypot(s.x - self._last_x, s.y - self._last_y)
        if moved < 0.005 and (s.nav_active or self._manual_override):
            self._no_move_ticks += 1
        else:
            self._no_move_ticks = 0
            self._last_x = s.x
            self._last_y = s.y

        # Trigger escape when stuck for too long
        if self._no_move_ticks >= self._STUCK_TICKS and s.nav_active:
            self._no_move_ticks = 0
            self._consec_escapes += 1
            self._escape_countdown = self._ESCAPE_TICKS
            self._escape_phase = 0
            # If stuck many times in a row, randomise direction
            if self._consec_escapes % 3 == 0:
                import random

                self._escape_omega = random.choice([-1.5, 1.5])
            else:
                self._escape_omega = self._lidar_clear_omega(s)

        # ── Velocity commands ──────────────────────────────────────────────
        if self._escape_countdown > 0:
            # Two-phase escape:
            #   Phase 0 (first half): back up hard while turning toward clear space
            #   Phase 1 (second half): slide forward in that direction to clear the obstacle
            self._escape_countdown -= 1
            phase_boundary = self._ESCAPE_TICKS // 3  # switch to phase-1 after 1/3
            if self._escape_countdown > (self._ESCAPE_TICKS - phase_boundary):
                # Phase 0: reverse + turn
                self._cmd_vx = -0.25
                self._cmd_omega = self._escape_omega
            else:
                # Phase 1: drive forward in escape direction (sideways slide past obstacle)
                if self._escape_phase == 0:
                    # Latch heading at end of phase 0 so we drive in that direction
                    self._escape_phase = 1
                    self._escape_omega = self._lidar_clear_omega(s)
                self._cmd_vx = 0.22
                self._cmd_omega = self._escape_omega * 0.5  # gentle curve, not full turn

        elif s.nav_active and s.goal_x is not None and s.goal_y is not None:
            dx = s.goal_x - s.x
            dy = s.goal_y - s.y
            dist = math.hypot(dx, dy)

            if dist < self.GOAL_TOLERANCE:
                s.nav_active = False
                s.nav_succeeded = True
                self._cmd_vx = 0.0
                self._cmd_omega = 0.0
                self._consec_escapes = 0
                self._log_event("nav_complete", f"Reached goal ({s.goal_x:.1f}, {s.goal_y:.1f})")
                # Advance to next waypoint if available
                if s.waypoints and s.waypoint_index < len(s.waypoints) - 1:
                    s.waypoint_index += 1
                    next_wp = s.waypoints[s.waypoint_index]
                    self.set_goal(next_wp[0], next_wp[1])
                    self._log_event(
                        "waypoint_next",
                        f"Advancing to waypoint {s.waypoint_index + 1}/{len(s.waypoints)}",
                    )
            else:
                target_angle = math.atan2(dy, dx)
                angle_err = _angle_diff(target_angle, s.theta)

                # Adaptive speed: slow near obstacles and when misaligned
                front_clear = self._front_clearance(s)
                obstacle_factor = min(
                    1.0, max(0.0, (front_clear - ROBOT_RADIUS) / self._DANGER_DIST)
                )
                alignment = 1.0 - min(1.0, abs(angle_err) / (math.pi / 3))
                self._cmd_vx = self.NAV_K_SPEED * alignment * obstacle_factor * min(1.0, dist * 0.8)

                # Reset escape counter when moving freely with clear path
                if front_clear > self._DANGER_DIST * 2 and obstacle_factor > 0.6:
                    self._consec_escapes = 0

                # Turn rate: bigger correction when misaligned; also nudge away from nearby obstacles
                self._cmd_omega = max(-1.8, min(1.8, self.NAV_K_ANGLE * angle_err))
                # Nudge sideways if an obstacle is closer on one side
                side_nudge = self._side_nudge(s)
                self._cmd_omega = max(-1.8, min(1.8, self._cmd_omega + side_nudge))

        elif not s.nav_active and not self._manual_override:
            # Coast to a stop
            self._cmd_vx *= 0.82
            self._cmd_omega *= 0.82
            if abs(self._cmd_vx) < 0.01:
                self._cmd_vx = 0.0
            if abs(self._cmd_omega) < 0.01:
                self._cmd_omega = 0.0

        # ── Apply acceleration limits (motor inertia) ──────────────────────
        accel_limit = self._MAX_ACCEL * dt
        alpha_limit = self._MAX_ALPHA * dt
        s.vx = s.vx + max(-accel_limit, min(accel_limit, self._cmd_vx - s.vx))
        s.omega = s.omega + max(-alpha_limit, min(alpha_limit, self._cmd_omega - s.omega))

        # Drag / damping terms and turn-related slip.
        # High yaw rate reduces net longitudinal traction in a physically plausible way.
        speed_factor = max(0.0, abs(s.vx) - 0.35)
        slip = min(
            0.28, self._SLIP_BASE + abs(s.omega) * self._SLIP_TURN_GAIN + speed_factor * 0.12
        )
        self._current_slip = slip
        s.vx *= max(0.0, 1.0 - self._ROLLING_RESISTANCE * dt)
        s.omega *= max(0.0, 1.0 - self._ANGULAR_DAMPING * dt)

        # ── Integrate position ─────────────────────────────────────────────
        effective_vx = s.vx * (1.0 - slip)
        new_x = s.x + math.cos(s.theta) * effective_vx * dt
        new_y = s.y + math.sin(s.theta) * effective_vx * dt
        new_theta = s.theta + s.omega * dt
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

        # Collision detection — stop forward motion; trigger escape on repeated impact
        if not self._collides(new_x, new_y):
            s.x = new_x
            s.y = new_y
        else:
            # Rebound: damp translational speed and apply tiny bounce-back.
            self._cmd_vx = 0.0
            s.vx = -0.06 * abs(s.vx)
            s.omega *= 0.7
            if s.nav_active and self._escape_countdown == 0:
                self._consec_escapes += 1
                self._escape_countdown = self._ESCAPE_TICKS
                self._escape_phase = 0
                if self._consec_escapes % 3 == 0:
                    import random

                    self._escape_omega = random.choice([-1.5, 1.5])
                else:
                    self._escape_omega = self._lidar_clear_omega(s)
        s.theta = new_theta

        # Clamp to map walls
        s.x = max(ROBOT_RADIUS, min(MAP_SIZE - ROBOT_RADIUS, s.x))
        s.y = max(ROBOT_RADIUS, min(MAP_SIZE - ROBOT_RADIUS, s.y))

        # ── Record velocity history (last 100 samples) ──────────────────────
        s.velocity_history.append(abs(s.vx))
        if len(s.velocity_history) > 100:
            s.velocity_history.pop(0)

        # ── Record path breadcrumb ─────────────────────────────────────────
        if not s.path or math.hypot(s.x - s.path[-1][0], s.y - s.path[-1][1]) > 0.08:
            s.path.append((s.x, s.y))
            if len(s.path) > 250:
                s.path.pop(0)
        if not s.full_path or math.hypot(s.x - s.full_path[-1][0], s.y - s.full_path[-1][1]) > 0.08:
            s.full_path.append((s.x, s.y))
            if len(s.full_path) > 2000:
                s.full_path.pop(0)

        # ── Update occupancy grid from LiDAR ────────────────────────────────
        self._update_occupancy_grid()

        # ── Battery drain ─────────────────────────────────────────────────
        s.battery = max(0.0, s.battery - 0.0008 * (abs(s.vx) + 0.3 * abs(s.omega)) * dt)

        # ── LiDAR scan ────────────────────────────────────────────────────
        t_lidar0 = time.perf_counter()
        s.lidar = self._cast_lidar()
        t_lidar1 = time.perf_counter()

        # ── Camera simulation ─────────────────────────────────────────
        t_cam0 = time.perf_counter()
        s.camera_detections, s.camera_depth_row = self._simulate_camera(s)
        t_cam1 = time.perf_counter()

        # ── SLAM update ───────────────────────────────────────────────
        t_slam0 = time.perf_counter()
        self._update_slam_covariance(s)
        t_slam1 = time.perf_counter()

        s.timing = {
            "lidar_ms": round((t_lidar1 - t_lidar0) * 1000, 3),
            "camera_ms": round((t_cam1 - t_cam0) * 1000, 3),
            "slam_ms": round((t_slam1 - t_slam0) * 1000, 3),
        }
        s.timestamp = time.time()

    # ── Camera simulation ─────────────────────────────────────────────────────

    # Horizontal FOV: 69° (typical RGB-D camera), 64 pixels wide virtual sensor
    _CAM_FOV_DEG: float = 69.0
    _CAM_WIDTH: int = 64
    _CAM_DETECTION_THRESH: float = 2.5  # metres — objects closer than this are "detected"
    _CAM_NOISE_SIGMA: float = 0.04  # depth noise σ

    def _simulate_camera(self, s: RobotState) -> tuple[list[dict], list[float]]:
        """Simulate an RGB-D camera using front-sector LiDAR rays.

        Returns
        -------
        detections : list[dict]
            Each entry: {label, confidence, distance_m, pixel_x, pixel_y, bbox_w, bbox_h}
        depth_row : list[float]
            Per-pixel depth (metres), CAM_WIDTH pixels wide.

        """
        import random as _rng

        if not s.lidar:
            return [], []

        n = len(s.lidar)
        fov_rad = math.radians(self._CAM_FOV_DEG)
        half_fov = fov_rad / 2.0

        # Sample LiDAR rays that fall inside the camera FOV
        depth_row: list[float] = []
        for px in range(self._CAM_WIDTH):
            # Map pixel column to angle offset
            ray_angle_off = -half_fov + fov_rad * (px / (self._CAM_WIDTH - 1))
            # Find the closest LiDAR ray index
            best_i = int(round((ray_angle_off / (2 * math.pi)) * n)) % n
            raw_depth = s.lidar[best_i]
            # Add depth noise
            noisy = raw_depth + _rng.gauss(0, self._CAM_NOISE_SIGMA)
            depth_row.append(round(max(0.05, noisy), 3))

        # Blob detection: contiguous columns closer than threshold → one detection
        detections: list[dict] = []
        in_blob = False
        blob_start = 0
        blob_depths: list[float] = []
        for px, d in enumerate(depth_row):
            is_close = d < self._CAM_DETECTION_THRESH
            if is_close and not in_blob:
                in_blob = True
                blob_start = px
                blob_depths = [d]
            elif is_close and in_blob:
                blob_depths.append(d)
            elif not is_close and in_blob:
                in_blob = False
                self._make_detection(detections, blob_start, px - 1, blob_depths, s)
                blob_depths = []
        if in_blob:
            self._make_detection(detections, blob_start, self._CAM_WIDTH - 1, blob_depths, s)

        return detections, depth_row

    def _make_detection(
        self,
        detections: list,
        x0: int,
        x1: int,
        depths: list[float],
        s: RobotState,
    ) -> None:
        import random as _rng

        cx = (x0 + x1) // 2
        w = max(1, x1 - x0 + 1)
        mean_d = sum(depths) / len(depths)
        # Label heuristic: walls are very close to map edges, else obstacle
        near_wall = s.x < 0.5 or s.x > MAP_SIZE - 0.5 or s.y < 0.5 or s.y > MAP_SIZE - 0.5
        label = "wall" if near_wall else "obstacle"
        confidence = round(
            min(
                0.99,
                0.75
                + (self._CAM_DETECTION_THRESH - mean_d) / self._CAM_DETECTION_THRESH * 0.24
                + _rng.uniform(-0.03, 0.03),
            ),
            2,
        )
        detections.append(
            {
                "label": label,
                "confidence": confidence,
                "distance_m": round(mean_d, 3),
                "pixel_x": cx,
                "pixel_y": 12,  # camera is mounted ~12 px above centre row
                "bbox_w": w,
                "bbox_h": max(4, int(30 / max(0.3, mean_d))),
            }
        )

    # ── SLAM covariance update ─────────────────────────────────────────────────

    def _update_slam_covariance(self, s: RobotState) -> None:
        """Update SLAM positional covariance using a simple EKF-inspired model.

        Motion increases uncertainty; sharp corners (many close LiDAR hits)
        reduce it (loop-closure proxy). Covariance is bounded.
        """
        speed = abs(s.vx)
        turn = abs(s.omega)
        # Propagation noise proportional to motion
        s.slam_covariance += (speed * 0.0003 + turn * 0.0001) * self.DT
        # Observation update: many close lidar hits → good features → shrink cov
        if s.lidar:
            close_hits = sum(1 for r in s.lidar if r < 1.5)
            feature_strength = close_hits / len(s.lidar)
            s.slam_covariance *= max(0.90, 1.0 - feature_strength * 0.08)
        # When escaping (lots of motion, poor geometry) → grow faster
        if self._escape_countdown > 0:
            s.slam_covariance += 0.002
        # Clamp
        s.slam_covariance = max(0.005, min(0.50, s.slam_covariance))
        # Count explored cells
        s.slam_map_cells_seen = sum(1 for row in s.occupancy_grid for c in row if c > 0.05)

    def get_slam_state(self) -> dict:
        """Return a snapshot of SLAM-layer data."""
        s = self.state
        total_cells = 100 * 100
        explored_pct = round(s.slam_map_cells_seen / total_cells * 100, 1)
        return {
            "pose_x": round(s.x, 4),
            "pose_y": round(s.y, 4),
            "pose_theta_deg": round(math.degrees(s.theta), 2),
            "covariance_m2": round(s.slam_covariance, 5),
            "position_uncertainty_cm": round(math.sqrt(s.slam_covariance) * 100, 2),
            "map_cells_seen": s.slam_map_cells_seen,
            "map_explored_pct": explored_pct,
            "scan_match_quality": round(max(0.0, 1.0 - s.slam_covariance * 3), 2),
        }

    def get_joint_states(self) -> dict:
        """Derive wheel joint states from differential-drive kinematics.

        TurtleBot4 wheel radius  ≈ 0.0352 m
        Track width (half)       ≈ 0.1168 m
        """
        s = self.state
        wheel_radius = 0.0352
        half_track = 0.1168
        # Wheel angular velocities (rad/s) from diff-drive model
        omega_left = (s.vx - half_track * s.omega) / wheel_radius
        omega_right = (s.vx + half_track * s.omega) / wheel_radius
        # Integrate to approximate joint positions (wraps at ±π)
        return {
            "wheel_left": {
                "position_rad": round(
                    math.atan2(
                        math.sin(s.timestamp * omega_left % (2 * math.pi)),
                        math.cos(s.timestamp * omega_left % (2 * math.pi)),
                    ),
                    4,
                ),
                "velocity_rad_s": round(omega_left, 4),
                "effort_Nm": round(abs(omega_left) * 0.15, 4),
            },
            "wheel_right": {
                "position_rad": round(
                    math.atan2(
                        math.sin(s.timestamp * omega_right % (2 * math.pi)),
                        math.cos(s.timestamp * omega_right % (2 * math.pi)),
                    ),
                    4,
                ),
                "velocity_rad_s": round(omega_right, 4),
                "effort_Nm": round(abs(omega_right) * 0.15, 4),
            },
        }

    def get_imu_state(self) -> dict:
        """Return a simulated IMU reading: orientation from theta, acceleration from velocity."""
        import random as _rng

        s = self.state
        qz = math.sin(s.theta / 2)
        qw = math.cos(s.theta / 2)
        # Linear accel approximation: centripetal + gravity noise
        ax = -s.vx * s.omega + _rng.gauss(0, 0.008)
        ay = s.omega * s.vx + _rng.gauss(0, 0.008)
        az = 9.806 + _rng.gauss(0, 0.005)
        return {
            "orientation": {"x": 0.0, "y": 0.0, "z": round(qz, 5), "w": round(qw, 5)},
            "angular_velocity": {
                "x": round(_rng.gauss(0, 0.002), 6),
                "y": round(_rng.gauss(0, 0.002), 6),
                "z": round(s.omega + _rng.gauss(0, 0.003), 6),
            },
            "linear_acceleration": {
                "x": round(ax, 6),
                "y": round(ay, 6),
                "z": round(az, 6),
            },
        }

    # ── Geometry helpers ──────────────────────────────────────────────────────

    def _front_clearance(self, s: RobotState) -> float:
        """Minimum LiDAR range in the forward ±60° cone."""
        if not s.lidar:
            return self.LIDAR_MAX
        n = len(s.lidar)
        front: list[float] = []
        for i in range(n):
            ray_angle = s.theta + (2 * math.pi * i / n)
            if abs(_angle_diff(ray_angle, s.theta)) <= self._FRONT_HALF_ANGLE:
                front.append(s.lidar[i])
        return min(front) if front else self.LIDAR_MAX

    def _lidar_clear_omega(self, s: RobotState) -> float:
        """Return ±1.5 rad/s toward the LiDAR half with more open space."""
        if not s.lidar:
            return 1.5
        n = len(s.lidar)
        # Left half: rays 1..n//2, right half: rays n//2+1..n-1
        left = sum(s.lidar[i] for i in range(1, n // 2))
        right = sum(s.lidar[i] for i in range(n // 2 + 1, n))
        return 1.5 if left >= right else -1.5

    def _side_nudge(self, s: RobotState) -> float:
        """Small rotational correction to push the robot away from close side walls."""
        if not s.lidar:
            return 0.0
        n = len(s.lidar)
        # Rays in the ±90° lateral bands
        left_rays = [s.lidar[i] for i in range(n // 8, 3 * n // 8)]
        right_rays = [s.lidar[i] for i in range(5 * n // 8, 7 * n // 8)]
        l_min = min(left_rays) if left_rays else self.LIDAR_MAX
        r_min = min(right_rays) if right_rays else self.LIDAR_MAX
        threshold = 0.45
        if l_min < threshold or r_min < threshold:
            # Nudge away from whichever side is closer
            return -0.6 if l_min < r_min else 0.6
        return 0.0

    def _collides(self, x: float, y: float) -> bool:
        r = ROBOT_RADIUS
        for ox, oy, w, h in OBSTACLES:
            if (ox - r) <= x <= (ox + w + r) and (oy - r) <= y <= (oy + h + r):
                return True
        return False

    def _cast_lidar(self) -> list[float]:
        """Cast LIDAR_RAYS rays from current position, return ranges with sensor noise."""
        import random

        s = self.state
        ranges: list[float] = []
        step = 0.04  # ray march step size (metres)
        n_steps = int(self.LIDAR_MAX / step)

        for i in range(self.LIDAR_RAYS):
            # Occasional dropout (glass/reflective surface returns max range)
            if random.random() < self._LIDAR_DROPOUT_PROB:
                ranges.append(self.LIDAR_MAX)
                continue

            angle = s.theta + (2 * math.pi * i / self.LIDAR_RAYS)
            dx = math.cos(angle) * step
            dy = math.sin(angle) * step
            px, py = s.x, s.y
            hit = self.LIDAR_MAX

            for j in range(1, n_steps + 1):
                px += dx
                py += dy

                # Room walls
                if px <= 0 or px >= MAP_SIZE or py <= 0 or py >= MAP_SIZE:
                    hit = j * step
                    break

                # Obstacles
                for ox, oy, w, h in OBSTACLES:
                    if ox <= px <= ox + w and oy <= py <= oy + h:
                        hit = j * step
                        break
                else:
                    continue
                break

            # Add Gaussian noise to simulate real LiDAR measurement error
            if hit < self.LIDAR_MAX:
                hit += random.gauss(0, self._LIDAR_NOISE_SIGMA)
                hit = max(0.01, min(self.LIDAR_MAX, hit))

            ranges.append(round(hit, 3))

        return ranges

    # ── Serialisation ─────────────────────────────────────────────────────────

    def get_state_dict(self) -> dict:
        """Return a JSON-serialisable snapshot of the current simulation state."""
        s = self.state
        collision = self._robot_profile_payload.get("collision", {})
        if not isinstance(collision, dict):
            collision = {}
        kinematics = self._robot_profile_payload.get("kinematics", {})
        if not isinstance(kinematics, dict):
            kinematics = {}
        assets = self._robot_profile_payload.get("assets", {})
        if not isinstance(assets, dict):
            assets = {}
        sensors = self._robot_profile_payload.get("sensors", [])
        if not isinstance(sensors, list):
            sensors = []

        profile_signature = (
            f"{self._robot_profile_name or 'none'}"
            f":{len(collision.get('primitives', [])) if isinstance(collision.get('primitives', []), list) else 0}"
            f":{len(kinematics.get('links', [])) if isinstance(kinematics.get('links', []), list) else 0}"
        )

        return {
            "x": round(s.x, 4),
            "y": round(s.y, 4),
            "theta": round(s.theta, 4),
            "vx": round(s.vx, 3),
            "omega": round(s.omega, 3),
            "battery": round(s.battery, 2),
            "goal_x": round(s.goal_x, 3) if s.goal_x is not None else None,
            "goal_y": round(s.goal_y, 3) if s.goal_y is not None else None,
            "nav_active": s.nav_active,
            "nav_succeeded": s.nav_succeeded,
            "escaping": self._escape_countdown > 0,
            "path": [[round(p[0], 2), round(p[1], 2)] for p in s.path[-80:]],
            "lidar": s.lidar,
            "obstacles": OBSTACLES,
            "map_size": MAP_SIZE,
            "ts": round(s.timestamp, 3),
            "waypoints": [[round(w[0], 2), round(w[1], 2)] for w in s.waypoints],
            "waypoint_index": s.waypoint_index,
            "velocity_history": [round(v, 4) for v in s.velocity_history[-100:]],
            "slip": round(self._current_slip, 4),
            "control_latency_ms": int(self._CONTROL_LATENCY_S * 1000),
            "robot_profile": {
                "loaded": self._robot_profile_name is not None,
                "name": self._robot_profile_name,
                "signature": profile_signature,
                "visual": self._robot_visual_style,
                "collision": collision,
                "kinematics": {
                    "base_type": kinematics.get("base_type", ""),
                    "links": kinematics.get("links", []),
                },
                "assets": {
                    "usd_reference": assets.get("usd_reference", ""),
                    "urdf_reference": assets.get("urdf_reference", ""),
                },
                "sensor_count": len(sensors),
            },
        }

    def get_full_path(self) -> list[list[float]]:
        """Return the last 1000 path poses as ``[x, y]`` pairs."""
        return [[round(p[0], 2), round(p[1], 2)] for p in self.state.full_path[-1000:]]

    def get_occupancy_grid(self, resolution: int = 20) -> list[list[float]]:
        """Return a downsampled occupancy grid for visualization."""
        grid = self.state.occupancy_grid
        src_size = 100
        out: list[list[float]] = [[0.0] * resolution for _ in range(resolution)]
        ratio = src_size / resolution
        for oy in range(resolution):
            sy0 = int(oy * ratio)
            sy1 = int((oy + 1) * ratio)
            for ox in range(resolution):
                sx0 = int(ox * ratio)
                sx1 = int((ox + 1) * ratio)
                total = 0.0
                count = 0
                for sy in range(sy0, sy1):
                    row = grid[sy]
                    for sx in range(sx0, sx1):
                        total += row[sx]
                        count += 1
                out[oy][ox] = total / max(count, 1)
        return out

    # ── Event log ───────────────────────────────────────────────────────────

    def _log_event(self, category: str, message: str) -> None:
        self._events.append(
            {
                "ts": time.time(),
                "cat": category,
                "msg": message,
            }
        )
        if len(self._events) > 200:
            self._events.pop(0)

    def get_events(self, limit: int = 50) -> list[dict]:
        """Return the most recent *limit* simulation events."""
        return self._events[-limit:]

    # ── Occupancy grid ──────────────────────────────────────────────────────

    def _init_occupancy_grid(self) -> None:
        """Initialize the occupancy grid with known obstacle positions."""
        grid = self.state.occupancy_grid
        cell_size = MAP_SIZE / 100  # 0.1m per cell
        for ox, oy, ow, oh in OBSTACLES:
            x0 = int(ox / cell_size)
            y0 = int(oy / cell_size)
            x1 = int((ox + ow) / cell_size)
            y1 = int((oy + oh) / cell_size)
            for gy in range(max(0, y0), min(100, y1 + 1)):
                for gx in range(max(0, x0), min(100, x1 + 1)):
                    grid[gy][gx] = 1.0  # occupied

    def _update_occupancy_grid(self) -> None:
        """Update occupancy grid using current LiDAR hits. Only updates cells near the robot."""
        s = self.state
        grid = s.occupancy_grid
        cell_size = MAP_SIZE / 100
        decay = 0.985

        # Only decay cells within a window around the robot position
        rx_i = int(s.x / cell_size)
        ry_i = int(s.y / cell_size)
        win = 30  # cells around robot
        for gy in range(max(0, ry_i - win), min(100, ry_i + win)):
            row = grid[gy]
            for gx in range(max(0, rx_i - win), min(100, rx_i + win)):
                v = row[gx]
                if v < 0.90:
                    row[gx] = v * decay

        # Mark LiDAR hits
        if not s.lidar:
            return
        n = len(s.lidar)
        for i in range(n):
            r = s.lidar[i]
            if r >= MAP_SIZE * 0.59 or r < 0.01:
                continue
            angle = s.theta + (2 * math.pi * i / n)
            hx = s.x + math.cos(angle) * r
            hy = s.y + math.sin(angle) * r
            hx = max(0.05, min(MAP_SIZE - 0.05, hx))
            hy = max(0.05, min(MAP_SIZE - 0.05, hy))
            gx = int(hx / cell_size)
            gy = int(hy / cell_size)
            if 0 <= gx < 100 and 0 <= gy < 100:
                grid[gy][gx] = min(1.0, grid[gy][gx] + 0.12)


# ── Singleton ─────────────────────────────────────────────────────────────────

_sim_instance: RobotSim | None = None


def get_sim() -> RobotSim:
    """Return the global :class:`RobotSim` singleton, creating it if needed."""
    global _sim_instance
    if _sim_instance is None:
        _sim_instance = RobotSim()
    return _sim_instance


# ── Utility ───────────────────────────────────────────────────────────────────


def _angle_diff(a: float, b: float) -> float:
    """Signed angle difference, result in [-pi, pi]."""
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))
