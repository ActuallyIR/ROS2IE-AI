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
    lidar: list[float] = field(default_factory=list)
    timestamp: float = field(default_factory=time.time)


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

    # Sensor noise
    _LIDAR_NOISE_SIGMA: float = 0.018  # Gaussian σ on LiDAR ranges (metres)
    _LIDAR_DROPOUT_PROB: float = 0.012  # probability of a ray returning max range (glass/dropout)

    # Obstacle avoidance
    _FRONT_HALF_ANGLE: float = math.pi / 3  # ±60° cone treated as "front"
    _DANGER_DIST: float = 0.55  # slow down below this clearance
    _ESCAPE_TICKS: int = 60  # ticks to run escape manoeuvre (~3 s)
    _STUCK_TICKS: int = 25  # ticks without movement = stuck (~1.25 s)

    def __init__(self) -> None:
        self.state = RobotState()
        self._cmd_vx: float = 0.0
        self._cmd_omega: float = 0.0
        self._manual_override: bool = False
        self._running: bool = False
        self._task: asyncio.Task | None = None  # type: ignore[type-arg]

        # Escape / stuck tracking
        self._escape_countdown: int = 0  # ticks remaining in escape mode
        self._escape_omega: float = 1.5  # direction of escape turn
        self._escape_phase: int = 0  # 0=turn, 1=forward-slide past obstacle
        self._no_move_ticks: int = 0  # consecutive ticks with near-zero movement
        self._last_x: float = self.state.x
        self._last_y: float = self.state.y
        self._consec_escapes: int = 0  # how many escapes from same spot

    # ── Public control API ────────────────────────────────────────────────────

    def set_cmd_vel(self, vx: float, omega: float) -> None:
        """Called by mock ROS2 when cmd_vel is published."""
        self._cmd_vx = max(-0.5, min(0.5, vx))
        self._cmd_omega = max(-2.0, min(2.0, omega))
        self._manual_override = True
        self.state.nav_active = False
        self._escape_countdown = 0

    def set_goal(self, x: float, y: float, goal_id: int | None = None) -> None:
        """Called by mock ROS2 when a navigation goal is sent."""
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

    def stop(self) -> None:
        """Kill the async loop."""
        self._running = False
        if self._task is not None:
            self._task.cancel()

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

        # ── Integrate position ─────────────────────────────────────────────
        new_x = s.x + math.cos(s.theta) * s.vx * dt
        new_y = s.y + math.sin(s.theta) * s.vx * dt
        new_theta = s.theta + s.omega * dt
        new_theta = math.atan2(math.sin(new_theta), math.cos(new_theta))

        # Collision detection — stop forward motion; trigger escape on repeated impact
        if not self._collides(new_x, new_y):
            s.x = new_x
            s.y = new_y
        else:
            # Rebound: kill forward speed, start escape if not already escaping
            self._cmd_vx = 0.0
            s.vx = 0.0
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

        # ── Record path breadcrumb ─────────────────────────────────────────
        if not s.path or math.hypot(s.x - s.path[-1][0], s.y - s.path[-1][1]) > 0.08:
            s.path.append((s.x, s.y))
            if len(s.path) > 250:
                s.path.pop(0)

        # ── Battery drain ─────────────────────────────────────────────────
        s.battery = max(0.0, s.battery - 0.0008 * (abs(s.vx) + 0.3 * abs(s.omega)) * dt)

        # ── LiDAR scan ────────────────────────────────────────────────────
        s.lidar = self._cast_lidar()
        s.timestamp = time.time()

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
        }


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
