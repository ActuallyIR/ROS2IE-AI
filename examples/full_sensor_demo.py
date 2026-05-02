"""Full sensor telemetry demo — no ROS 2 installation needed.

Run with:
    python examples/full_sensor_demo.py

What you'll see (refreshed every 0.4 s):
  ┌─ INPUT      cmd_vel commands sent to the robot
  ├─ MOTOR      actual wheel speeds, acceleration, slip
  ├─ JOINTS     left/right wheel joint positions, velocities, effort
  ├─ IMU        orientation quaternion, angular vel, linear accel
  ├─ LIDAR      90-ray scan — ASCII polar plot + statistics
  ├─ CAMERA     RGB-D depth row + detected objects with bounding boxes
  ├─ SLAM       pose estimate, covariance, map exploration %
  ├─ NAV        goal, distance, heading error, nav status
  ├─ BATTERY    state-of-charge + drain rate
  └─ TIMING     per-layer processing latency
"""

from __future__ import annotations

import asyncio
import math
import os
import sys
import time

# ── Make the package importable when run from the repo root ──────────────────
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from ros2_agent.simulation.robot_sim import MAP_SIZE, OBSTACLES, get_sim

# ── ANSI colour helpers (disabled on Windows without VT mode) ─────────────────
_USE_COLOR = sys.platform != "win32" or os.environ.get("TERM") or (
    os.system("") == 0  # enable VT on Windows Terminal
)

def _c(code: str, text: str) -> str:
    if _USE_COLOR:
        return f"\033[{code}m{text}\033[0m"
    return text

def RED(t: str) -> str:    return _c("91", t)
def GREEN(t: str) -> str:  return _c("92", t)
def YELLOW(t: str) -> str: return _c("93", t)
def CYAN(t: str) -> str:   return _c("96", t)
def BOLD(t: str) -> str:   return _c("1",  t)
def DIM(t: str) -> str:    return _c("2",  t)
def MAGENTA(t: str) -> str: return _c("95", t)

# ── LIDAR ASCII polar plot ─────────────────────────────────────────────────────

_POLAR_ROWS = 11
_POLAR_COLS = 23

def _lidar_polar(ranges: list[float], lidar_max: float) -> list[str]:
    """Render a tiny top-down polar plot of LiDAR ranges (ASCII art)."""
    rows = _POLAR_ROWS
    cols = _POLAR_COLS
    cx = cols // 2
    cy = rows // 2
    # scale: plot radius = cy cells = lidar_max metres
    scale = cy / lidar_max

    grid = [[" "] * cols for _ in range(rows)]

    # Draw range ring markers
    for ring_m in (1.0, 3.0, 5.0):
        r_cells = ring_m * scale
        for i in range(360):
            a = math.radians(i)
            rx = cx + int(round(math.cos(a) * r_cells))
            ry = cy - int(round(math.sin(a) * r_cells))
            if 0 <= rx < cols and 0 <= ry < rows:
                if grid[ry][rx] == " ":
                    grid[ry][rx] = "·"

    # Draw robot at centre
    grid[cy][cx] = "R"

    # Draw LiDAR hits
    n = len(ranges)
    for i, r in enumerate(ranges):
        if r >= lidar_max * 0.98:
            continue  # miss / dropout
        angle = 2 * math.pi * i / n
        r_cells = r * scale
        rx = cx + int(round(math.cos(angle) * r_cells))
        ry = cy - int(round(math.sin(angle) * r_cells))
        if 0 <= rx < cols and 0 <= ry < rows and (rx != cx or ry != cy):
            grid[ry][rx] = "█"

    return ["│" + "".join(row) + "│" for row in grid]


# ── Camera depth row visualiser ───────────────────────────────────────────────

_DEPTH_CHARS = " ░▒▓█"  # shallow → deep

def _depth_bar(depth_row: list[float], lidar_max: float, width: int = 64) -> str:
    if not depth_row:
        return " " * width
    bar = []
    for d in depth_row[:width]:
        norm = 1.0 - min(1.0, d / lidar_max)  # 0=far 1=close
        idx = int(norm * (len(_DEPTH_CHARS) - 1))
        bar.append(_DEPTH_CHARS[idx])
    return "".join(bar)


# ── Occupancy mini-map ────────────────────────────────────────────────────────

_MAP_ROWS = 11
_MAP_COLS = 22

def _mini_map(sim) -> list[str]:
    """Tiny occupancy-grid ASCII map with robot position."""
    scale_x = _MAP_COLS / MAP_SIZE
    scale_y = _MAP_ROWS / MAP_SIZE
    s = sim.state
    # Simple static obstacle rendering
    grid = [[" "] * _MAP_COLS for _ in range(_MAP_ROWS)]
    # Room border
    for c in range(_MAP_COLS):
        grid[0][c] = "─"
        grid[_MAP_ROWS - 1][c] = "─"
    for r in range(_MAP_ROWS):
        grid[r][0] = "│"
        grid[r][_MAP_COLS - 1] = "│"
    # Obstacles
    for ox, oy, ow, oh in OBSTACLES:
        for row in range(max(0, int(oy * scale_y)), min(_MAP_ROWS - 1, int((oy + oh) * scale_y) + 1)):
            for col in range(max(0, int(ox * scale_x)), min(_MAP_COLS - 1, int((ox + ow) * scale_x) + 1)):
                grid[row][col] = "▪"
    # Goal
    if s.goal_x is not None and s.nav_active:
        gr = int(s.goal_y * scale_y)
        gc = int(s.goal_x * scale_x)
        if 0 <= gr < _MAP_ROWS and 0 <= gc < _MAP_COLS:
            grid[gr][gc] = "G"
    # Robot
    rr = int(s.y * scale_y)
    rc = int(s.x * scale_x)
    rr = max(1, min(_MAP_ROWS - 2, rr))
    rc = max(1, min(_MAP_COLS - 2, rc))
    # Direction arrow
    arrows = {(-1, 0): "←", (1, 0): "→", (0, -1): "↓", (0, 1): "↑",
              (-1, -1): "↙", (-1, 1): "↖", (1, -1): "↘", (1, 1): "↗"}
    dx = int(round(math.cos(s.theta)))
    dy = int(round(math.sin(s.theta)))
    grid[rr][rc] = arrows.get((dx, dy), "◉")
    return ["│" + "".join(row) + "│" for row in grid]


# ── Section header helper ─────────────────────────────────────────────────────

def _hdr(name: str, width: int = 70) -> str:
    pad = width - len(name) - 4
    return BOLD(CYAN(f"┌─ {name} " + "─" * max(0, pad) + "┐"))

def _sep(width: int = 70) -> str:
    return DIM("└" + "─" * (width - 2) + "┘")

# ── Bar chart helper ──────────────────────────────────────────────────────────

def _bar(value: float, lo: float, hi: float, width: int = 20, color_fn=GREEN) -> str:
    frac = (value - lo) / max(1e-9, hi - lo)
    frac = max(0.0, min(1.0, frac))
    filled = int(frac * width)
    return color_fn("█" * filled) + DIM("░" * (width - filled))


# ── Main telemetry print ──────────────────────────────────────────────────────

def _tick_count_str(n: int) -> str:
    return DIM(f"  [tick #{n}  t={time.strftime('%H:%M:%S')}]")


def print_telemetry(sim, tick: int, cmd_vx: float, cmd_omega: float) -> None:
    s = sim.state
    W = 72

    os.system("cls" if sys.platform == "win32" else "clear")

    title = BOLD("  🤖  ROS 2 ROBOT FULL SENSOR TELEMETRY  ") + _tick_count_str(tick)
    print(title)
    print()

    # ──────────────────────────────────────────────────────────────────────────
    # INPUT LAYER
    # ──────────────────────────────────────────────────────────────────────────
    print(_hdr("INPUT  /cmd_vel", W))
    vx_bar  = _bar(abs(cmd_vx),    0, 0.55, 16, GREEN if cmd_vx >= 0 else YELLOW)
    om_bar  = _bar(abs(cmd_omega), 0, 2.0,  16, YELLOW)
    dir_str = GREEN("FWD") if cmd_vx > 0.01 else (RED("REV") if cmd_vx < -0.01 else DIM("STOP"))
    turn_str = YELLOW("LEFT") if cmd_omega > 0.01 else (MAGENTA("RIGHT") if cmd_omega < -0.01 else DIM("STRAIGHT"))
    print(f"  linear.x  = {cmd_vx:+.4f} m/s  {vx_bar}  {dir_str}")
    print(f"  angular.z = {cmd_omega:+.4f} r/s  {om_bar}  {turn_str}")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # MOTOR / ACTUATOR LAYER
    # ──────────────────────────────────────────────────────────────────────────
    print(_hdr("MOTOR  actuator state", W))
    actual_vx  = s.vx
    actual_om  = s.omega
    slip       = sim._current_slip
    latency_ms = sim._CONTROL_LATENCY_S * 1000
    slip_color = (RED if slip > 0.15 else YELLOW if slip > 0.07 else GREEN)
    print(f"  actual vx     = {actual_vx:+.4f} m/s   (cmd={cmd_vx:+.4f})")
    print(f"  actual omega  = {actual_om:+.4f} r/s   (cmd={cmd_omega:+.4f})")
    print(f"  slip          = {slip:.4f}  {_bar(slip, 0, 0.28, 14, slip_color)}")
    print(f"  control latency = {latency_ms:.1f} ms    deadzone vx≥{sim._MOTOR_DEADZONE_VX:.3f} ω≥{sim._MOTOR_DEADZONE_OMEGA:.3f}")
    print(f"  max_accel={sim._MAX_ACCEL:.1f} m/s²  max_alpha={sim._MAX_ALPHA:.1f} r/s²")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # JOINT STATES
    # ──────────────────────────────────────────────────────────────────────────
    joints = sim.get_joint_states()
    wl = joints["wheel_left"]
    wr = joints["wheel_right"]
    print(_hdr("JOINTS  /joint_states", W))
    print(f"  {'Joint':<22} {'Position (rad)':>14}  {'Velocity (r/s)':>14}  {'Effort (Nm)':>11}")
    print(f"  {DIM('─' * 65)}")
    wl_vc = GREEN if abs(wl['velocity_rad_s']) > 0.01 else DIM
    wr_vc = GREEN if abs(wr['velocity_rad_s']) > 0.01 else DIM
    wl_vel_str = wl_vc(f"{wl['velocity_rad_s']:>+14.4f}")
    wr_vel_str = wr_vc(f"{wr['velocity_rad_s']:>+14.4f}")
    print(f"  {'wheel_left_joint':<22} {wl['position_rad']:>14.4f}  {wl_vel_str}  {wl['effort_Nm']:>11.4f}")
    print(f"  {'wheel_right_joint':<22} {wr['position_rad']:>14.4f}  {wr_vel_str}  {wr['effort_Nm']:>11.4f}")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # IMU
    # ──────────────────────────────────────────────────────────────────────────
    imu = sim.get_imu_state()
    ori = imu["orientation"]
    av  = imu["angular_velocity"]
    la  = imu["linear_acceleration"]
    print(_hdr("IMU  /imu", W))
    print(f"  orientation    qx={ori['x']:.5f}  qy={ori['y']:.5f}  qz={ori['z']:.5f}  qw={ori['w']:.5f}")
    print(f"  angular_vel    wx={av['x']:.6f}  wy={av['y']:.6f}  wz={av['z']:.6f} r/s")
    print(f"  linear_accel   ax={la['x']:.6f}  ay={la['y']:.6f}  az={la['z']:.6f} m/s²")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # LIDAR
    # ──────────────────────────────────────────────────────────────────────────
    print(_hdr("LIDAR  /scan  (90-ray RPLIDAR sim)", W))
    lidar = s.lidar
    if lidar:
        n = len(lidar)
        hits = [r for r in lidar if r < sim.LIDAR_MAX * 0.98]
        hit_rate = len(hits) / n * 100
        min_r  = min(hits) if hits else sim.LIDAR_MAX
        max_r  = max(hits) if hits else sim.LIDAR_MAX
        mean_r = sum(hits) / len(hits) if hits else sim.LIDAR_MAX
        front_clear = sim._front_clearance(s)
        # Polar plot + mini-map side by side
        polar = _lidar_polar(lidar, sim.LIDAR_MAX)
        mmap  = _mini_map(sim)
        rows = max(len(polar), len(mmap))
        polar += ["│" + " " * _POLAR_COLS + "│"] * (rows - len(polar))
        mmap  += ["│" + " " * _MAP_COLS   + "│"] * (rows - len(mmap))
        print(f"  rays={n}  hits={len(hits)}({hit_rate:.0f}%)  "
              f"min={min_r:.2f}m  max={max_r:.2f}m  mean={mean_r:.2f}m  "
              f"front={front_clear:.2f}m")
        print(f"  {'  Polar (top-down)  ':^25}    {'  Occupancy map  ':^24}")
        for p_row, m_row in zip(polar, mmap):
            print(f"  {p_row}    {m_row}")
        # Compact range histogram (8 buckets)
        buckets = [0] * 8
        bw = sim.LIDAR_MAX / 8
        for r in lidar:
            bi = min(7, int(r / bw))
            buckets[bi] += 1
        print(f"  Range histogram  (each bucket = {bw:.2f} m):")
        for bi, cnt in enumerate(buckets):
            label = f"  {bi*bw:.1f}-{(bi+1)*bw:.1f}m"
            bar = GREEN("█") * cnt + DIM("░") * (n - cnt)
            print(f"  {label:<11} {bar}  {cnt:>2}")
    else:
        print("  (no scan yet)")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # CAMERA
    # ──────────────────────────────────────────────────────────────────────────
    print(_hdr("CAMERA  /camera/color+depth  (69° FOV, 64px virtual)", W))
    depth_row = s.camera_depth_row
    detections = s.camera_detections
    if depth_row:
        bar = _depth_bar(depth_row, sim.LIDAR_MAX, 64)
        min_d = min(depth_row)
        max_d = max(depth_row)
        mean_d = sum(depth_row) / len(depth_row)
        print(f"  Depth row (64 px): [{bar}]")
        print(f"  depth  min={min_d:.2f}m  max={max_d:.2f}m  mean={mean_d:.2f}m")
    else:
        print("  (no depth frame yet)")

    if detections:
        print(f"  Detections ({len(detections)} object{'s' if len(detections) != 1 else ''}):")
        for i, det in enumerate(detections):
            conf_c = GREEN if det['confidence'] > 0.85 else YELLOW
            dist_c = RED if det['distance_m'] < 0.6 else (YELLOW if det['distance_m'] < 1.2 else GREEN)
            label_str = BOLD(det['label'])
            conf_str = conf_c(f"{det['confidence']:.2f}")
            dist_str = dist_c(f"{det['distance_m']:.2f}m")
            print(f"    [{i}] {label_str:<12}  conf={conf_str}  dist={dist_str}  bbox=({det['pixel_x']}px, {det['pixel_y']}px, {det['bbox_w']}x{det['bbox_h']})")
    else:
        print(f"  {GREEN('✓')} No obstacles detected in camera FOV")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # SLAM
    # ──────────────────────────────────────────────────────────────────────────
    slam = sim.get_slam_state()
    print(_hdr("SLAM  slam_toolbox  (pose + covariance + map)", W))
    cov_c = GREEN if slam['covariance_m2'] < 0.05 else (YELLOW if slam['covariance_m2'] < 0.15 else RED)
    print(f"  pose  x={slam['pose_x']:.4f}m  y={slam['pose_y']:.4f}m  th={slam['pose_theta_deg']:.2f} deg")
    cov_str = cov_c(f"{slam['position_uncertainty_cm']:.2f} cm")
    print(f"  position uncertainty = {cov_str}  (cov={slam['covariance_m2']:.5f} m2)")
    print(f"  scan match quality   = {_bar(slam['scan_match_quality'], 0, 1, 14, GREEN)}  {slam['scan_match_quality']:.2f}")
    exp_c = GREEN if slam['map_explored_pct'] > 50 else YELLOW
    exp_str = exp_c(f"{slam['map_explored_pct']:.1f}%")
    print(f"  map explored         = {exp_str}  ({slam['map_cells_seen']} / 10000 cells)")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # NAVIGATION
    # ──────────────────────────────────────────────────────────────────────────
    print(_hdr("NAV  bt_navigator / planner_server", W))
    if s.goal_x is not None:
        dx = s.goal_x - s.x
        dy = s.goal_y - s.y
        dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_err = math.atan2(math.sin(target_angle - s.theta), math.cos(target_angle - s.theta))
        status = RED("ESCAPING") if sim._escape_countdown > 0 else (
            GREEN("ACTIVE") if s.nav_active else (
                GREEN("SUCCEEDED") if s.nav_succeeded else DIM("IDLE")
            )
        )
        wp_str = f"waypoint {s.waypoint_index + 1}/{len(s.waypoints)}" if s.waypoints else "direct"
        print(f"  goal  x={s.goal_x:.2f}m  y={s.goal_y:.2f}m  [{wp_str}]")
        print(f"  distance to goal = {_bar(dist, 0, MAP_SIZE, 12, CYAN)}  {dist:.3f} m")
        print(f"  heading error    = {angle_err:+.3f} rad  ({math.degrees(angle_err):+.1f}°)")
        print(f"  nav status       = {status}  "
              f"(escapes={sim._consec_escapes}  stuck_ticks={sim._no_move_ticks})")
    else:
        print(f"  {DIM('No active navigation goal')}")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # ODOMETRY
    # ──────────────────────────────────────────────────────────────────────────
    qz = math.sin(s.theta / 2)
    qw = math.cos(s.theta / 2)
    print(_hdr("ODOMETRY  /odom", W))
    print(f"  pose   x={s.x:.4f}m  y={s.y:.4f}m  θ={math.degrees(s.theta):.2f}°")
    print(f"         qz={qz:.5f}  qw={qw:.5f}")
    print(f"  twist  vx={s.vx:+.4f} m/s  ω={s.omega:+.4f} r/s")
    if s.velocity_history:
        avg_spd = sum(s.velocity_history[-10:]) / len(s.velocity_history[-10:])
        print(f"  avg speed (10-sample) = {avg_spd:.4f} m/s  "
              f"{_bar(avg_spd, 0, 0.55, 14, GREEN)}")
    print(f"  path breadcrumbs = {len(s.path)}  full_path = {len(s.full_path)}")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # BATTERY
    # ──────────────────────────────────────────────────────────────────────────
    bat = s.battery
    bat_c = GREEN if bat > 50 else (YELLOW if bat > 20 else RED)
    volts = 14.0 + (bat / 100.0) * 2.5
    print(_hdr("BATTERY  /battery_state", W))
    print(f"  {_bar(bat, 0, 100, 30, bat_c)}  {bat_c(f'{bat:.2f}%')}  ({volts:.2f} V)")
    drain = 0.0008 * (abs(s.vx) + 0.3 * abs(s.omega))
    print(f"  drain rate = {drain:.6f} %/s  (motion-proportional)")
    print(_sep(W))

    # ──────────────────────────────────────────────────────────────────────────
    # PROCESSING TIMES
    # ──────────────────────────────────────────────────────────────────────────
    timing = s.timing
    print(_hdr("PROCESSING TIMES  (per physics tick @ 20 Hz)", W))
    for layer, key in [("LiDAR cast", "lidar_ms"), ("Camera+detect", "camera_ms"), ("SLAM update", "slam_ms")]:
        ms = timing.get(key, 0.0)
        ms_c = GREEN if ms < 2.0 else (YELLOW if ms < 8.0 else RED)
        print(f"  {layer:<20} {ms_c(f'{ms:.3f} ms')}  {_bar(ms, 0, 10, 16, ms_c)}")
    print(_sep(W))


# ── Async simulation loop ─────────────────────────────────────────────────────

_WAYPOINTS = [
    (2.5, 3.0),
    (7.5, 2.0),
    (7.0, 7.5),
    (3.0, 7.5),
    (5.0, 5.0),
    (1.5, 1.5),
]

async def main() -> None:
    sim = get_sim()
    sim.start_background()

    # Queue waypoints
    sim.set_waypoints(_WAYPOINTS)

    REFRESH_HZ = 2.5     # terminal update rate
    REFRESH_S  = 1.0 / REFRESH_HZ
    tick = 0

    print(BOLD(CYAN("Starting simulation…  Press Ctrl-C to stop.\n")))
    await asyncio.sleep(0.3)  # let physics spin up

    try:
        while True:
            t0 = time.perf_counter()
            s = sim.state
            cmd_vx    = sim._cmd_vx
            cmd_omega = sim._cmd_omega
            print_telemetry(sim, tick, cmd_vx, cmd_omega)
            tick += 1
            elapsed = time.perf_counter() - t0
            await asyncio.sleep(max(0, REFRESH_S - elapsed))
    except KeyboardInterrupt:
        print("\n" + BOLD(YELLOW("Simulation stopped by user.")))
        s = sim.state
        print(f"Final position: x={s.x:.3f}  y={s.y:.3f}  battery={s.battery:.1f}%")
        print(f"Path length: {len(s.full_path)} breadcrumbs")
        slam = sim.get_slam_state()
        print(f"Map explored: {slam['map_explored_pct']:.1f}%  "
              f"Position uncertainty: {slam['position_uncertainty_cm']:.2f} cm")
    finally:
        sim.stop()


if __name__ == "__main__":
    asyncio.run(main())
