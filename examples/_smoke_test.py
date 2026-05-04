"""Quick integration smoke-test for all sensor layers."""
import asyncio
import sys

sys.path.insert(0, ".")
from ros2_agent.simulation.robot_sim import get_sim

sim = get_sim()

async def run():
    sim.start_background()
    sim.set_waypoints([(3.0, 4.0), (7.0, 7.0), (2.0, 2.0)])
    await asyncio.sleep(1.5)
    s = sim.state
    slam = sim.get_slam_state()
    joints = sim.get_joint_states()
    imu = sim.get_imu_state()

    print("=== ODOMETRY ===")
    print(f"  x={s.x:.4f}  y={s.y:.4f}  theta={s.theta:.4f}  vx={s.vx:.4f}  omega={s.omega:.4f}")

    print("=== JOINTS ===")
    wl = joints["wheel_left"]
    wr = joints["wheel_right"]
    print(f"  L vel={wl['velocity_rad_s']:.4f} r/s  effort={wl['effort_Nm']:.4f} Nm")
    print(f"  R vel={wr['velocity_rad_s']:.4f} r/s  effort={wr['effort_Nm']:.4f} Nm")

    print("=== IMU ===")
    print(f"  az={imu['linear_acceleration']['z']:.4f} m/s2  wz={imu['angular_velocity']['z']:.6f} r/s")

    print("=== SLAM ===")
    print(f"  pose x={slam['pose_x']:.4f}  uncertainty={slam['position_uncertainty_cm']:.2f} cm  explored={slam['map_explored_pct']}%")

    print("=== LIDAR ===")
    print(f"  rays={len(s.lidar)}  min={min(s.lidar):.3f}m  max={max(s.lidar):.3f}m  mean={sum(s.lidar)/len(s.lidar):.3f}m")

    print("=== CAMERA ===")
    print(f"  depth_pixels={len(s.camera_depth_row)}  detections={len(s.camera_detections)}")
    for d in s.camera_detections:
        print(f"    {d}")

    print("=== TIMING ===")
    print(f"  {s.timing}")

    print("=== BATTERY ===")
    print(f"  {s.battery:.2f}%")

    sim.stop()
    print("\nAll sensor layers OK.")

asyncio.run(run())
