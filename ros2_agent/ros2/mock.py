"""Mock ROS 2 environment simulating a TurtleBot4 in Gazebo.

Used when ``--mock`` is passed so the agent can be demonstrated and
tested without a running ROS 2 installation.
"""

from __future__ import annotations

import json
import random
import time
from typing import Any

from ros2_agent.ros2.bridge import CommandResult
from ros2_agent.simulation.robot_sim import get_sim

# ── Simulated robot state ─────────────────────────────────────────────────────

_TOPICS: list[str] = [
    "/battery_state",
    "/camera/color/image_raw",
    "/camera/depth/image_rect_raw",
    "/cmd_vel",
    "/diagnostics",
    "/dock",
    "/imu",
    "/joint_states",
    "/map",
    "/odom",
    "/parameter_events",
    "/rosout",
    "/scan",
    "/tf",
    "/tf_static",
    "/wheel_vels",
    "/initialpose",
    "/goal_pose",
    "/navigate_to_pose/_action/feedback",
    "/navigate_to_pose/_action/status",
    "/global_costmap/costmap",
    "/local_costmap/costmap",
]

_SERVICES: list[str] = [
    "/controller_server/describe_parameters",
    "/global_costmap/describe_parameters",
    "/map_server/map",
    "/move_base/clear_costmaps",
    "/navigate_to_pose",
    "/planner_server/describe_parameters",
    "/robot_state_publisher/describe_parameters",
    "/slam_toolbox/save_map",
    "/slam_toolbox/serialize_map",
]

_ACTIONS: list[str] = [
    "/navigate_to_pose",
    "/navigate_through_poses",
    "/spin",
    "/backup",
    "/follow_waypoints",
]

_NODES: list[str] = [
    "/amcl",
    "/bt_navigator",
    "/controller_server",
    "/gazebo",
    "/global_costmap/global_costmap",
    "/joint_state_publisher",
    "/lifecycle_manager_navigation",
    "/local_costmap/local_costmap",
    "/map_server",
    "/planner_server",
    "/robot_state_publisher",
    "/slam_toolbox",
    "/transform_listener_impl",
    "/turtlebot4_node",
    "/velocity_smoother",
    "/waypoint_follower",
]

_PACKAGES: list[str] = [
    "nav2_bringup",
    "nav2_bt_navigator",
    "nav2_controller",
    "nav2_planner",
    "slam_toolbox",
    "turtlebot4_bringup",
    "turtlebot4_description",
    "turtlebot4_navigation",
    "turtlebot4_simulator",
    "gazebo_ros",
    "robot_state_publisher",
    "joint_state_publisher",
    "rviz2",
    "ros2bag",
    "tf2_ros",
]

_MOCK_LOGS: list[dict[str, str]] = [
    {"level": "INFO", "node": "/bt_navigator", "msg": "Beginning navigation to (2.50, 3.00)"},
    {"level": "INFO", "node": "/controller_server", "msg": "Received goal, begin following path"},
    {"level": "WARN", "node": "/local_costmap", "msg": "Costmap is not fully initialized yet"},
    {"level": "INFO", "node": "/planner_server", "msg": "Creating path using NavFn planner"},
    {"level": "INFO", "node": "/velocity_smoother", "msg": "Velocity smoother started"},
    {"level": "ERROR", "node": "/amcl", "msg": "AMCL covariance high — localization uncertain"},
    {"level": "INFO", "node": "/turtlebot4_node", "msg": "Battery charge: 78%"},
    {"level": "WARN", "node": "/controller_server", "msg": "DWB controller: obstacle too close"},
    {"level": "INFO", "node": "/map_server", "msg": "Map loaded: 100x100, resolution 0.05m/px"},
    {"level": "INFO", "node": "/slam_toolbox", "msg": "Processing scan, pose updated"},
]

_NODE_PARAMS: dict[str, dict[str, Any]] = {
    "/controller_server": {
        "FollowPath.max_vel_x": 0.26,
        "FollowPath.min_vel_x": -0.26,
        "FollowPath.max_vel_theta": 1.0,
        "controller_frequency": 20.0,
    },
    "/amcl": {
        "max_particles": 2000,
        "min_particles": 500,
        "update_min_a": 0.2,
        "update_min_d": 0.25,
    },
    "/planner_server": {
        "GridBased.allow_unknown": True,
        "GridBased.tolerance": 0.5,
        "GridBased.use_astar": False,
        "planner_frequency": 1.0,
    },
}


class MockROS2:
    """Static handler that interprets `ros2` CLI args and returns realistic mock data."""

    @classmethod
    def run(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="No ros2 subcommand given")

        sub = args[0]
        rest = args[1:]

        dispatch = {
            "topic": cls._topic,
            "service": cls._service,
            "action": cls._action,
            "node": cls._node,
            "param": cls._param,
            "pkg": cls._pkg,
            "launch": cls._launch,
            "run": cls._run,
            "bag": cls._bag,
            "doctor": cls._doctor,
            "wtf": cls._doctor,
        }

        handler = dispatch.get(sub)
        if handler is None:
            return CommandResult(
                success=False,
                error=f"[mock] Unknown ros2 subcommand: {sub!r}",
            )
        return handler(rest)

    # ── Topic ──────────────────────────────────────────────────────────────────

    @classmethod
    def _topic(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 topic <cmd> required")

        cmd = args[0]
        rest = args[1:]

        if cmd == "list":
            return CommandResult(success=True, stdout="\n".join(_TOPICS) + "\n")

        if cmd == "echo":
            topic = next((a for a in rest if not a.startswith("-")), "")
            return CommandResult(success=True, stdout=cls._generate_topic_msg(topic))

        if cmd == "info":
            topic = next((a for a in rest if not a.startswith("-")), "")
            return CommandResult(success=True, stdout=cls._topic_info(topic))

        if cmd == "pub":
            # Extract topic name and payload (first two positionals after flags)
            positionals = [a for a in rest if not a.startswith("-")]
            topic = positionals[0] if positionals else "unknown"
            # If publishing to cmd_vel, feed the velocity into the live sim
            if "/cmd_vel" in topic and len(positionals) >= 3:
                try:
                    payload = json.loads(positionals[2])
                    lin = payload.get("linear", {})
                    ang = payload.get("angular", {})
                    vx = float(lin.get("x", 0.0))
                    omega = float(ang.get("z", 0.0))
                    get_sim().set_cmd_vel(vx, omega)
                except Exception:  # noqa: BLE001
                    pass
            return CommandResult(
                success=True,
                stdout=f"publisher: beginning loop, publishing to '{topic}'\npublished: 1\n",
            )

        if cmd == "hz":
            topic = next((a for a in rest if not a.startswith("-")), "")
            hz = round(random.uniform(9.5, 10.5), 2)
            return CommandResult(
                success=True,
                stdout=(
                    f"average rate: {hz}\n"
                    f"\tmin: {hz - 0.1:.3f}s max: {hz + 0.1:.3f}s std dev: 0.00050s "
                    f"window: 10\n"
                ),
            )

        if cmd == "bw":
            topic = next((a for a in rest if not a.startswith("-")), "")
            bw = round(random.uniform(1.0, 50.0), 2)
            return CommandResult(
                success=True, stdout=f"average: {bw} KB/s\n\tmin: {bw * 0.9:.2f} KB/s\n"
            )

        return CommandResult(success=False, error=f"[mock] Unknown topic sub-command: {cmd}")

    @classmethod
    def _generate_topic_msg(cls, topic: str) -> str:
        ts = time.time()
        sec = int(ts)
        nanosec = int((ts - sec) * 1e9)

        if "battery" in topic:
            pct = get_sim().state.battery / 100.0
            volts = 14.0 + pct * 2.5
            return (
                f"header:\n  stamp:\n    sec: {sec}\n    nanosec: {nanosec}\n"
                f"  frame_id: ''\n"
                f"voltage: {volts:.2f}\n"
                f"percentage: {pct:.4f}\n"
                f"power_supply_status: 2\npower_supply_health: 1\npresent: true\n"
            )
        if "odom" in topic:
            sim = get_sim()
            s = sim.state
            import math as _math
            qz = _math.sin(s.theta / 2)
            qw = _math.cos(s.theta / 2)
            return (
                f"header:\n  stamp:\n    sec: {sec}\n    nanosec: {nanosec}\n"
                f"  frame_id: odom\n"
                f"child_frame_id: base_link\n"
                f"pose:\n  pose:\n    position:\n      x: {s.x:.4f}\n      y: {s.y:.4f}\n      z: 0.0\n"
                f"    orientation:\n      x: 0.0\n      y: 0.0\n      z: {qz:.4f}\n"
                f"      w: {qw:.4f}\n"
                f"twist:\n  twist:\n    linear:\n      x: {s.vx:.4f}\n"
                f"      y: 0.0\n      z: 0.0\n"
                f"    angular:\n      x: 0.0\n      y: 0.0\n      z: {s.omega:.4f}\n"
            )
        if "scan" in topic:
            sim_ranges = get_sim().state.lidar
            if sim_ranges:
                range_str = ", ".join(str(r) for r in sim_ranges)
            else:
                range_str = ", ".join(str(round(random.uniform(0.5, 5.0), 3)) for _ in range(32))
            return (
                f"header:\n  stamp:\n    sec: {sec}\n  frame_id: base_scan\n"
                f"angle_min: -3.1416\nangle_max: 3.1416\nangle_increment: 0.1963\n"
                f"range_min: 0.1\nrange_max: 6.0\n"
                f"ranges: [{range_str}]\n"
            )
        if "joint" in topic:
            return (
                f"header:\n  stamp:\n    sec: {sec}\n  frame_id: ''\n"
                f"name: [wheel_left_joint, wheel_right_joint]\n"
                f"position: [{random.uniform(-3.14, 3.14):.4f}, {random.uniform(-3.14, 3.14):.4f}]\n"
                f"velocity: [{random.uniform(-0.5, 0.5):.4f}, {random.uniform(-0.5, 0.5):.4f}]\n"
                f"effort: []\n"
            )
        if "imu" in topic:
            return (
                f"header:\n  stamp:\n    sec: {sec}\n  frame_id: imu_link\n"
                f"orientation:\n  x: 0.0\n  y: 0.0\n  z: {random.uniform(-1,1):.4f}\n  w: {random.uniform(0,1):.4f}\n"
                f"angular_velocity:\n  x: {random.uniform(-0.01, 0.01):.6f}\n"
                f"  y: {random.uniform(-0.01, 0.01):.6f}\n"
                f"  z: {random.uniform(-0.05, 0.05):.6f}\n"
                f"linear_acceleration:\n  x: {random.uniform(-0.1, 0.1):.6f}\n"
                f"  y: {random.uniform(-0.1, 0.1):.6f}\n"
                f"  z: {random.uniform(9.7, 9.9):.6f}\n"
            )
        # Generic fallback
        return f"data: '{topic} message at t={sec}'\n"

    @classmethod
    def _topic_info(cls, topic: str) -> str:
        type_map = {
            "/battery_state": "sensor_msgs/msg/BatteryState",
            "/cmd_vel": "geometry_msgs/msg/Twist",
            "/odom": "nav_msgs/msg/Odometry",
            "/scan": "sensor_msgs/msg/LaserScan",
            "/imu": "sensor_msgs/msg/Imu",
            "/map": "nav_msgs/msg/OccupancyGrid",
            "/joint_states": "sensor_msgs/msg/JointState",
            "/tf": "tf2_msgs/msg/TFMessage",
        }
        msg_type = type_map.get(topic, "std_msgs/msg/String")
        return (
            f"Type: {msg_type}\n\n"
            f"Publisher count: {random.randint(1, 2)}\n"
            f"Subscription count: {random.randint(0, 3)}\n\n"
            f"Node name: /turtlebot4_node\n"
            f"Node namespace: /\n"
            f"Topic type: {msg_type}\n"
            f"Endpoint type: PUBLISHER\n"
            f"GID: 01.0f.0a.{random.randint(10, 99)}.00.00.00.00.00.00.00.00\n"
            f"QoS profile:\n"
            f"  Reliability: RELIABLE\n"
            f"  History (Depth): KEEP_LAST (10)\n"
            f"  Durability: VOLATILE\n"
            f"  Lifespan: Infinite\n"
            f"  Deadline: Infinite\n"
            f"  Liveliness: AUTOMATIC\n"
            f"  Liveliness lease duration: Infinite\n"
        )

    # ── Service ────────────────────────────────────────────────────────────────

    @classmethod
    def _service(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 service <cmd> required")
        cmd = args[0]
        rest = args[1:]

        if cmd == "list":
            return CommandResult(success=True, stdout="\n".join(_SERVICES) + "\n")

        if cmd == "call":
            srv = next((a for a in rest if not a.startswith("-")), "")
            return CommandResult(
                success=True,
                stdout=f"requester: making request: {srv}\nresponse:\n  success: True\n  message: ''\n",
            )

        if cmd == "type":
            srv = next((a for a in rest if not a.startswith("-")), "")
            type_map = {
                "/map_server/map": "nav2_msgs/srv/GetCostmap",
                "/slam_toolbox/save_map": "nav2_msgs/srv/SaveMap",
                "/move_base/clear_costmaps": "std_srvs/srv/Empty",
            }
            stype = type_map.get(srv, "std_srvs/srv/Empty")
            return CommandResult(success=True, stdout=stype + "\n")

        return CommandResult(success=False, error=f"[mock] Unknown service sub-command: {cmd}")

    # ── Action ────────────────────────────────────────────────────────────────

    @classmethod
    def _action(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 action <cmd> required")
        cmd = args[0]
        rest = args[1:]

        if cmd == "list":
            return CommandResult(success=True, stdout="\n".join(_ACTIONS) + "\n")

        if cmd == "send_goal":
            positionals = [a for a in rest if not a.startswith("-")]
            action = positionals[0] if positionals else "unknown"
            goal_id = random.randint(100000, 999999)

            # Parse goal pose from the YAML/JSON payload if provided
            if len(positionals) >= 3:
                try:
                    payload = json.loads(positionals[2])
                    pose = payload.get("pose", payload)
                    pos = pose.get("pose", pose).get("position", pose)
                    gx = float(pos.get("x", 2.0))
                    gy = float(pos.get("y", 2.0))
                    get_sim().set_goal(gx, gy, goal_id=goal_id)
                except Exception:  # noqa: BLE001
                    # Try key=value style: x: 3.0, y: 1.5
                    try:
                        import re
                        raw = " ".join(positionals[1:])
                        gx = float(re.search(r"x[:\s]+([\d.\-]+)", raw).group(1))  # type: ignore[union-attr]
                        gy = float(re.search(r"y[:\s]+([\d.\-]+)", raw).group(1))  # type: ignore[union-attr]
                        get_sim().set_goal(gx, gy, goal_id=goal_id)
                    except Exception:  # noqa: BLE001
                        get_sim().set_goal(2.0, 2.0, goal_id=goal_id)
            else:
                get_sim().set_goal(2.0, 2.0, goal_id=goal_id)

            return CommandResult(
                success=True,
                stdout=(
                    f"Sending goal:\n     {action}\n\n"
                    f"Goal accepted with ID: {goal_id}\n"
                    f"Result:\n  result: {{}}\n"
                    f"Goal finished with status: SUCCEEDED\n"
                ),
            )

        if cmd == "info":
            action = next((a for a in rest if not a.startswith("-")), "")
            return CommandResult(
                success=True,
                stdout=(
                    f"Action: {action}\nAction clients: 1\nAction servers: 1\n"
                    f"Node name: /bt_navigator\n"
                ),
            )

        return CommandResult(success=False, error=f"[mock] Unknown action sub-command: {cmd}")

    # ── Node ──────────────────────────────────────────────────────────────────

    @classmethod
    def _node(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 node <cmd> required")
        cmd = args[0]
        rest = args[1:]

        if cmd == "list":
            return CommandResult(success=True, stdout="\n".join(_NODES) + "\n")

        if cmd == "info":
            node = next((a for a in rest if not a.startswith("-")), "")
            return CommandResult(
                success=True,
                stdout=(
                    f"Node: {node}\nNamespace: /\n"
                    f"Types:\n  lifecycle\nPublishers:\n  /rosout: rcl_interfaces/msg/Log\n"
                    f"  /tf: tf2_msgs/msg/TFMessage\n"
                    f"Subscribers:\n  /parameter_events: rcl_interfaces/msg/ParameterEvent\n"
                    f"Service Servers:\n  {node}/describe_parameters: "
                    f"rcl_interfaces/srv/DescribeParameters\n"
                    f"Service Clients:\n\nAction Servers:\nAction Clients:\n"
                ),
            )

        return CommandResult(success=False, error=f"[mock] Unknown node sub-command: {cmd}")

    # ── Param ─────────────────────────────────────────────────────────────────

    @classmethod
    def _param(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 param <cmd> required")
        cmd = args[0]
        rest = args[1:]

        if cmd == "list":
            node = next((a for a in rest if not a.startswith("-")), "/controller_server")
            params = _NODE_PARAMS.get(node, {"use_sim_time": True, "qos_overrides": {}})
            return CommandResult(
                success=True,
                stdout="\n".join(f"  {k}" for k in params) + "\n",
            )

        if cmd == "get":
            positionals = [a for a in rest if not a.startswith("-")]
            node = positionals[0] if len(positionals) > 0 else "/controller_server"
            param = positionals[1] if len(positionals) > 1 else "controller_frequency"
            params = _NODE_PARAMS.get(node, {})
            value = params.get(param, 10.0)
            return CommandResult(
                success=True,
                stdout=f"{param}:\n  Type: double\n  Value: {value}\n",
            )

        if cmd == "set":
            positionals = [a for a in rest if not a.startswith("-")]
            param = positionals[1] if len(positionals) > 1 else "param"
            value = positionals[2] if len(positionals) > 2 else "value"
            return CommandResult(
                success=True, stdout=f"Set parameter successful: {param} = {value}\n"
            )

        return CommandResult(success=False, error=f"[mock] Unknown param sub-command: {cmd}")

    # ── Package ───────────────────────────────────────────────────────────────

    @classmethod
    def _pkg(cls, args: list[str]) -> CommandResult:
        if not args:
            return CommandResult(success=False, error="ros2 pkg <cmd> required")
        cmd = args[0]

        if cmd == "list":
            return CommandResult(success=True, stdout="\n".join(sorted(_PACKAGES)) + "\n")

        if cmd == "executables":
            rest = args[1:]
            pkg = next((a for a in rest if not a.startswith("-")), "nav2_bringup")
            return CommandResult(
                success=True, stdout=f"{pkg}  bringup\n{pkg}  localization\n"
            )

        return CommandResult(success=False, error=f"[mock] Unknown pkg sub-command: {cmd}")

    # ── Launch ────────────────────────────────────────────────────────────────

    @classmethod
    def _launch(cls, args: list[str]) -> CommandResult:
        pkg = args[0] if len(args) > 0 else "unknown_pkg"
        launch_file = args[1] if len(args) > 1 else "bringup.launch.py"
        return CommandResult(
            success=True,
            stdout=(
                f"[INFO] [launch]: All log files can be found below /root/.ros/log/\n"
                f"[INFO] [launch]: Default logging verbosity is set to INFO\n"
                f"[INFO] [{pkg}]: process started with pid [12345]\n"
                f"[INFO] [launch_ros.actions.node]: process[{pkg}-1]: started with pid [12345]\n"
                f"[{pkg}]: Launching {launch_file}\n"
            ),
        )

    # ── Run ───────────────────────────────────────────────────────────────────

    @classmethod
    def _run(cls, args: list[str]) -> CommandResult:
        pkg = args[0] if len(args) > 0 else "unknown"
        exe = args[1] if len(args) > 1 else "node"
        return CommandResult(
            success=True,
            stdout=f"[INFO] [{pkg}]: Starting {exe}\n",
        )

    # ── Bag ───────────────────────────────────────────────────────────────────

    @classmethod
    def _bag(cls, args: list[str]) -> CommandResult:
        cmd = args[0] if args else "record"
        if cmd == "record":
            return CommandResult(
                success=True,
                stdout="[INFO] [rosbag2_recorder]: Listening for topics...\n[INFO] [rosbag2_recorder]: Recording started\n",
            )
        if cmd == "info":
            bag_file = args[1] if len(args) > 1 else "rosbag2"
            return CommandResult(
                success=True,
                stdout=(
                    f"Files:             {bag_file}_0.db3\n"
                    f"Bag size:          14.4 MiB\n"
                    f"Storage id:        sqlite3\n"
                    f"Duration:          30.031s\n"
                    f"Start:             Apr 27 2026 10:00:00.000 (1745748000.000)\n"
                    f"End:               Apr 27 2026 10:00:30.031 (1745748030.031)\n"
                    f"Messages:          9012\n"
                    f"Topic information: Topic: /scan | Type: sensor_msgs/msg/LaserScan | "
                    f"Count: 301 | Serialization Format: cdr\n"
                ),
            )
        return CommandResult(success=False, error=f"[mock] Unknown bag sub-command: {cmd}")

    # ── Doctor ────────────────────────────────────────────────────────────────

    @classmethod
    def _doctor(cls, _args: list[str]) -> CommandResult:
        return CommandResult(
            success=True,
            stdout=(
                "All 5 checks passed\n\n"
                "ros2doctor: Found 0 error(s), 0 warning(s) in 5 packages\n\n"
                " network             : [OK]\n"
                " platform            : [OK]\n"
                " rmw                 : [OK]\n"
                " ros2doctor          : [OK]\n"
                " topic checker       : [OK]\n"
            ),
        )
