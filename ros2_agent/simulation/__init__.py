"""2-D differential-drive robot simulation.

A lightweight pure-Python physics engine that runs as an asyncio background
task.  The mock ROS 2 layer reads/writes this singleton so that real agent
tool-calls (publish cmd_vel, send navigation goal, read /odom, read /scan)
actually move a virtual robot in a small obstacle-filled room.

The web server exposes a WebSocket endpoint (``/ws/sim``) that streams the
simulation state at ~20 Hz so the browser canvas can render live motion.
"""

from ros2_agent.simulation.robot_sim import RobotSim, get_sim

__all__ = ["RobotSim", "get_sim"]
