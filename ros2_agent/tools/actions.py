"""ROS 2 action tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_action_tools(bridge: ROS2Bridge) -> list:
    """Create action-related tools bound to *bridge*."""

    @tool
    def list_actions(filter_str: str = "") -> str:
        """List all available ROS 2 actions on the robot.

        Args:
            filter_str: Optional substring to filter by name.

        """
        result = bridge.run(["action", "list"])
        if not result.success:
            return f"Error listing actions: {result.output}"

        actions = [a for a in result.stdout.strip().splitlines() if a]
        if filter_str:
            actions = [a for a in actions if filter_str.lower() in a.lower()]

        if not actions:
            return "No actions found."

        return f"Found {len(actions)} action(s):\n" + "\n".join(f"  {a}" for a in sorted(actions))

    @tool
    def get_action_info(action_name: str) -> str:
        """Get details about a ROS 2 action: type, clients, and servers.

        Args:
            action_name: Action name (e.g. /navigate_to_pose).

        """
        result = bridge.run(["action", "info", "--count", action_name])
        if not result.success:
            return f"Error getting action info for {action_name}: {result.output}"
        return result.stdout or f"No info available for {action_name}."

    @tool
    def send_action_goal(
        action_name: str,
        action_type: str,
        goal_yaml: str,
        feedback: bool = False,
    ) -> str:
        """Send a goal to a ROS 2 action server and wait for the result.

        ⚠️  Navigation actions will physically move the robot!

        Args:
            action_name: Action server name (e.g. /navigate_to_pose).
            action_type: Action type (e.g. nav2_msgs/action/NavigateToPose).
            goal_yaml: Goal in YAML format.
            feedback: Whether to print feedback messages while executing.

        """
        args = ["action", "send_goal"]
        if feedback:
            args.append("--feedback")
        args += [action_name, action_type, goal_yaml]

        result = bridge.run(args, timeout=120)
        if not result.success:
            return f"Error sending goal to {action_name}: {result.output}"
        return result.stdout or "Goal sent. Check action server for status."

    return [list_actions, get_action_info, send_action_goal]
