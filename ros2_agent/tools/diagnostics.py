"""ROS 2 diagnostic tools — health checks, TF, and system status."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_diagnostic_tools(bridge: ROS2Bridge) -> list:
    """Create diagnostic tools bound to *bridge*."""

    @tool
    def run_ros_doctor() -> str:
        """Run `ros2 doctor` to check the overall ROS 2 system health.

        Checks network configuration, DDS middleware, and ROS 2 installation.
        """
        result = bridge.run(["doctor"], timeout=30)
        if not result.success:
            return f"ros2 doctor failed: {result.output}"
        return result.stdout or "ros2 doctor completed with no output."

    @tool
    def check_robot_health() -> str:
        """Perform a comprehensive robot health check.

        Reads diagnostics from /diagnostics topic, lists active nodes,
        and checks for recent error messages.
        """
        steps: list[str] = []

        # 1. Active nodes
        nodes_result = bridge.run(["node", "list"])
        node_count = len(nodes_result.stdout.strip().splitlines()) if nodes_result.success else 0
        steps.append(f"✓ Active nodes: {node_count}")

        # 2. Active topics
        topics_result = bridge.run(["topic", "list"])
        topic_count = len(topics_result.stdout.strip().splitlines()) if topics_result.success else 0
        steps.append(f"✓ Active topics: {topic_count}")

        # 3. Battery (if available)
        batt_result = bridge.run(["topic", "echo", "--once", "/battery_state"], timeout=3)
        if batt_result.success and batt_result.stdout:
            pct_lines = [l for l in batt_result.stdout.splitlines() if "percentage" in l.lower()]
            if pct_lines:
                steps.append(f"✓ Battery: {pct_lines[0].strip()}")
        else:
            steps.append("ℹ Battery topic not available.")

        # 4. Diagnostics topic
        diag_result = bridge.run(["topic", "echo", "--once", "/diagnostics"], timeout=3)
        if diag_result.success and diag_result.stdout:
            steps.append("✓ Diagnostics topic: receiving data")
        else:
            steps.append("ℹ /diagnostics topic not publishing.")

        # 5. ros2 doctor summary
        doctor_result = bridge.run(["doctor"], timeout=20)
        if doctor_result.success:
            first_line = doctor_result.stdout.strip().splitlines()[0] if doctor_result.stdout else ""
            steps.append(f"✓ ros2 doctor: {first_line}")
        else:
            steps.append(f"✗ ros2 doctor: {doctor_result.output}")

        return "=== Robot Health Report ===\n" + "\n".join(steps)

    @tool
    def list_tf_frames() -> str:
        """List all transform frames currently available in the TF tree."""
        result = bridge.run(
            ["run", "tf2_tools", "view_frames"],
            timeout=5,
        )
        if result.success:
            return result.stdout or "TF frames captured. Check frames.pdf in current directory."

        # Fallback: echo /tf_static for a quick frame list
        tf_result = bridge.run(["topic", "echo", "--once", "/tf_static"], timeout=5)
        if tf_result.success and tf_result.stdout:
            return f"TF static transforms:\n{tf_result.stdout}"

        return (
            "Could not retrieve TF tree. "
            "Ensure tf2_tools is installed: sudo apt install ros-$ROS_DISTRO-tf2-tools"
        )

    @tool
    def get_transform(source_frame: str, target_frame: str) -> str:
        """Get the current transform between two coordinate frames.

        Args:
            source_frame: Source frame (e.g. 'base_link', 'map').
            target_frame: Target frame (e.g. 'odom', 'base_footprint').
        """
        result = bridge.run(
            ["run", "tf2_ros", "tf2_echo", source_frame, target_frame],
            timeout=8,
        )
        if not result.success:
            return (
                f"Could not get transform {source_frame} → {target_frame}: {result.output}\n"
                "Make sure both frames exist in the TF tree."
            )
        return result.stdout or f"No transform data for {source_frame} → {target_frame}."

    @tool
    def get_diagnostics(timeout_sec: int = 3) -> str:
        """Read the /diagnostics topic to get hardware status reports.

        Args:
            timeout_sec: How long to wait for a diagnostics message (default 3).
        """
        result = bridge.run(
            ["topic", "echo", "--once", "/diagnostics"],
            timeout=timeout_sec + 2,
        )
        if not result.success:
            return f"Could not read /diagnostics: {result.output}"
        return result.stdout or "No diagnostics data received."

    return [
        run_ros_doctor,
        check_robot_health,
        list_tf_frames,
        get_transform,
        get_diagnostics,
    ]
