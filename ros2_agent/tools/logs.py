"""ROS 2 log tools — reads from /rosout and journal."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_log_tools(bridge: ROS2Bridge) -> list:
    """Create log-related tools bound to *bridge*."""

    @tool
    def get_ros_logs(
        count: int = 20,
        level: str = "",
        node_filter: str = "",
    ) -> str:
        """Retrieve recent ROS 2 log messages from /rosout.

        Args:
            count: Number of messages to retrieve (default 20).
            level: Minimum log level filter: DEBUG | INFO | WARN | ERROR | FATAL.
            node_filter: Filter messages from a specific node name substring.

        """
        # Echo /rosout with a count limit
        result = bridge.run(
            ["topic", "echo", "--count", str(count), "--no-daemon", "/rosout"],
            timeout=count + 5,
        )
        if not result.success:
            # Fallback: try reading the log directory via `ros2 daemon`
            return f"Could not read /rosout: {result.output}"

        lines = result.stdout.strip().splitlines()
        if not lines:
            return "No log messages found."

        filtered: list[str] = []
        for line in lines:
            if level and level.upper() not in line.upper():
                continue
            if node_filter and node_filter.lower() not in line.lower():
                continue
            filtered.append(line)

        if not filtered:
            conds = []
            if level:
                conds.append(f"level={level}")
            if node_filter:
                conds.append(f"node={node_filter}")
            return "No log messages matching: " + ", ".join(conds)

        return "\n".join(filtered)

    @tool
    def get_error_logs(count: int = 10) -> str:
        """Get the most recent ERROR and FATAL log messages from /rosout.

        Args:
            count: Maximum number of error messages to return (default 10).

        """
        result = bridge.run(
            ["topic", "echo", "--count", str(count * 4), "--no-daemon", "/rosout"],
            timeout=30,
        )
        if not result.success:
            return f"Could not read logs: {result.output}"

        lines = result.stdout.strip().splitlines()
        errors = [line for line in lines if "ERROR" in line.upper() or "FATAL" in line.upper()][
            :count
        ]

        if not errors:
            return "No ERROR or FATAL log messages found. System appears healthy."

        return f"Found {len(errors)} error(s):\n" + "\n".join(errors)

    @tool
    def explain_errors(count: int = 10) -> str:
        """Retrieve recent errors from ROS 2 logs and provide a diagnostic summary.

        This tool reads the latest error messages and returns them formatted for
        analysis — the LLM will then explain causes and suggest fixes.

        Args:
            count: Number of recent error messages to analyse (default 10).

        """
        result = bridge.run(
            ["topic", "echo", "--count", str(count * 5), "--no-daemon", "/rosout"],
            timeout=30,
        )
        if not result.success:
            return f"Could not read logs: {result.output}"

        lines = result.stdout.strip().splitlines()
        issues = [
            line
            for line in lines
            if any(keyword in line.upper() for keyword in ("ERROR", "FATAL", "WARN"))
        ][:count]

        if not issues:
            return "No warnings or errors found in recent logs. System looks healthy."

        header = f"Found {len(issues)} warning(s)/error(s) in recent logs:\n"
        return header + "\n".join(f"  [{i + 1}] {line}" for i, line in enumerate(issues))

    return [get_ros_logs, get_error_logs, explain_errors]
