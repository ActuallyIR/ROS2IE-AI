"""ROS 2 topic tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_topic_tools(bridge: ROS2Bridge) -> list:
    """Create topic-related tools bound to *bridge*."""

    @tool
    def list_topics(filter_str: str = "") -> str:
        """List all active ROS 2 topics on the robot or simulation.

        Args:
            filter_str: Optional substring to filter results (e.g. 'camera', 'nav').

        """
        result = bridge.run(["topic", "list"])
        if not result.success:
            return f"Error listing topics: {result.output}"

        topics = [t for t in result.stdout.strip().splitlines() if t]
        if filter_str:
            topics = [t for t in topics if filter_str.lower() in t.lower()]

        if not topics:
            suffix = f" matching '{filter_str}'" if filter_str else ""
            return f"No topics found{suffix}."

        return f"Found {len(topics)} topic(s):\n" + "\n".join(f"  {t}" for t in sorted(topics))

    @tool
    def echo_topic(topic_name: str, count: int = 1, timeout_sec: int = 5) -> str:
        """Read one or more messages from a ROS 2 topic.

        Args:
            topic_name: Topic to read (e.g. /odom, /scan, /battery_state).
            count: Number of messages to read (default 1).
            timeout_sec: Seconds to wait for messages (default 5).

        """
        args = ["topic", "echo"]
        if count == 1:
            args.append("--once")
        else:
            args += ["--count", str(count)]
        args.append(topic_name)

        result = bridge.run(args, timeout=timeout_sec + 2)
        if not result.success:
            return f"Error reading {topic_name}: {result.output}"
        return result.stdout or f"No messages received on {topic_name} within {timeout_sec}s."

    @tool
    def get_topic_info(topic_name: str) -> str:
        """Get detailed info about a topic: message type, publisher and subscriber counts.

        Args:
            topic_name: Topic name (e.g. /cmd_vel).

        """
        result = bridge.run(["topic", "info", "--verbose", topic_name])
        if not result.success:
            return f"Error getting info for {topic_name}: {result.output}"
        return result.stdout or f"No information available for {topic_name}."

    @tool
    def publish_to_topic(
        topic_name: str,
        msg_type: str,
        message_yaml: str,
        times: int = 1,
    ) -> str:
        """Publish a message to a ROS 2 topic.

        ⚠️  Publishing to motion topics (e.g. /cmd_vel) will physically move the robot!
        Always confirm with the user before calling this on a real robot.

        Args:
            topic_name: Topic to publish to (e.g. /cmd_vel).
            msg_type: ROS 2 message type (e.g. geometry_msgs/msg/Twist).
            message_yaml: Message content in YAML format (e.g. "{linear: {x: 0.5}}").
            times: Number of times to publish (default 1).

        """
        args = ["topic", "pub"]
        if times == 1:
            args.append("--once")
        else:
            args += ["--times", str(times)]
        args += [topic_name, msg_type, message_yaml]

        result = bridge.run(args, timeout=max(15, times * 2))
        if not result.success:
            return f"Error publishing to {topic_name}: {result.output}"
        return f"Successfully published to {topic_name}.\n{result.stdout}"

    @tool
    def get_topic_hz(topic_name: str, window: int = 10) -> str:
        """Measure the publishing frequency (Hz) of a topic.

        Args:
            topic_name: Topic name to monitor.
            window: Number of messages to average over (default 10).

        """
        result = bridge.run(
            ["topic", "hz", "--window", str(window), topic_name],
            timeout=20,
        )
        if not result.success:
            return f"Error measuring frequency of {topic_name}: {result.output}"
        return result.stdout or "No frequency data — topic may not be publishing."

    @tool
    def get_topic_bandwidth(topic_name: str) -> str:
        """Measure the bandwidth (KB/s) used by a topic.

        Args:
            topic_name: Topic name to measure.

        """
        result = bridge.run(["topic", "bw", topic_name], timeout=15)
        if not result.success:
            return f"Error measuring bandwidth of {topic_name}: {result.output}"
        return result.stdout or "No bandwidth data available."

    return [
        list_topics,
        echo_topic,
        get_topic_info,
        publish_to_topic,
        get_topic_hz,
        get_topic_bandwidth,
    ]
