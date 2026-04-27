"""ROS 2 tools for the LangGraph agent."""

from __future__ import annotations

from ros2_agent.ros2.bridge import ROS2Bridge
from ros2_agent.tools.actions import create_action_tools
from ros2_agent.tools.diagnostics import create_diagnostic_tools
from ros2_agent.tools.launch import create_launch_tools
from ros2_agent.tools.logs import create_log_tools
from ros2_agent.tools.nodes import create_node_tools
from ros2_agent.tools.params import create_param_tools
from ros2_agent.tools.services import create_service_tools
from ros2_agent.tools.topics import create_topic_tools


def get_all_tools(bridge: ROS2Bridge) -> list:
    """Return all ROS 2 tools bound to the given bridge."""
    return [
        *create_topic_tools(bridge),
        *create_service_tools(bridge),
        *create_action_tools(bridge),
        *create_node_tools(bridge),
        *create_log_tools(bridge),
        *create_param_tools(bridge),
        *create_diagnostic_tools(bridge),
        *create_launch_tools(bridge),
    ]


__all__ = ["get_all_tools"]
