"""
ROS2 Agent - LLM-powered agent for ROS 2 robots.

Control, monitor, and debug your ROS 2 robots using natural language.
"""

__version__ = "0.1.0"
__author__ = "ROS2 Agent Contributors"
__license__ = "MIT"

from ros2_agent.agent.core import ROS2Agent
from ros2_agent.config.settings import Settings

__all__ = ["ROS2Agent", "Settings", "__version__"]
