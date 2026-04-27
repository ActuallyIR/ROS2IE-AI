"""ROS 2 parameter tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_param_tools(bridge: ROS2Bridge) -> list:
    """Create parameter-related tools bound to *bridge*."""

    @tool
    def list_params(node_name: str) -> str:
        """List all parameters of a ROS 2 node.

        Args:
            node_name: Node name (e.g. /controller_server, /amcl).
        """
        result = bridge.run(["param", "list", node_name])
        if not result.success:
            return f"Error listing parameters for {node_name}: {result.output}"
        return result.stdout or f"No parameters found for {node_name}."

    @tool
    def get_param(node_name: str, param_name: str) -> str:
        """Get the current value of a ROS 2 node parameter.

        Args:
            node_name: Node name (e.g. /controller_server).
            param_name: Parameter name (e.g. controller_frequency).
        """
        result = bridge.run(["param", "get", node_name, param_name])
        if not result.success:
            return f"Error getting {param_name} from {node_name}: {result.output}"
        return result.stdout.strip() or f"Parameter {param_name} not found on {node_name}."

    @tool
    def set_param(node_name: str, param_name: str, value: str) -> str:
        """Set a ROS 2 node parameter at runtime.

        Note: Not all parameters support runtime updates. Changes may not persist
        after the node restarts.

        Args:
            node_name: Node name (e.g. /controller_server).
            param_name: Parameter name (e.g. controller_frequency).
            value: New value as a string (e.g. '20.0', 'true', 'hello').
        """
        result = bridge.run(["param", "set", node_name, param_name, value])
        if not result.success:
            return f"Error setting {param_name} on {node_name}: {result.output}"
        return result.stdout.strip() or f"Parameter {param_name} set to {value} on {node_name}."

    @tool
    def dump_params(node_name: str) -> str:
        """Dump all parameter values of a node to YAML format.

        Args:
            node_name: Node name (e.g. /planner_server).
        """
        result = bridge.run(["param", "dump", node_name])
        if not result.success:
            return f"Error dumping parameters for {node_name}: {result.output}"
        return result.stdout or f"No parameters to dump for {node_name}."

    return [list_params, get_param, set_param, dump_params]
