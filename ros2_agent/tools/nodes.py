"""ROS 2 node tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_node_tools(bridge: ROS2Bridge) -> list:
    """Create node-related tools bound to *bridge*."""

    @tool
    def list_nodes(filter_str: str = "") -> str:
        """List all running ROS 2 nodes.

        Args:
            filter_str: Optional substring to filter by name.
        """
        result = bridge.run(["node", "list"])
        if not result.success:
            return f"Error listing nodes: {result.output}"

        nodes = [n for n in result.stdout.strip().splitlines() if n]
        if filter_str:
            nodes = [n for n in nodes if filter_str.lower() in n.lower()]

        if not nodes:
            return "No nodes found."

        return f"Found {len(nodes)} node(s):\n" + "\n".join(f"  {n}" for n in sorted(nodes))

    @tool
    def get_node_info(node_name: str) -> str:
        """Get detailed information about a running ROS 2 node.

        Shows publishers, subscribers, service servers/clients, and action servers/clients.

        Args:
            node_name: Node name (e.g. /bt_navigator).
        """
        result = bridge.run(["node", "info", node_name])
        if not result.success:
            return f"Error getting info for {node_name}: {result.output}"
        return result.stdout or f"No info available for {node_name}."

    return [list_nodes, get_node_info]
