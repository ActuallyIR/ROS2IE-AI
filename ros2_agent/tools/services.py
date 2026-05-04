"""ROS 2 service tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_service_tools(bridge: ROS2Bridge) -> list:
    """Create service-related tools bound to *bridge*."""

    @tool
    def list_services(filter_str: str = "") -> str:
        """List all available ROS 2 services.

        Args:
            filter_str: Optional substring to filter results.

        """
        result = bridge.run(["service", "list"])
        if not result.success:
            return f"Error listing services: {result.output}"

        services = [s for s in result.stdout.strip().splitlines() if s]
        if filter_str:
            services = [s for s in services if filter_str.lower() in s.lower()]

        if not services:
            suffix = f" matching '{filter_str}'" if filter_str else ""
            return f"No services found{suffix}."

        return f"Found {len(services)} service(s):\n" + "\n".join(
            f"  {s}" for s in sorted(services)
        )

    @tool
    def get_service_type(service_name: str) -> str:
        """Get the message type of a ROS 2 service.

        Args:
            service_name: Full service name (e.g. /move_base/clear_costmaps).

        """
        result = bridge.run(["service", "type", service_name])
        if not result.success:
            return f"Error getting type for {service_name}: {result.output}"
        return f"Service type: {result.stdout.strip()}"

    @tool
    def call_service(service_name: str, service_type: str, request_yaml: str = "{}") -> str:
        """Call a ROS 2 service with an optional request payload.

        Args:
            service_name: Service to call (e.g. /move_base/clear_costmaps).
            service_type: Service type (e.g. std_srvs/srv/Empty).
            request_yaml: Request data in YAML format (default '{}' for empty request).

        """
        result = bridge.run(
            ["service", "call", service_name, service_type, request_yaml],
            timeout=15,
        )
        if not result.success:
            return f"Error calling service {service_name}: {result.output}"
        return result.stdout or "Service called successfully (no response data)."

    return [list_services, get_service_type, call_service]
