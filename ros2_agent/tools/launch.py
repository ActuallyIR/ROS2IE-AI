"""ROS 2 launch and package tools."""

from __future__ import annotations

from langchain_core.tools import tool

from ros2_agent.ros2.bridge import ROS2Bridge


def create_launch_tools(bridge: ROS2Bridge) -> list:
    """Create launch and package tools bound to *bridge*."""

    @tool
    def list_packages(filter_str: str = "") -> str:
        """List installed ROS 2 packages.

        Args:
            filter_str: Optional substring to filter package names.
        """
        result = bridge.run(["pkg", "list"])
        if not result.success:
            return f"Error listing packages: {result.output}"

        pkgs = [p for p in result.stdout.strip().splitlines() if p]
        if filter_str:
            pkgs = [p for p in pkgs if filter_str.lower() in p.lower()]

        if not pkgs:
            return (
                f"No packages found matching '{filter_str}'."
                if filter_str
                else "No packages found."
            )

        return f"Found {len(pkgs)} package(s):\n" + "\n".join(f"  {p}" for p in sorted(pkgs))

    @tool
    def list_package_executables(package_name: str) -> str:
        """List all executables (nodes) available in a ROS 2 package.

        Args:
            package_name: ROS 2 package name (e.g. nav2_bringup).
        """
        result = bridge.run(["pkg", "executables", package_name])
        if not result.success:
            return f"Error listing executables in {package_name}: {result.output}"
        return result.stdout or f"No executables found in {package_name}."

    @tool
    def launch_package(
        package_name: str,
        launch_file: str,
        launch_args: str = "",
    ) -> str:
        """Launch a ROS 2 launch file from a package.

        ⚠️  This starts a new process — the agent will not be able to stop it
        directly. Use this when you intend to start a subsystem (e.g. Nav2, SLAM).

        Args:
            package_name: ROS 2 package (e.g. nav2_bringup).
            launch_file: Launch file name (e.g. bringup.launch.py).
            launch_args: Optional launch arguments as space-separated key:=value pairs.
        """
        args = ["launch", package_name, launch_file]
        if launch_args:
            args += launch_args.split()

        result = bridge.run(args, timeout=30)
        if not result.success:
            return f"Error launching {package_name}/{launch_file}: {result.output}"
        return result.stdout or f"Launched {package_name}/{launch_file} successfully."

    @tool
    def run_node(package_name: str, node_executable: str, node_args: str = "") -> str:
        """Run a single ROS 2 node from a package.

        Args:
            package_name: ROS 2 package (e.g. turtlesim).
            node_executable: Executable name inside the package (e.g. turtlesim_node).
            node_args: Additional ROS arguments (e.g. '--ros-args -p use_sim_time:=true').
        """
        args = ["run", package_name, node_executable]
        if node_args:
            args += node_args.split()

        result = bridge.run(args, timeout=15)
        if not result.success:
            return f"Error running {package_name}/{node_executable}: {result.output}"
        return result.stdout or f"Node {package_name}/{node_executable} started."

    @tool
    def record_bag(
        topics: str,
        output_name: str = "ros2bag_recording",
        duration_sec: int = 30,
    ) -> str:
        """Record ROS 2 topics to a bag file.

        Args:
            topics: Space-separated list of topics to record (e.g. '/scan /odom /tf').
                    Use '-a' to record all topics.
            output_name: Output bag file name prefix (default: ros2bag_recording).
            duration_sec: Recording duration in seconds (default 30).
        """
        topic_args = topics.split() if topics != "-a" else ["-a"]
        args = ["bag", "record", "-o", output_name, "--duration", str(duration_sec), *topic_args]

        result = bridge.run(args, timeout=duration_sec + 10)
        if not result.success:
            return f"Error recording bag: {result.output}"
        return result.stdout or f"Bag recorded to {output_name}/ ({duration_sec}s)."

    return [
        list_packages,
        list_package_executables,
        launch_package,
        run_node,
        record_bag,
    ]
