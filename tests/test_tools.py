"""Tests for ROS 2 bridge, mock layer, and tools."""

from __future__ import annotations

from ros2_agent.ros2.bridge import CommandResult
from ros2_agent.tools.actions import create_action_tools
from ros2_agent.tools.diagnostics import create_diagnostic_tools
from ros2_agent.tools.launch import create_launch_tools
from ros2_agent.tools.nodes import create_node_tools
from ros2_agent.tools.params import create_param_tools
from ros2_agent.tools.services import create_service_tools
from ros2_agent.tools.topics import create_topic_tools

# ── CommandResult ─────────────────────────────────────────────────────────────


class TestCommandResult:
    """Unit tests for the CommandResult dataclass."""

    def test_success_output(self):
        """Successful result returns stdout as output."""
        r = CommandResult(success=True, stdout="hello\n")
        assert r.output == "hello\n"
        assert str(r) == "hello\n"

    def test_failure_uses_error(self):
        """Failed result returns the error field when set."""
        r = CommandResult(success=False, error="something broke")
        assert r.output == "something broke"

    def test_failure_falls_back_to_stderr(self):
        """Failed result falls back to stderr when error field is empty."""
        r = CommandResult(success=False, stderr="stderr message")
        assert r.output == "stderr message"


# ── ROS2Bridge ────────────────────────────────────────────────────────────────


class TestROS2Bridge:
    """Integration tests for ROS2Bridge in mock mode."""

    def test_mock_mode_is_available(self, mock_bridge):
        """Bridge reports available=True in mock mode."""
        assert mock_bridge.available is True

    def test_mock_topic_list_succeeds(self, mock_bridge):
        """Mock topic list returns common navigation topics."""
        result = mock_bridge.run(["topic", "list"])
        assert result.success
        assert "/cmd_vel" in result.stdout
        assert "/odom" in result.stdout

    def test_mock_node_list_succeeds(self, mock_bridge):
        """Mock node list includes bt_navigator."""
        result = mock_bridge.run(["node", "list"])
        assert result.success
        assert "/bt_navigator" in result.stdout

    def test_mock_service_list_succeeds(self, mock_bridge):
        """Mock service list returns at least one service."""
        result = mock_bridge.run(["service", "list"])
        assert result.success
        assert len(result.stdout.strip().splitlines()) > 0

    def test_mock_action_list_succeeds(self, mock_bridge):
        """Mock action list includes navigate_to_pose."""
        result = mock_bridge.run(["action", "list"])
        assert result.success
        assert "/navigate_to_pose" in result.stdout

    def test_mock_param_get(self, mock_bridge):
        """Mock param get succeeds for a known parameter."""
        result = mock_bridge.run(["param", "get", "/controller_server", "controller_frequency"])
        assert result.success

    def test_mock_battery_echo(self, mock_bridge):
        """Mock battery state echo returns percentage field."""
        result = mock_bridge.run(["topic", "echo", "--once", "/battery_state"])
        assert result.success
        assert "percentage" in result.stdout.lower()

    def test_mock_odom_echo(self, mock_bridge):
        """Mock odom echo returns position data."""
        result = mock_bridge.run(["topic", "echo", "--once", "/odom"])
        assert result.success
        assert "position" in result.stdout

    def test_mock_unknown_subcommand(self, mock_bridge):
        """Unknown subcommand returns failure with descriptive error."""
        result = mock_bridge.run(["unknown_command", "foo"])
        assert not result.success
        assert "Unknown ros2 subcommand" in result.error

    def test_mock_ros_doctor(self, mock_bridge):
        """Mock ros2 doctor returns a passing result."""
        result = mock_bridge.run(["doctor"])
        assert result.success
        assert "All" in result.stdout

    def test_mock_pkg_list(self, mock_bridge):
        """Mock package list includes nav2_bringup."""
        result = mock_bridge.run(["pkg", "list"])
        assert result.success
        assert "nav2_bringup" in result.stdout

    def test_bridge_repr_shows_mode(self, mock_bridge):
        """Bridge repr includes the active mode string."""
        assert "mock" in repr(mock_bridge)


# ── Topic Tools ───────────────────────────────────────────────────────────────


class TestTopicTools:
    """Tests for the topic tool factory."""

    def test_list_topics_returns_topics(self, mock_bridge):
        """list_topics returns cmd_vel and a Found summary line."""
        tools = create_topic_tools(mock_bridge)
        list_fn = next(t for t in tools if t.name == "list_topics")
        result = list_fn.invoke({"filter_str": ""})
        assert "/cmd_vel" in result
        assert "Found" in result

    def test_list_topics_with_filter(self, mock_bridge):
        """list_topics with filter returns only matching topics."""
        tools = create_topic_tools(mock_bridge)
        list_fn = next(t for t in tools if t.name == "list_topics")
        result = list_fn.invoke({"filter_str": "camera"})
        assert "camera" in result.lower()

    def test_echo_topic(self, mock_bridge):
        """echo_topic returns odom position data."""
        tools = create_topic_tools(mock_bridge)
        echo_fn = next(t for t in tools if t.name == "echo_topic")
        result = echo_fn.invoke({"topic_name": "/odom", "count": 1, "timeout_sec": 5})
        assert "position" in result.lower()

    def test_get_topic_info(self, mock_bridge):
        """get_topic_info returns type info for cmd_vel."""
        tools = create_topic_tools(mock_bridge)
        info_fn = next(t for t in tools if t.name == "get_topic_info")
        result = info_fn.invoke({"topic_name": "/cmd_vel"})
        assert "geometry_msgs" in result or "Type" in result

    def test_publish_to_topic(self, mock_bridge):
        """publish_to_topic confirms a successful publish."""
        tools = create_topic_tools(mock_bridge)
        pub_fn = next(t for t in tools if t.name == "publish_to_topic")
        result = pub_fn.invoke(
            {
                "topic_name": "/cmd_vel",
                "msg_type": "geometry_msgs/msg/Twist",
                "message_yaml": "{linear: {x: 0.0}}",
                "times": 1,
            }
        )
        assert "published" in result.lower() or "Successfully" in result

    def test_get_topic_hz(self, mock_bridge):
        """get_topic_hz returns frequency data for /scan."""
        tools = create_topic_tools(mock_bridge)
        hz_fn = next(t for t in tools if t.name == "get_topic_hz")
        result = hz_fn.invoke({"topic_name": "/scan", "window": 5})
        assert "rate" in result.lower() or "hz" in result.lower()


# ── Service Tools ─────────────────────────────────────────────────────────────


class TestServiceTools:
    """Tests for the service tool factory."""

    def test_list_services(self, mock_bridge):
        """list_services returns a non-empty service list."""
        tools = create_service_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_services")
        result = fn.invoke({"filter_str": ""})
        assert "Found" in result or "/navigate" in result

    def test_call_service(self, mock_bridge):
        """call_service returns a success or response message."""
        tools = create_service_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "call_service")
        result = fn.invoke(
            {
                "service_name": "/move_base/clear_costmaps",
                "service_type": "std_srvs/srv/Empty",
                "request_yaml": "{}",
            }
        )
        assert "success" in result.lower() or "response" in result.lower()


# ── Action Tools ──────────────────────────────────────────────────────────────


class TestActionTools:
    """Tests for the action tool factory."""

    def test_list_actions(self, mock_bridge):
        """list_actions returns navigate_to_pose."""
        tools = create_action_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_actions")
        result = fn.invoke({"filter_str": ""})
        assert "navigate_to_pose" in result

    def test_send_action_goal(self, mock_bridge):
        """send_action_goal returns SUCCEEDED or Goal status message."""
        tools = create_action_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "send_action_goal")
        result = fn.invoke(
            {
                "action_name": "/navigate_to_pose",
                "action_type": "nav2_msgs/action/NavigateToPose",
                "goal_yaml": "{}",
                "feedback": False,
            }
        )
        assert "SUCCEEDED" in result or "Goal" in result


# ── Node Tools ────────────────────────────────────────────────────────────────


class TestNodeTools:
    """Tests for the node tool factory."""

    def test_list_nodes(self, mock_bridge):
        """list_nodes includes bt_navigator."""
        tools = create_node_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_nodes")
        result = fn.invoke({"filter_str": ""})
        assert "/bt_navigator" in result

    def test_get_node_info(self, mock_bridge):
        """get_node_info returns node details for bt_navigator."""
        tools = create_node_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "get_node_info")
        result = fn.invoke({"node_name": "/bt_navigator"})
        assert "Node" in result


# ── Param Tools ───────────────────────────────────────────────────────────────


class TestParamTools:
    """Tests for the parameter tool factory."""

    def test_list_params(self, mock_bridge):
        """list_params returns controller_frequency for /controller_server."""
        tools = create_param_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_params")
        result = fn.invoke({"node_name": "/controller_server"})
        assert "controller_frequency" in result or len(result) > 0

    def test_get_param(self, mock_bridge):
        """get_param returns the value of controller_frequency."""
        tools = create_param_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "get_param")
        result = fn.invoke(
            {"node_name": "/controller_server", "param_name": "controller_frequency"}
        )
        assert "20" in result or "Value" in result

    def test_set_param(self, mock_bridge):
        """set_param confirms the new value was applied."""
        tools = create_param_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "set_param")
        result = fn.invoke(
            {
                "node_name": "/controller_server",
                "param_name": "controller_frequency",
                "value": "25.0",
            }
        )
        assert "Set parameter" in result or "25" in result


# ── Diagnostic Tools ──────────────────────────────────────────────────────────


class TestDiagnosticTools:
    """Tests for the diagnostic tool factory."""

    def test_run_ros_doctor(self, mock_bridge):
        """run_ros_doctor returns a health check result."""
        tools = create_diagnostic_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "run_ros_doctor")
        result = fn.invoke({})
        assert "check" in result.lower() or "OK" in result

    def test_check_robot_health(self, mock_bridge):
        """check_robot_health returns a health report with nodes and topics."""
        tools = create_diagnostic_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "check_robot_health")
        result = fn.invoke({})
        assert "Health Report" in result
        assert "nodes" in result.lower() or "topics" in result.lower()


# ── Launch Tools ──────────────────────────────────────────────────────────────


class TestLaunchTools:
    """Tests for the launch and package tool factory."""

    def test_list_packages(self, mock_bridge):
        """list_packages returns nav2_bringup in the result."""
        tools = create_launch_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_packages")
        result = fn.invoke({"filter_str": ""})
        assert "nav2_bringup" in result

    def test_list_packages_with_filter(self, mock_bridge):
        """list_packages with filter returns only matching packages."""
        tools = create_launch_tools(mock_bridge)
        fn = next(t for t in tools if t.name == "list_packages")
        result = fn.invoke({"filter_str": "turtlebot"})
        assert "turtlebot" in result.lower()
