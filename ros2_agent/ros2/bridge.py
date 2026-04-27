"""ROS 2 subprocess bridge — executes `ros2` CLI commands safely."""

from __future__ import annotations

import os
import shutil
import subprocess
from dataclasses import dataclass, field
from typing import Optional


@dataclass
class CommandResult:
    """Result of a `ros2` CLI invocation."""

    success: bool
    stdout: str = ""
    stderr: str = ""
    returncode: int = 0
    error: str = ""  # Human-readable error for non-subprocess failures

    @property
    def output(self) -> str:
        """Return the most relevant output string."""
        return self.stdout if self.success else (self.error or self.stderr)

    def __str__(self) -> str:
        return self.output


class ROS2Bridge:
    """Thin wrapper around the `ros2` CLI.

    Handles environment setup, timeout management, and optional mock mode
    so the agent works even without a live ROS 2 installation.
    """

    def __init__(
        self,
        ros_domain_id: int = 0,
        ros_distro: str = "humble",
        workspace_path: str = "",
        timeout: int = 10,
        mock: bool = False,
    ) -> None:
        self.ros_domain_id = ros_domain_id
        self.ros_distro = ros_distro
        self.workspace_path = workspace_path
        self.timeout = timeout
        self.mock = mock

        if mock:
            self._available = False
        else:
            self._available = shutil.which("ros2") is not None

    # ── Public API ────────────────────────────────────────────────────────────

    @property
    def available(self) -> bool:
        """True when a `ros2` binary is reachable or mock mode is active."""
        return self._available or self.mock

    def run(self, args: list[str], timeout: Optional[int] = None) -> CommandResult:
        """Execute `ros2 <args>` and return a :class:`CommandResult`.

        Args:
            args: Arguments to pass after `ros2` (e.g. ``["topic", "list"]``).
            timeout: Per-call timeout override in seconds.
        """
        if self.mock:
            from ros2_agent.ros2.mock import MockROS2
            return MockROS2.run(args)

        if not self._available:
            return CommandResult(
                success=False,
                error=(
                    "ros2 not found in PATH.\n"
                    "Make sure ROS 2 is installed and the setup script is sourced:\n"
                    f"  source /opt/ros/{self.ros_distro}/setup.bash\n"
                    "Or run with --mock to use simulated data."
                ),
            )

        cmd = ["ros2", *args]
        env = self._build_env()
        effective_timeout = timeout if timeout is not None else self.timeout

        try:
            proc = subprocess.run(  # noqa: S603
                cmd,
                capture_output=True,
                text=True,
                timeout=effective_timeout,
                env=env,
            )
            return CommandResult(
                success=proc.returncode == 0,
                stdout=proc.stdout,
                stderr=proc.stderr,
                returncode=proc.returncode,
            )
        except subprocess.TimeoutExpired:
            return CommandResult(
                success=False,
                error=f"Command timed out after {effective_timeout}s: ros2 {' '.join(args)}",
            )
        except FileNotFoundError:
            return CommandResult(
                success=False,
                error="ros2 executable not found — ensure ROS 2 is installed and on PATH.",
            )
        except Exception as exc:  # noqa: BLE001
            return CommandResult(success=False, error=str(exc))

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _build_env(self) -> dict[str, str]:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = str(self.ros_domain_id)
        return env

    def __repr__(self) -> str:
        mode = "mock" if self.mock else ("live" if self._available else "unavailable")
        return (
            f"ROS2Bridge(domain_id={self.ros_domain_id}, "
            f"distro={self.ros_distro!r}, mode={mode!r})"
        )
