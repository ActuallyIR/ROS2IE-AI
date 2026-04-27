"""Shared test fixtures."""

from __future__ import annotations

import pytest

from ros2_agent.config.settings import Settings
from ros2_agent.ros2.bridge import ROS2Bridge


@pytest.fixture
def mock_bridge() -> ROS2Bridge:
    """A ROS2Bridge in mock mode — no real ROS 2 needed."""
    return ROS2Bridge(mock=True)


@pytest.fixture
def mock_settings() -> Settings:
    """Settings configured for mock mode with a fake API key."""
    return Settings(
        mock_ros2=True,
        llm_provider="openai",
        llm_model="gpt-4o",
        openai_api_key="test-key-not-real",
    )
