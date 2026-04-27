"""Example: Basic navigation using ROS2 Agent."""

from __future__ import annotations

import os
from ros2_agent import ROS2Agent, Settings

# ── Configure ─────────────────────────────────────────────────────────────────
# You can also set these via environment variables or a .env file.
settings = Settings(
    llm_provider="openai",
    llm_model="gpt-4o",
    openai_api_key=os.environ.get("OPENAI_API_KEY", ""),
    mock_ros2=True,  # Set to False when using a real robot
    ros_domain_id=0,
    ros_distro="humble",
)

agent = ROS2Agent(settings)

# ── Example 1: Check robot status ─────────────────────────────────────────────
print("=" * 60)
print("Example 1: Robot health check")
print("=" * 60)
response = agent.chat("Run a full health check on the robot")
print(response)

# ── Example 2: Read battery level ─────────────────────────────────────────────
print("\n" + "=" * 60)
print("Example 2: Battery level")
print("=" * 60)
response = agent.chat("What is the current battery level?")
print(response)

# ── Example 3: List active topics ─────────────────────────────────────────────
print("\n" + "=" * 60)
print("Example 3: Active topics")
print("=" * 60)
response = agent.chat("List all navigation-related topics")
print(response)

# ── Example 4: Navigation goal ────────────────────────────────────────────────
print("\n" + "=" * 60)
print("Example 4: Send navigation goal")
print("=" * 60)
response = agent.chat(
    "Send the robot to coordinates (2.5, 3.0) using the navigate_to_pose action"
)
print(response)

# ── Example 5: Streaming (token by token) ─────────────────────────────────────
print("\n" + "=" * 60)
print("Example 5: Streaming response")
print("=" * 60)
print("Agent: ", end="", flush=True)
for event in agent.stream("What are the current odometry readings?"):
    if event.kind == "tool_call":
        print(f"\n[calling {event.data['name']}...]", end="", flush=True)
    elif event.kind == "text":
        print(event.data, end="", flush=True)
    elif event.kind == "done":
        print()  # final newline

# ── Example 6: Session memory (multi-turn) ────────────────────────────────────
print("\n" + "=" * 60)
print("Example 6: Multi-turn conversation (session memory)")
print("=" * 60)

agent.chat("My robot is called 'Spot' and is a quadruped.")
response = agent.chat("What is the name of my robot and what type is it?")
print(response)
