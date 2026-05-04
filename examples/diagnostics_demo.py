"""Example: Diagnostics and debugging with ROS2 Agent."""

from __future__ import annotations

import os

from ros2_agent import ROS2Agent, Settings

settings = Settings(
    llm_provider="openai",
    openai_api_key=os.environ.get("OPENAI_API_KEY", ""),
    mock_ros2=True,
)

agent = ROS2Agent(settings)

# ── 1. Get recent error logs ───────────────────────────────────────────────────
print("Querying recent error logs…")
response = agent.chat("Show me the last 10 error messages from the ROS 2 logs")
print(response)

# ── 2. Explain a crash ────────────────────────────────────────────────────────
print("\n" + "-" * 40)
print("Diagnosing a hypothetical crash…")
response = agent.chat(
    "My navigation stack crashed. Show me recent WARN/ERROR logs and explain "
    "what might have caused the issue."
)
print(response)

# ── 3. Parameter tuning suggestion ────────────────────────────────────────────
print("\n" + "-" * 40)
print("Controller parameter inspection…")
response = agent.chat(
    "Show me the current parameters for /controller_server and suggest "
    "if the controller_frequency could be improved."
)
print(response)

# ── 4. TF tree check ──────────────────────────────────────────────────────────
print("\n" + "-" * 40)
print("Checking transforms…")
response = agent.chat("What is the current transform between 'map' and 'base_link'?")
print(response)
