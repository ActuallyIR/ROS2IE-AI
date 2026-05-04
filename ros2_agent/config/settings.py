"""Settings and configuration for ROS2 Agent."""

from __future__ import annotations

from typing import Literal

from pydantic import Field, field_validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """ROS2 Agent configuration.

    All values can be set via environment variables (prefix: ``ROS2_AGENT_``) or a ``.env`` file.
    """

    model_config = SettingsConfigDict(
        env_prefix="ROS2_AGENT_",
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
    )

    # ── LLM Settings ──────────────────────────────────────────────────────────
    llm_provider: Literal["openai", "anthropic", "ollama"] = Field(
        default="openai",
        description="LLM provider to use",
    )
    llm_model: str = Field(
        default="gpt-4o",
        description="Model name (depends on provider)",
    )
    openai_api_key: str = Field(default="", description="OpenAI API key")
    anthropic_api_key: str = Field(default="", description="Anthropic API key")
    ollama_base_url: str = Field(
        default="http://localhost:11434",
        description="Ollama server base URL",
    )
    llm_temperature: float = Field(default=0.0, ge=0.0, le=2.0)

    # ── ROS 2 Settings ────────────────────────────────────────────────────────
    ros_domain_id: int = Field(default=0, ge=0, le=232, description="ROS_DOMAIN_ID")
    ros_distro: str = Field(default="humble", description="ROS 2 distribution name")
    workspace_path: str = Field(
        default="",
        description="Path to ROS 2 workspace (sourced before commands)",
    )
    command_timeout: int = Field(
        default=10,
        description="Default timeout (seconds) for ROS 2 commands",
    )

    # ── Agent Settings ────────────────────────────────────────────────────────
    max_iterations: int = Field(default=15, description="Max tool-call iterations per query")
    verbose: bool = Field(default=False, description="Enable verbose tool-call logging")
    mock_ros2: bool = Field(
        default=False,
        description="Use simulated ROS 2 data (no real ROS 2 required)",
    )

    # ── Web UI Settings ───────────────────────────────────────────────────────
    web_host: str = Field(default="0.0.0.0", description="Web UI bind host")
    web_port: int = Field(default=8080, ge=1, le=65535, description="Web UI port")

    # ── Session Settings ──────────────────────────────────────────────────────
    session_id: str = Field(default="default", description="Session ID for memory isolation")

    @field_validator("llm_model", mode="before")
    @classmethod
    def set_default_model(cls, v: str, info: object) -> str:  # noqa: ARG003
        """Fall back to ``gpt-4o`` when the model string is empty."""
        return v or "gpt-4o"

    def get_provider_display(self) -> str:
        """Return a short ``provider/model`` display string."""
        return f"{self.llm_provider}/{self.llm_model}"
