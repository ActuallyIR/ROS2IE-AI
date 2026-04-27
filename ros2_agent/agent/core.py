"""Core ROS2Agent class — ties settings, bridge, tools, and graph together."""

from __future__ import annotations

import uuid
from collections.abc import AsyncIterator, Iterator
from typing import Any, Literal

from langchain_core.language_models import BaseChatModel
from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_core.runnables import RunnableConfig

from ros2_agent.agent.graph import build_agent_graph
from ros2_agent.config.settings import Settings
from ros2_agent.ros2.bridge import ROS2Bridge
from ros2_agent.tools import get_all_tools

# ── LLM factory ───────────────────────────────────────────────────────────────


def _build_llm(settings: Settings) -> BaseChatModel:
    """Instantiate the configured LLM."""
    if settings.llm_provider == "openai":
        from langchain_openai import ChatOpenAI

        return ChatOpenAI(
            model=settings.llm_model,
            api_key=settings.openai_api_key or None,  # type: ignore[arg-type]
            temperature=settings.llm_temperature,
            streaming=True,
        )

    if settings.llm_provider == "anthropic":
        from langchain_anthropic import ChatAnthropic

        return ChatAnthropic(  # type: ignore[call-arg]
            model=settings.llm_model,
            api_key=settings.anthropic_api_key or None,  # type: ignore[arg-type]
            temperature=settings.llm_temperature,
            streaming=True,
        )

    if settings.llm_provider == "ollama":
        from langchain_ollama import ChatOllama

        return ChatOllama(
            model=settings.llm_model,
            base_url=settings.ollama_base_url,
            temperature=settings.llm_temperature,
        )

    raise ValueError(
        f"Unsupported llm_provider: {settings.llm_provider!r}. "
        "Choose from: openai, anthropic, ollama"
    )


# ── StreamEvent dataclass ─────────────────────────────────────────────────────


def _extract_text(content: str | list) -> str:
    """Extract plain text from an AIMessage content field.

    Anthropic returns a list of content blocks such as
    ``[{'type': 'text', 'text': '...'}]``, while OpenAI / Ollama return a
    plain string.  This normalises both cases.
    """
    if isinstance(content, str):
        return content
    # List of content blocks
    parts: list[str] = []
    for block in content:
        if isinstance(block, dict) and block.get("type") == "text":
            parts.append(block["text"])
        elif isinstance(block, str):
            parts.append(block)
    return "".join(parts)


class StreamEvent:
    """A single streamed event from the agent."""

    def __init__(self, kind: str, data: Any) -> None:
        self.kind = kind  # "text" | "tool_call" | "tool_result" | "done"
        self.data = data

    def __repr__(self) -> str:
        return f"StreamEvent(kind={self.kind!r}, data={str(self.data)[:80]!r})"


# ── ROS2Agent ─────────────────────────────────────────────────────────────────


class ROS2Agent:
    """High-level interface for interacting with ROS 2 via an LLM agent.

    Example::

        from ros2_agent import ROS2Agent, Settings

        agent = ROS2Agent(Settings(llm_provider="openai", mock_ros2=True))
        for event in agent.stream("What topics are active?"):
            if event.kind == "text":
                print(event.data, end="", flush=True)
    """

    def __init__(self, settings: Settings | None = None) -> None:
        self.settings = settings or Settings()
        self.bridge = ROS2Bridge(
            ros_domain_id=self.settings.ros_domain_id,
            ros_distro=self.settings.ros_distro,
            workspace_path=self.settings.workspace_path,
            timeout=self.settings.command_timeout,
            mock=self.settings.mock_ros2,
        )
        self.tools = get_all_tools(self.bridge)
        self.llm = _build_llm(self.settings)
        self.graph = build_agent_graph(
            llm=self.llm,
            tools=self.tools,
            ros_domain_id=self.settings.ros_domain_id,
            ros_distro=self.settings.ros_distro,
            mock=self.settings.mock_ros2,
        )
        self._session_id = self.settings.session_id

    # ── Synchronous interface ─────────────────────────────────────────────────

    def chat(self, message: str, session_id: str | None = None) -> str:
        """Send a message and return the complete response as a string."""
        parts: list[str] = []
        for event in self.stream(message, session_id=session_id):
            if event.kind == "text":
                parts.append(event.data)
        return "".join(parts)

    def stream(
        self,
        message: str,
        session_id: str | None = None,
    ) -> Iterator[StreamEvent]:
        """Stream events from the agent for a given *message*.

        Yields :class:`StreamEvent` objects with ``kind`` in:
        - ``"text"``        — partial or full LLM text
        - ``"tool_call"``   — dict with ``name`` and ``args``
        - ``"tool_result"`` — dict with ``name`` and ``content``
        - ``"done"``        — signals completion
        """
        tid = session_id or self._session_id
        config: RunnableConfig = {
            "configurable": {"thread_id": tid},
            "recursion_limit": self.settings.max_iterations * 2,
        }
        stream_mode: Literal["updates"] = "updates"
        input_state = {"messages": [HumanMessage(content=message)]}

        for chunk in self.graph.stream(input_state, config=config, stream_mode=stream_mode):
            for _node_name, node_output in chunk.items():
                messages = node_output.get("messages", [])
                for msg in messages:
                    if isinstance(msg, AIMessage):
                        # Emit tool calls first
                        for tc in msg.tool_calls:
                            yield StreamEvent(
                                "tool_call",
                                {"name": tc["name"], "args": tc["args"]},
                            )
                        # Emit text content
                        text = _extract_text(msg.content)
                        if text:
                            yield StreamEvent("text", text)

                    elif isinstance(msg, ToolMessage):
                        yield StreamEvent(
                            "tool_result",
                            {"name": msg.name, "content": msg.content},
                        )

        yield StreamEvent("done", None)

    # ── Async interface ───────────────────────────────────────────────────────

    async def astream(
        self,
        message: str,
        session_id: str | None = None,
    ) -> AsyncIterator[StreamEvent]:
        """Async version of :meth:`stream`."""
        tid = session_id or self._session_id
        config: RunnableConfig = {
            "configurable": {"thread_id": tid},
            "recursion_limit": self.settings.max_iterations * 2,
        }
        stream_mode: Literal["updates"] = "updates"
        input_state = {"messages": [HumanMessage(content=message)]}

        async for chunk in self.graph.astream(
            input_state,
            config=config,
            stream_mode=stream_mode,
        ):
            for _node_name, node_output in chunk.items():
                messages = node_output.get("messages", [])
                for msg in messages:
                    if isinstance(msg, AIMessage):
                        for tc in msg.tool_calls:
                            yield StreamEvent(
                                "tool_call",
                                {"name": tc["name"], "args": tc["args"]},
                            )
                        text = _extract_text(msg.content)
                        if text:
                            yield StreamEvent("text", text)
                    elif isinstance(msg, ToolMessage):
                        yield StreamEvent(
                            "tool_result",
                            {"name": msg.name, "content": msg.content},
                        )

        yield StreamEvent("done", None)

    # ── Convenience helpers ───────────────────────────────────────────────────

    def new_session(self) -> str:
        """Create a fresh session ID (clears conversation memory)."""
        self._session_id = str(uuid.uuid4())
        return self._session_id

    @property
    def ros_available(self) -> bool:
        return self.bridge.available

    def __repr__(self) -> str:
        return (
            f"ROS2Agent(provider={self.settings.llm_provider!r}, "
            f"model={self.settings.llm_model!r}, "
            f"mock={self.settings.mock_ros2})"
        )
