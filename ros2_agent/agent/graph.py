"""LangGraph graph definition for the ROS2 Agent."""

from __future__ import annotations

from typing import TYPE_CHECKING

from langchain_core.messages import SystemMessage
from langgraph.checkpoint.memory import MemorySaver
from langgraph.graph import MessagesState, StateGraph
from langgraph.prebuilt import ToolNode, tools_condition

if TYPE_CHECKING:
    from langchain_core.language_models import BaseChatModel
    from langgraph.graph.state import CompiledStateGraph

SYSTEM_PROMPT = """\
You are ROS2Agent — an expert AI assistant for controlling, monitoring, and debugging ROS 2 robots.

You have access to a comprehensive set of ROS 2 tools:
• Topic tools   — list, echo, publish, measure frequency/bandwidth
• Service tools — list, type-check, call
• Action tools  — list, get info, send goals (navigation, spin, backup, etc.)
• Node tools    — list, inspect info
• Log tools     — read /rosout, filter errors, explain crashes
• Param tools   — list, get, set, dump node parameters
• Diagnostic tools — full health check, TF tree, ros2 doctor
• Launch tools  — list packages/executables, launch files, record bags

Behaviour guidelines:
1. UNDERSTAND first — read the user's intent before reaching for a tool.
2. USE THE MINIMUM tools needed. Do not over-call.
3. EXPLAIN what you are doing in plain English before each tool call.
4. SAFETY FIRST — before publishing to motion topics (/cmd_vel, etc.) or sending
   navigation goals, explicitly warn the user and confirm this is intentional.
5. DIAGNOSE proactively — when an error is reported, gather logs and parameters
   before suggesting a fix.
6. BE CONCISE — summarise tool output; do not dump raw YAML unless asked.

Current configuration:
  ROS_DOMAIN_ID : {ros_domain_id}
  ROS 2 distro  : {ros_distro}
  Mode          : {mode}
"""


def build_agent_graph(
    llm: BaseChatModel,
    tools: list,
    ros_domain_id: int = 0,
    ros_distro: str = "humble",
    mock: bool = False,
) -> CompiledStateGraph:
    """Compile and return the LangGraph ReAct agent graph.

    The graph follows the standard pattern:
        user → agent node → (tool node → agent node)* → END
    """
    memory = MemorySaver()

    system_content = SYSTEM_PROMPT.format(
        ros_domain_id=ros_domain_id,
        ros_distro=ros_distro,
        mode="🟡 MOCK (simulated data)" if mock else "🟢 LIVE",
    )

    llm_with_tools = llm.bind_tools(tools)
    tool_node = ToolNode(tools)

    def agent_node(state: MessagesState) -> dict:
        messages = list(state["messages"])

        # Prepend system message if not already present
        if not messages or not isinstance(messages[0], SystemMessage):
            messages = [SystemMessage(content=system_content), *messages]

        response = llm_with_tools.invoke(messages)
        return {"messages": [response]}

    workflow = StateGraph(MessagesState)
    workflow.add_node("agent", agent_node)
    workflow.add_node("tools", tool_node)

    workflow.set_entry_point("agent")
    workflow.add_conditional_edges("agent", tools_condition)
    workflow.add_edge("tools", "agent")

    return workflow.compile(checkpointer=memory)
