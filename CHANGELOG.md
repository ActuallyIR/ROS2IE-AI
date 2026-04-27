# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.1.0] - 2024-04-27

### Added
- Initial release of ROS2 Agent
- LangGraph ReAct agent with persistent session memory
- **Topic tools**: list, echo, publish, hz, bandwidth
- **Service tools**: list, type, call
- **Action tools**: list, info, send_goal (navigation, spin, backup)
- **Node tools**: list, info
- **Log tools**: get_ros_logs, get_error_logs, explain_errors
- **Parameter tools**: list, get, set, dump
- **Diagnostic tools**: ros2 doctor, robot health check, TF frames, get_transform
- **Launch tools**: list packages/executables, launch_package, run_node, record_bag
- Multi-provider LLM support: OpenAI, Anthropic, Ollama
- Rich terminal UI with streaming responses and tool-call display
- Web UI with FastAPI + Server-Sent Events for real-time streaming
- Topic/node/service browser panel in Web UI
- Mock ROS 2 mode simulating TurtleBot4 + Nav2 stack
- One-command install: `pip install ros2-agent`
- `ros2-agent chat` — interactive REPL
- `ros2-agent run "<query>"` — non-interactive single query
- `ros2-agent web` — web server
- `ros2-agent info` — environment info
- GitHub Actions CI (lint, test, typecheck) for Python 3.10–3.12
- PyPI release automation workflow
