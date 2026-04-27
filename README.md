<div align="center">

```
██████╗  ██████╗ ███████╗██╗███████╗
██╔══██╗██╔═══██╗██╔════╝██║██╔════╝
██████╔╝██║   ██║███████╗██║█████╗
██╔══██╗██║   ██║╚════██║██║██╔══╝
██║  ██║╚██████╔╝███████║██║███████╗
╚═╝  ╚═╝ ╚═════╝ ╚══════╝╚═╝╚══════╝
```

### **ROSIE** — *Robot Operations via Structured Intelligent English*
#### Your robot understands you now.

[![CI](https://github.com/ros2-agent/ros2-agent/actions/workflows/ci.yml/badge.svg)](https://github.com/ros2-agent/ros2-agent/actions/workflows/ci.yml)
[![PyPI version](https://badge.fury.io/py/ros2-agent.svg)](https://badge.fury.io/py/ros2-agent)
[![Python 3.10+](https://img.shields.io/badge/python-3.10%2B-blue.svg)](https://www.python.org/downloads/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS_2-Humble%20%7C%20Iron%20%7C%20Jazzy-brightgreen)](https://docs.ros.org/en/humble/)
[![LangGraph](https://img.shields.io/badge/powered%20by-LangGraph-orange)](https://github.com/langchain-ai/langgraph)
[![Stars](https://img.shields.io/github/stars/ros2-agent/ros2-agent?style=social)](https://github.com/ros2-agent/ros2-agent/stargazers)

<br/>

> **"I spent years learning ROS 2 CLI commands.**
> **Then I spent 10 minutes building this."**

<br/>

<!-- ================================================================
  HOW TO ADD YOUR OWN VIDEO — pick one method:

  METHOD 1 · GitHub-hosted video (renders inline, native player)
  ──────────────────────────────────────────────────────────────────
  1. Open any GitHub Issue in this repo
  2. Drag-and-drop your .mp4 / .mov / .webm into the comment box
  3. GitHub uploads it and gives you a CDN URL like:
       https://github.com/user-attachments/assets/xxxxxxxx.mp4
  4. Replace the <video src="..."> below with that URL

  METHOD 2 · YouTube thumbnail → link (most common in popular repos)
  ──────────────────────────────────────────────────────────────────
  Replace the <a href="..."><img src="..."/></a> block below with:
  [![Demo](https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg)](https://youtu.be/YOUR_VIDEO_ID)

  METHOD 3 · Animated GIF (no click needed, auto-plays everywhere)
  ──────────────────────────────────────────────────────────────────
  Record with: https://www.screentogif.com  or  ffmpeg -i demo.mp4 demo.gif
  Then: ![ROSIE demo](docs/demo.gif)
================================================================ -->

<!-- Native GitHub video — drag your .mp4 onto a GitHub Issue, paste the URL here -->
<!--
<video src="https://github.com/user-attachments/assets/REPLACE_WITH_YOUR_UPLOAD.mp4"
  autoplay loop muted playsinline width="100%"
  title="ROSIE — chatting with a ROS 2 robot in plain English">
</video>
-->

<!-- YouTube clickable thumbnail (replace VIDEO_ID) -->
<!--
<a href="https://youtu.be/YOUR_VIDEO_ID">
  <img src="https://img.youtube.com/vi/YOUR_VIDEO_ID/maxresdefault.jpg"
       alt="ROSIE demo — click to watch" width="100%" />
</a>
<sub>▶ Click to watch the 90-second demo</sub>
-->

</div>

---

## The problem nobody talks about

You've spent weeks learning `ros2 topic echo`, `ros2 param set`, `ros2 launch`, `ros2 bag record`, TF trees, Nav2 lifecycles, `ros2 doctor`... and you *still* have to Google the exact syntax every single time.

Meanwhile your robot just sits there.

**ROSIE fixes this.** Type what you want in plain English. She figures out the commands, runs them, and explains what she found — all in one shot.

```
You  ›  My robot stopped navigating. What's wrong?

ROSIE  ⚙  get_error_logs(count=20)
       ⚙  check_robot_health()
       ⚙  get_param(node="/bt_navigator", param="default_bt_xml_filename")

       Found the issue. Your behaviour tree file doesn't exist at the path
       configured in bt_navigator: "/home/user/my_bt.xml"

       The node fell back to the default tree but /map is not yet published,
       so NavigateToPose goals are being rejected immediately.

       Fix: ensure your SLAM or map_server node is running before sending goals.
```

No Stack Overflow. No man pages. Just answers.

---

## ⚡ 30-Second Quick Start

```bash
pip install ros2-agent
cp .env.example .env        # add your API key
ros2-agent chat --mock      # no robot required
```

That's it. You're talking to a robot.

```
You  ›  What topics are active?

ROSIE  ⚙  list_topics()

       Found 21 active topics:
       /battery_state · /camera/color/image_raw · /cmd_vel
       /map · /odom · /scan · /tf · /tf_static …
```

### With a real robot

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2-agent chat
```

### Dark-mode Web UI + Live Simulator

```bash
ros2-agent web --mock
# → http://localhost:8080
```

A full physics simulator streams to your browser in real time.
90-ray LiDAR, differential-drive dynamics, acceleration limits, Gaussian sensor noise —
no Gazebo, no ROS 2 install needed.

<!-- Web UI screenshot or video — replace src with your GitHub-uploaded asset -->
<!--
<video src="https://github.com/user-attachments/assets/REPLACE_WITH_WEBUI_UPLOAD.mp4"
  autoplay loop muted playsinline width="100%"
  title="ROSIE Web UI — live robot simulator">
</video>
-->
<!-- Or a static screenshot: ![Web UI](docs/screenshot-webui.png) -->

---

## 🎮 What ROSIE Can Do

> She has **22 ROS 2 tools** and knows when to chain them.

| Say this... | ROSIE does this |
|---|---|
| `"What's the battery level?"` | Reads `/battery_state`, extracts percentage |
| `"Why is my nav stack crashing?"` | Chains log reader + health check + param dump |
| `"Navigate to (2.5, 3.0)"` | Sends `NavigateToPose` goal via Nav2 action |
| `"Start SLAM"` | Launches `slam_toolbox` with correct args |
| `"Record /scan and /odom for 30 seconds"` | Runs `ros2 bag record` with your spec |
| `"Set controller frequency to 25 Hz"` | Calls `ros2 param set /controller_server …` |
| `"Show the TF tree"` | Lists frames + gets map→base_link transform |
| `"Run a full health check"` | Nodes + battery + diagnostics in one report |
| `"List all packages with 'nav' in the name"` | Filters `ros2 pkg list` output |
| `"What's the CPU usage of the navigation node?"` | Reads `/diagnostics` and explains it |

<details>
<summary><b>📡 Topic Tools (6)</b> — list, echo, info, publish, hz, bandwidth</summary>

| Tool | Description |
|---|---|
| `list_topics` | List all active topics with optional filter |
| `echo_topic` | Read N messages from a topic |
| `get_topic_info` | Type, publishers, and subscribers |
| `publish_to_topic` | Publish a message in YAML format |
| `get_topic_hz` | Measure publishing frequency |
| `get_topic_bandwidth` | Measure bandwidth usage |

</details>

<details>
<summary><b>🔧 Service & Action Tools (6)</b> — call services, send Nav2 goals</summary>

| Tool | Description |
|---|---|
| `list_services` | List all services |
| `call_service` | Call any service with request data |
| `list_actions` | List all action servers |
| `send_action_goal` | Send a goal (navigation, spin, dock…) |
| `get_service_type` | Get service message type |
| `get_action_info` | Inspect action type and clients |

</details>

<details>
<summary><b>⚙️ Parameter Tools (4)</b> — get, set, list, dump YAML</summary>

| Tool | Description |
|---|---|
| `list_params` | List all node parameters |
| `get_param` | Get a specific parameter value |
| `set_param` | Set a parameter at runtime |
| `dump_params` | Full YAML dump of all parameters |

</details>

<details>
<summary><b>🏥 Diagnostic & TF Tools (5)</b> — health check, transforms</summary>

| Tool | Description |
|---|---|
| `run_ros_doctor` | Full `ros2 doctor` report |
| `check_robot_health` | Nodes + battery + diagnostics in one shot |
| `list_tf_frames` | All frames in the TF tree |
| `get_transform` | Transform between any two frames |
| `get_diagnostics` | Parse `/diagnostics` topic |

</details>

<details>
<summary><b>📋 Log Tools (3)</b> — rosout, error filter, AI diagnosis</summary>

| Tool | Description |
|---|---|
| `get_ros_logs` | Recent `/rosout` messages |
| `get_error_logs` | Filter ERROR and FATAL messages only |
| `explain_errors` | Collect and AI-explain all warnings |

</details>

<details>
<summary><b>🚀 Launch & Node Tools (5)</b> — run packages, record bags</summary>

| Tool | Description |
|---|---|
| `list_packages` | All installed ROS 2 packages |
| `list_package_executables` | Executables in a specific package |
| `launch_package` | Launch a `.launch.py` file |
| `run_node` | Run a specific node |
| `record_bag` | Record topics to a bag file |

</details>

---

## 🧠 LLM Providers

No vendor lock-in. Swap the brain without touching the robot.

| Provider | Models | Cost | Notes |
|---|---|---|---|
| **Anthropic** | `claude-haiku-4-5-20251001` ✦ recommended | ~$0.001/query | Fast, cheap, great at tool use |
| **Anthropic** | `claude-sonnet-4-5-20251001` | ~$0.005/query | Better reasoning for complex queries |
| **OpenAI** | `gpt-4o`, `gpt-4o-mini` | $$ | Great alternative |
| **Ollama** | `llama3.2`, `mistral`, `codellama` | **Free** | Fully local, no data leaves your machine |

```bash
# Free, 100% local — no API key
ollama pull llama3.2
ros2-agent chat --provider ollama --model llama3.2 --mock
```

Configure via `.env` or env vars (prefix `ROS2_AGENT_`):

```dotenv
ROS2_AGENT_LLM_PROVIDER=anthropic
ROS2_AGENT_LLM_MODEL=claude-haiku-4-5-20251001
ROS2_AGENT_ANTHROPIC_API_KEY=sk-ant-...
```

See [`.env.example`](.env.example) for the full reference.

---

## 🏗 Architecture

ROSIE is a **LangGraph ReAct agent** wrapped around a subprocess-based ROS 2 bridge.
Works anywhere `ros2` is on your PATH — no Python binding hell.

```
  You (CLI or Web)
        │
        ▼
  ┌─────────────────────────────────────────┐
  │          ROSIE — LangGraph Agent        │
  │                                         │
  │  ┌──────────────┐    ┌───────────────┐  │
  │  │  LLM         │◄──►│  22 ROS Tools │  │
  │  │  (any model) │    │               │  │
  │  └──────────────┘    └──────┬────────┘  │
  │       MemorySaver (history) │           │
  └─────────────────────────────┼───────────┘
                                │
                    ┌───────────▼──────────┐
                    │     ROS 2 Bridge      │
                    │  subprocess / mock    │
                    └───────────┬──────────┘
                                │
                    ┌───────────▼──────────┐
                    │  Your Robot / Gazebo  │
                    │  or built-in Mock Sim │
                    └──────────────────────┘
```

**Design choices that matter:**
- **No ROS 2 Python bindings required** — pure subprocess; install anywhere with `pip`
- **Mock mode is first-class** — full TurtleBot4 + Nav2 simulation, zero setup
- **Streaming by default** — tokens flow to CLI and Web UI in real time via SSE
- **Conversation memory** — ask follow-up questions, ROSIE remembers the context

---

## 🐳 Docker

```bash
docker build -t rosie .

# Chat
docker run -it -e ROS2_AGENT_ANTHROPIC_API_KEY=sk-ant-... rosie chat --mock

# Web UI on port 8080
docker run -p 8080:8080 -e ROS2_AGENT_ANTHROPIC_API_KEY=sk-ant-... rosie web --mock
```

---

## 🧪 Development

```bash
git clone https://github.com/ros2-agent/ros2-agent
cd ros2-agent
pip install -e ".[dev]"

pytest -v                     # all tests run in mock mode, no ROS 2 needed
ruff check ros2_agent tests   # lint
```

---

## 🗺 Roadmap

- [x] **v0.1** — 22 tools, CLI, dark Web UI, built-in physics simulator, mock mode
- [ ] **v0.2** — RViz2 bridge (stream point clouds and costmaps to the Web UI)
- [ ] **v0.2** — Gazebo and Isaac Sim integration
- [ ] **v0.3** — Bag file intelligence (`"explain what happened in this recording"`)
- [ ] **v0.3** — Plugin API for custom tools
- [ ] **v0.4** — Multi-robot fleet support
- [ ] **v0.5** — Workspace code analysis (`"why does this package fail to build?"`)
- [ ] **v1.0** — Native desktop app

Have an idea? [Open a discussion](https://github.com/ros2-agent/ros2-agent/discussions) — contributions are very welcome.

---

## 🤝 Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md). All tests run without a real robot — just `--mock`.

## 📄 License

MIT — see [LICENSE](LICENSE).

---

<div align="center">

**If ROSIE saves you even one Stack Overflow trip, drop a ⭐ — it helps more people find the project.**

</div>
