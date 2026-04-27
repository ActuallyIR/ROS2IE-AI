# Security Policy

## Supported Versions

| Version | Supported |
|---------|-----------|
| 0.1.x   | ✅        |

## Reporting a Vulnerability

**Please do not open a public GitHub issue for security vulnerabilities.**

To report a security issue, email the maintainers at **security@ros2-agent.dev** (or open a
[GitHub private security advisory](https://github.com/ros2-agent/ros2-agent/security/advisories/new)).

Include:
- A description of the vulnerability and its potential impact
- Steps to reproduce or a proof-of-concept
- Any suggested mitigations you have in mind

You should receive an acknowledgement within **48 hours** and a substantive response within **7 days**.

## Security Considerations

### API Keys
- API keys are read from environment variables or a `.env` file — **never commit `.env` to version control**.
- `.env` is listed in `.gitignore` by default.
- The web UI does not expose API keys to the browser.

### Mock Mode
- `--mock` mode does not connect to any real robot or external network service.
- The simulation runs entirely in-process.

### Web UI
- The web server (`ros2-agent web`) binds to `0.0.0.0` by default — restrict to `127.0.0.1` in
  production environments by setting `ROS2_AGENT_WEB_HOST=127.0.0.1`.
- There is no authentication on the web UI; do not expose it to untrusted networks without adding
  a reverse proxy with auth.

### ROS 2 Commands
- The agent executes `ros2` CLI subprocesses. Only run it on a trusted machine connected to a
  trusted ROS 2 network (DDS domain).
- Avoid passing untrusted user input directly to the agent in automated pipelines without input
  validation.
