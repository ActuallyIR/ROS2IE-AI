# Contributing to ROS2 Agent

Thank you for your interest in contributing! 🤖

## Development Setup

```bash
git clone https://github.com/ros2-agent/ros2-agent
cd ros2-agent
pip install -e ".[dev]"
pre-commit install
```

## Running Tests

```bash
# All tests (no ROS 2 needed — uses mock mode)
pytest

# With coverage
pytest --cov=ros2_agent --cov-report=html

# Specific test file
pytest tests/test_tools.py -v
```

## Code Style

This project uses **ruff** for linting and formatting:

```bash
ruff check ros2_agent tests     # lint
ruff format ros2_agent tests    # format
```

## Adding a New ROS 2 Tool

1. Create or edit a file in `ros2_agent/tools/`.
2. Write a factory function `create_<category>_tools(bridge: ROS2Bridge) -> list`.
3. Use the `@tool` decorator with a clear docstring (this is what the LLM sees!).
4. Register it in `ros2_agent/tools/__init__.py → get_all_tools()`.
5. Add mock data in `ros2_agent/ros2/mock.py` if needed.
6. Write tests in `tests/test_tools.py`.

**Good tool docstring example:**
```python
@tool
def list_topics(filter_str: str = "") -> str:
    """List all active ROS 2 topics on the robot or simulation.

    Args:
        filter_str: Optional substring to filter results (e.g. 'camera', 'nav').
    """
```

## Pull Request Guidelines

- Keep PRs focused — one feature or fix per PR.
- Write tests for new tools/features.
- Update `CHANGELOG.md` under `[Unreleased]`.
- Ensure `pytest` and `ruff check` pass before opening a PR.

## Commit Message Format

```
type(scope): short description

Examples:
  feat(tools): add /diagnostics topic reader
  fix(bridge): handle timeout on ros2 topic echo
  docs(readme): add Jazzy installation instructions
  test(tools): add tests for action tools
```

## Reporting Bugs

Use the [Bug Report](.github/ISSUE_TEMPLATE/bug_report.yml) template.

## License

By contributing, you agree that your contributions will be licensed under the MIT License.
