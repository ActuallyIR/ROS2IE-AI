"""Entry point for the ros2-agent CLI."""

from ros2_agent.cli.app import app


def main() -> None:
    """Launch the ros2-agent CLI application."""
    app()


if __name__ == "__main__":
    main()
