"""Entry point for the ros2-agent CLI."""

from ros2_agent.cli.app import app


def main() -> None:
    app()


if __name__ == "__main__":
    main()
