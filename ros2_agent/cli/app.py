"""Rich-powered CLI for ROS2 Agent."""

from __future__ import annotations

import typer
from rich.console import Console
from rich.markdown import Markdown
from rich.panel import Panel
from rich.prompt import Prompt
from rich.rule import Rule
from rich.table import Table

from ros2_agent import __version__
from ros2_agent.config.settings import Settings

app = typer.Typer(
    name="ros2-agent",
    help="🤖 LLM-powered agent for ROS 2 robots — control, monitor, and debug in plain English.",
    add_completion=False,
    rich_markup_mode="rich",
)

console = Console()
err_console = Console(stderr=True, style="bold red")


# ── Helpers ───────────────────────────────────────────────────────────────────


def _print_banner(settings: Settings) -> None:
    mode_text = "[yellow]MOCK[/yellow]" if settings.mock_ros2 else "[green]LIVE[/green]"
    ros_status = "[green]✓[/green]" if not settings.mock_ros2 else "[yellow]~[/yellow]"

    banner = (
        f"  [bold cyan]ROS2 Agent[/bold cyan] [dim]v{__version__}[/dim]\n"
        f"  LLM  : [bold]{settings.llm_provider}[/bold] / [bold]{settings.llm_model}[/bold]\n"
        f"  ROS2 : {ros_status} domain_id=[bold]{settings.ros_domain_id}[/bold]  "
        f"distro=[bold]{settings.ros_distro}[/bold]  mode={mode_text}\n"
        f"  [dim]Type your question below. 'exit' or Ctrl-C to quit.[/dim]"
    )
    console.print(Panel(banner, border_style="cyan", padding=(0, 1)))


def _render_tool_call(name: str, args: dict) -> str:
    args_str = ", ".join(f"{k}={v!r}" for k, v in args.items()) if args else ""
    return f"[dim cyan]⚙  {name}({args_str})[/dim cyan]"


def _render_tool_result(name: str, content: str) -> str:
    # Truncate very long results in the display
    preview = content[:300] + "…" if len(content) > 300 else content
    return f"[dim green]   → {preview}[/dim green]"


# ── Commands ──────────────────────────────────────────────────────────────────


@app.command()
def chat(
    provider: str | None = typer.Option(
        None, "--provider", "-p", help="LLM provider: openai | anthropic | ollama"
    ),
    model: str | None = typer.Option(
        None, "--model", "-m", help="Model name (e.g. gpt-4o, claude-haiku-4-5-20251001)"
    ),
    ros_domain_id: int | None = typer.Option(
        None, "--domain-id", "-d", help="ROS_DOMAIN_ID (default: 0)"
    ),
    ros_distro: str | None = typer.Option(
        None, "--distro", help="ROS 2 distro (humble/iron/jazzy)"
    ),
    mock: bool = typer.Option(
        False, "--mock", help="Use simulated ROS 2 data (no real robot needed)"
    ),
    verbose: bool = typer.Option(False, "--verbose", "-v", help="Show raw tool outputs"),
    session_id: str | None = typer.Option(
        None, "--session", "-s", help="Session ID for persistent memory"
    ),
) -> None:
    """[bold cyan]Start an interactive ROS 2 Agent chat session.[/bold cyan]

    \b
    Examples:
      ros2-agent chat
      ros2-agent chat --mock
      ros2-agent chat --provider anthropic --model claude-haiku-4-5-20251001
      ros2-agent chat --provider ollama --model llama3.2 --mock
    """
    # Build settings (CLI flags override env / .env file)
    overrides: dict = {}
    if provider:
        overrides["llm_provider"] = provider
    if model:
        overrides["llm_model"] = model
    if ros_domain_id is not None:
        overrides["ros_domain_id"] = ros_domain_id
    if ros_distro:
        overrides["ros_distro"] = ros_distro
    if mock:
        overrides["mock_ros2"] = True
    if verbose:
        overrides["verbose"] = True
    if session_id:
        overrides["session_id"] = session_id

    try:
        settings = Settings(**overrides)
    except Exception as exc:
        err_console.print(f"Configuration error: {exc}")
        raise typer.Exit(1) from exc

    # Lazy-import to avoid slow startup
    from ros2_agent.agent.core import ROS2Agent

    try:
        agent = ROS2Agent(settings)
    except Exception as exc:
        err_console.print(f"Failed to initialise agent: {exc}")
        raise typer.Exit(1) from exc

    _print_banner(settings)

    # ── REPL ──────────────────────────────────────────────────────────────────
    while True:
        try:
            user_input = Prompt.ask("\n[bold green]You[/bold green]").strip()
        except (KeyboardInterrupt, EOFError):
            console.print("\n[dim]Goodbye! 👋[/dim]")
            break

        if not user_input:
            continue
        if user_input.lower() in {"exit", "quit", "q", ":q"}:
            console.print("[dim]Goodbye! 👋[/dim]")
            break

        console.print(Rule(style="dim"))
        console.print("[bold cyan]Agent[/bold cyan]", end=" ")

        response_parts: list[str] = []
        try:
            for event in agent.stream(user_input):
                if event.kind == "tool_call":
                    console.print()
                    console.print(_render_tool_call(event.data["name"], event.data["args"]))
                    if verbose:
                        console.print(f"  [dim]args: {event.data['args']}[/dim]")

                elif event.kind == "tool_result":
                    if verbose:
                        console.print(
                            _render_tool_result(event.data["name"], event.data["content"])
                        )

                elif event.kind == "text":
                    # Stream text character by character feel
                    console.print(event.data, end="", highlight=False)
                    response_parts.append(event.data)

                elif event.kind == "done":
                    console.print()  # final newline

        except KeyboardInterrupt:
            console.print("\n[yellow]Interrupted.[/yellow]")
        except Exception as exc:  # noqa: BLE001
            console.print(f"\n[red]Error: {exc}[/red]")
            if verbose:
                import traceback

                traceback.print_exc()


@app.command()
def run(
    query: str = typer.Argument(..., help="Single query to run non-interactively"),
    provider: str | None = typer.Option(None, "--provider", "-p"),
    model: str | None = typer.Option(None, "--model", "-m"),
    mock: bool = typer.Option(False, "--mock"),
    json_output: bool = typer.Option(False, "--json", help="Output raw JSON"),
) -> None:
    """Run a [bold]single query[/bold] and print the result (non-interactive).

    \b
    Example:
      ros2-agent run "What topics are active?" --mock
      ros2-agent run "List all nodes" | tee nodes.txt
    """
    overrides: dict = {}
    if provider:
        overrides["llm_provider"] = provider
    if model:
        overrides["llm_model"] = model
    if mock:
        overrides["mock_ros2"] = True

    try:
        settings = Settings(**overrides)
    except Exception as exc:
        err_console.print(f"Configuration error: {exc}")
        raise typer.Exit(1) from exc

    from ros2_agent.agent.core import ROS2Agent

    agent = ROS2Agent(settings)

    if not json_output:
        with console.status("[cyan]Thinking…[/cyan]", spinner="dots"):
            response = agent.chat(query)
        console.print(Markdown(response))
    else:
        import json

        response = agent.chat(query)
        print(json.dumps({"query": query, "response": response}))


@app.command()
def info() -> None:
    """Show current configuration and ROS 2 environment status."""
    settings = Settings()

    table = Table(title="ROS2 Agent Configuration", border_style="cyan", show_header=False)
    table.add_column("Key", style="bold")
    table.add_column("Value")

    table.add_row("Version", f"v{__version__}")
    table.add_row("LLM Provider", settings.llm_provider)
    table.add_row("LLM Model", settings.llm_model)
    table.add_row("ROS_DOMAIN_ID", str(settings.ros_domain_id))
    table.add_row("ROS 2 Distro", settings.ros_distro)
    table.add_row("Mock Mode", str(settings.mock_ros2))
    table.add_row("Max Iterations", str(settings.max_iterations))
    table.add_row("Web Host:Port", f"{settings.web_host}:{settings.web_port}")

    import shutil

    ros2_on_path = shutil.which("ros2") is not None
    table.add_row(
        "ros2 on PATH",
        "[green]✓ Yes[/green]" if ros2_on_path else "[red]✗ No[/red]",
    )

    console.print(table)


@app.command()
def web(
    host: str = typer.Option("0.0.0.0", "--host", help="Bind host"),
    port: int = typer.Option(8080, "--port", help="Port"),
    mock: bool = typer.Option(False, "--mock"),
    reload: bool = typer.Option(False, "--reload", help="Auto-reload on code changes (dev)"),
) -> None:
    """Start the [bold cyan]web UI[/bold cyan] server.

    \b
    Then open http://localhost:8080 in your browser.
    """
    import uvicorn

    from ros2_agent.web.app import create_app

    overrides: dict = {}
    if mock:
        overrides["mock_ros2"] = True
    settings = Settings(**overrides)

    console.print(
        Panel(
            f"Starting web UI at [bold cyan]http://{host}:{port}[/bold cyan]\n"
            f"LLM: {settings.get_provider_display()}  |  Mock: {mock}",
            border_style="cyan",
        )
    )
    fast_app = create_app(settings)
    uvicorn.run(fast_app, host=host, port=port, reload=reload)


# ── Version callback ──────────────────────────────────────────────────────────


def _version_callback(value: bool) -> None:
    if value:
        typer.echo(f"ros2-agent v{__version__}")
        raise typer.Exit()


@app.callback()
def main_callback(
    version: bool | None = typer.Option(
        None, "--version", callback=_version_callback, is_eager=True, help="Print version and exit"
    ),
) -> None:
    pass
