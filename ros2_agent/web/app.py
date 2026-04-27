"""FastAPI web server for ROS2 Agent.

Provides:
  GET  /              → Web UI (HTML)
  POST /chat          → Single-turn chat (JSON)
  GET  /chat/stream   → SSE streaming chat
  GET  /ros/topics    → List topics
  GET  /ros/nodes     → List nodes
  GET  /ros/status    → Robot health summary
  GET  /health        → API health check
"""

from __future__ import annotations

import asyncio
import json
import uuid
from typing import AsyncIterator

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from ros2_agent.agent.core import ROS2Agent
from ros2_agent.config.settings import Settings
from ros2_agent.simulation.robot_sim import get_sim

import pathlib

STATIC_DIR = pathlib.Path(__file__).parent / "static"


# ── Request / response models ─────────────────────────────────────────────────

class ChatRequest(BaseModel):
    message: str
    session_id: str = "web-default"


class ChatResponse(BaseModel):
    response: str
    session_id: str


# ── App factory ───────────────────────────────────────────────────────────────

def create_app(settings: Settings | None = None) -> FastAPI:
    settings = settings or Settings()
    agent = ROS2Agent(settings)
    sim = get_sim()  # physics singleton — shared with mock layer

    fast_app = FastAPI(
        title="ROS2 Agent API",
        description="LLM-powered agent for ROS 2 robots",
        version="0.1.0",
        docs_url="/docs",
    )

    fast_app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    @fast_app.on_event("startup")
    async def _start_sim() -> None:
        """Launch the physics simulation as a background asyncio task."""
        sim.start_background()

    # ── Routes ────────────────────────────────────────────────────────────────

    @fast_app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def root() -> str:
        index_path = STATIC_DIR / "index.html"
        if index_path.exists():
            return index_path.read_text(encoding="utf-8")
        return "<h1>ROS2 Agent Web UI</h1><p>Static files not found.</p>"

    @fast_app.get("/health")
    async def health() -> dict:
        return {
            "status": "ok",
            "ros_available": agent.ros_available,
            "mock": settings.mock_ros2,
            "provider": settings.get_provider_display(),
        }

    @fast_app.post("/chat", response_model=ChatResponse)
    async def chat(req: ChatRequest) -> ChatResponse:
        try:
            response = agent.chat(req.message, session_id=req.session_id)
            return ChatResponse(response=response, session_id=req.session_id)
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @fast_app.get("/chat/stream")
    async def chat_stream(
        message: str = Query(..., description="Natural language query"),
        session_id: str = Query(default="web-default"),
    ) -> StreamingResponse:
        """Server-Sent Events stream for real-time agent responses."""

        async def event_generator() -> AsyncIterator[str]:
            try:
                async for event in agent.astream(message, session_id=session_id):
                    payload = json.dumps({"kind": event.kind, "data": event.data})
                    yield f"data: {payload}\n\n"
                    await asyncio.sleep(0)  # allow other coroutines to run
            except Exception as exc:  # noqa: BLE001
                error_payload = json.dumps({"kind": "error", "data": str(exc)})
                yield f"data: {error_payload}\n\n"

        return StreamingResponse(
            event_generator(),
            media_type="text/event-stream",
            headers={
                "Cache-Control": "no-cache",
                "X-Accel-Buffering": "no",
            },
        )

    @fast_app.get("/ros/topics")
    async def ros_topics(filter_str: str = "") -> dict:
        result = agent.bridge.run(["topic", "list"])
        topics = [t for t in result.stdout.strip().splitlines() if t] if result.success else []
        if filter_str:
            topics = [t for t in topics if filter_str.lower() in t.lower()]
        return {"topics": sorted(topics), "count": len(topics)}

    @fast_app.get("/ros/nodes")
    async def ros_nodes() -> dict:
        result = agent.bridge.run(["node", "list"])
        nodes = [n for n in result.stdout.strip().splitlines() if n] if result.success else []
        return {"nodes": sorted(nodes), "count": len(nodes)}

    @fast_app.get("/ros/services")
    async def ros_services() -> dict:
        result = agent.bridge.run(["service", "list"])
        services = [s for s in result.stdout.strip().splitlines() if s] if result.success else []
        return {"services": sorted(services), "count": len(services)}

    @fast_app.get("/ros/status")
    async def ros_status() -> dict:
        nodes_r = agent.bridge.run(["node", "list"])
        topics_r = agent.bridge.run(["topic", "list"])
        node_count = len(nodes_r.stdout.strip().splitlines()) if nodes_r.success else 0
        topic_count = len(topics_r.stdout.strip().splitlines()) if topics_r.success else 0

        batt_pct: float | None = round(sim.state.battery, 1)

        return {
            "ros_available": agent.ros_available,
            "mock": settings.mock_ros2,
            "node_count": node_count,
            "topic_count": topic_count,
            "battery_pct": batt_pct,
            "domain_id": settings.ros_domain_id,
            "distro": settings.ros_distro,
        }

    @fast_app.websocket("/ws/sim")
    async def ws_sim(websocket: WebSocket) -> None:
        """Stream simulation state at ~20 Hz over WebSocket."""
        await websocket.accept()
        try:
            while True:
                await asyncio.sleep(0.05)  # 20 Hz
                await websocket.send_json(sim.get_state_dict())
        except (WebSocketDisconnect, Exception):
            pass

    @fast_app.post("/sim/goal")
    async def sim_goal(x: float, y: float) -> dict:
        """Directly set a navigation goal without going through the LLM."""
        import random as _r
        sim.set_goal(x, y, goal_id=_r.randint(100000, 999999))
        return {"ok": True, "goal_x": x, "goal_y": y}

    @fast_app.post("/sim/cmd_vel")
    async def sim_cmd_vel(vx: float = 0.0, omega: float = 0.0) -> dict:
        """Directly publish a velocity command."""
        sim.set_cmd_vel(vx, omega)
        return {"ok": True, "vx": vx, "omega": omega}

    return fast_app
