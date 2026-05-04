"""FastAPI web server for ROS2 Agent."""

from __future__ import annotations

import asyncio
import json
import pathlib
from collections.abc import AsyncIterator
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import HTMLResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from ros2_agent.agent.core import ROS2Agent
from ros2_agent.config.settings import Settings
from ros2_agent.simulation.robot_sim import get_sim

STATIC_DIR = pathlib.Path(__file__).parent / "static"


# ── Request / response models ─────────────────────────────────────────────────


class ChatRequest(BaseModel):
    """Request body for the /chat endpoint."""

    message: str
    session_id: str = "web-default"


class ChatResponse(BaseModel):
    """Response body for the /chat endpoint."""

    response: str
    session_id: str


class WaypointsRequest(BaseModel):
    """Request body for setting navigation waypoints."""

    waypoints: list[list[float]]


class PhysicsUploadRequest(BaseModel):
    """Request body for uploading a custom physics profile."""

    name: str = "custom-physics"
    config: dict


class PolicyUploadRequest(BaseModel):
    """Request body for uploading a robot control policy."""

    name: str = "custom-policy"
    config: dict


class PolicyRunRequest(BaseModel):
    """Request body for triggering a policy execution run."""

    duration_s: float = 5.0


class UniversalProfileRequest(BaseModel):
    """Request body for uploading a universal robot profile."""

    schema_version: str = "1.0.0"
    name: str = "universal-robot-profile"
    profile_type: str = "robot_profile"
    profile: dict


# ── App factory ───────────────────────────────────────────────────────────────


def create_app(settings: Settings | None = None) -> FastAPI:
    """Create and configure the FastAPI application."""
    settings = settings or Settings()
    agent = ROS2Agent(settings)
    sim = get_sim()  # physics singleton — shared with mock layer

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        sim.start_background()
        yield
        sim.stop()

    fast_app = FastAPI(
        title="ROS2 Agent API",
        description="LLM-powered agent for ROS 2 robots",
        version="0.2.0",
        docs_url="/docs",
        lifespan=lifespan,
    )

    fast_app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    if STATIC_DIR.exists():
        fast_app.mount("/static", StaticFiles(directory=STATIC_DIR), name="static")

    @fast_app.get("/", response_class=HTMLResponse, include_in_schema=False)
    async def root() -> str:
        index_path = STATIC_DIR / "index.html"
        if index_path.exists():
            return index_path.read_text(encoding="utf-8")
        return "<h1>ROS2 Agent Web UI</h1><p>Static files not found.</p>"

    @fast_app.get("/dashboard", response_class=HTMLResponse, include_in_schema=False)
    async def dashboard() -> str:
        """Live end-to-end data flow dashboard."""
        dash_path = STATIC_DIR / "dashboard.html"
        if dash_path.exists():
            return dash_path.read_text(encoding="utf-8")
        return "<h1>Dashboard not found</h1>"

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

    @fast_app.websocket("/ws/telemetry")
    async def ws_telemetry(websocket: WebSocket) -> None:
        """Stream full sensor telemetry (all layers) at 20 Hz over WebSocket."""
        await websocket.accept()
        try:
            while True:
                await asyncio.sleep(0.05)  # 20 Hz
                state = sim.get_state_dict()
                # Commanded velocities (before motor model)
                state["cmd_vx"] = round(sim._cmd_vx, 4)
                state["cmd_omega"] = round(sim._cmd_omega, 4)
                state["manual_override"] = sim._manual_override
                state["escape_countdown"] = sim._escape_countdown
                state["no_move_ticks"] = sim._no_move_ticks
                # Camera sensor data
                state["camera_detections"] = sim.state.camera_detections
                state["camera_depth_row"] = sim.state.camera_depth_row
                # Per-tick processing times
                state["timing"] = sim.state.timing
                # SLAM state
                state["slam"] = sim.get_slam_state()
                # Joint states (differential-drive kinematics)
                state["joints"] = sim.get_joint_states()
                # IMU state
                state["imu"] = sim.get_imu_state()
                await websocket.send_json(state)
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

    @fast_app.post("/sim/waypoints")
    async def sim_waypoints(req: WaypointsRequest) -> dict:
        """Set a waypoint path for the robot to follow."""
        waypoints = [(wp[0], wp[1]) for wp in req.waypoints]
        sim.set_waypoints(waypoints)
        return {"ok": True, "count": len(waypoints)}

    @fast_app.get("/sim/events")
    async def sim_events(limit: int = 50) -> dict:
        """Return recent simulation events (log)."""
        return {"events": sim.get_events(limit=limit)}

    @fast_app.post("/sim/reset")
    async def sim_reset() -> dict:
        """Reset the simulation to initial state."""
        s = sim.state
        s.x = 1.5
        s.y = 1.5
        s.theta = 0.0
        s.vx = 0.0
        s.omega = 0.0
        s.battery = 85.0
        s.goal_x = None
        s.goal_y = None
        s.nav_active = False
        s.nav_succeeded = False
        s.path.clear()
        s.full_path.clear()
        s.waypoints.clear()
        s.waypoint_index = 0
        s.velocity_history.clear()
        sim._cmd_vx = 0.0
        sim._cmd_omega = 0.0
        sim._manual_override = False
        sim._escape_countdown = 0
        sim._consec_escapes = 0
        sim._events.clear()
        sim._log_event("system", "Simulation reset")
        return {"ok": True}

    @fast_app.get("/sim/occupancy")
    async def sim_occupancy(res: int = 20) -> dict:
        """Return a downsampled occupancy grid for visualization."""
        return {"grid": sim.get_occupancy_grid(resolution=res), "resolution": res}

    @fast_app.get("/sim/full_path")
    async def sim_full_path() -> dict:
        """Return the full session path trail."""
        return {"path": sim.get_full_path()}

    @fast_app.post("/sim/physics/upload")
    async def sim_upload_physics(req: PhysicsUploadRequest) -> dict:
        """Upload and apply a physics profile for the simulator."""
        try:
            result = sim.apply_physics_profile(req.name, req.config)
            return {"ok": True, **result}
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @fast_app.post("/sim/policy/upload")
    async def sim_upload_policy(req: PolicyUploadRequest) -> dict:
        """Upload a policy profile for later rollout."""
        try:
            result = sim.load_policy_profile(req.name, req.config)
            return {"ok": True, **result}
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    @fast_app.post("/sim/policy/run")
    async def sim_run_policy(req: PolicyRunRequest) -> dict:
        """Run the currently uploaded policy for a bounded duration."""
        try:
            result = await sim.run_loaded_policy(duration_s=req.duration_s)
            return {"ok": True, **result}
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=500, detail=str(exc)) from exc

    @fast_app.post("/sim/profile/upload")
    async def sim_upload_universal_profile(req: UniversalProfileRequest) -> dict:
        """Upload one universal URDF/USD-inspired robot profile JSON.

        Expected envelope:
        {
          "schema_version": "1.0.0",
          "name": "...",
          "profile_type": "robot_profile",
          "profile": {
            "physics": {...},
            "visual": {...},
            "kinematics": {...},
            "sensors": [...],
            "controllers": {...},
            "policy": {...}
          }
        }
        """
        if req.profile_type != "robot_profile":
            raise HTTPException(status_code=400, detail="profile_type must be 'robot_profile'")
        try:
            result = sim.load_universal_profile(req.model_dump())
            return {"ok": True, **result}
        except Exception as exc:  # noqa: BLE001
            raise HTTPException(status_code=400, detail=str(exc)) from exc

    return fast_app
