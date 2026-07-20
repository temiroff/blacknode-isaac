"""Managed loopback bridge and closed-loop policy runtime for Isaac Sim."""
from __future__ import annotations

import atexit
import base64
import io
import json
import math
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import urlparse

try:
    import numpy as np
except Exception:  # pragma: no cover - package health reports this
    np = None

try:
    from PIL import Image as PILImage
except Exception:  # pragma: no cover - package health reports this
    PILImage = None


def _policy_module():
    try:
        from blacknode.pkg.blacknode_controllers.policy import policy_runtime
    except Exception as exc:  # pragma: no cover - surfaced by nodes
        raise RuntimeError("blacknode-controllers/policy is required for the shared policy safety runtime") from exc
    return policy_runtime


def _finite(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _loopback(host: str) -> bool:
    return str(host or "").strip().lower() in {"127.0.0.1", "localhost", "::1"}


class BridgeState:
    """Latest Isaac observation and command exchanged over a local HTTP bridge."""

    def __init__(self, run_id: str, artifact: dict[str, Any], host: str, port: int) -> None:
        if np is None or PILImage is None:
            raise RuntimeError("numpy and Pillow are required for the Isaac policy bridge")
        if artifact.get("kind") != "blacknode.policy-artifact":
            raise ValueError("connect a blacknode.policy-artifact")
        self.run_id = str(run_id)
        self.artifact = dict(artifact)
        self.host = str(host or "127.0.0.1")
        if not _loopback(self.host):
            raise ValueError("IsaacPolicyBridge is loopback-only; use 127.0.0.1 or localhost")
        self.port = int(port)
        self.joint_names = [str(name) for name in artifact.get("joint_names") or []]
        self.camera_names = [str(name) for name in artifact.get("camera_names") or []]
        if not self.joint_names or not self.camera_names:
            raise ValueError("policy artifact must declare ordered joints and cameras")
        self.lock = threading.RLock()
        self.server: ThreadingHTTPServer | None = None
        self.thread: threading.Thread | None = None
        self.started_at = time.time()
        self.connected = False
        self.prim_path = ""
        self.pose: dict[str, float] = {}
        self.joint_limits: dict[str, tuple[float, float]] = {}
        self.images: dict[str, Any] = {}
        self.workspace: dict[str, float] = {}
        self.observation_at = 0.0
        self.camera_at: dict[str, float] = {}
        self.workspace_at = 0.0
        self.observation_sequence = 0
        self.command: dict[str, float] = {}
        self.command_sequence = 0
        self.control_mode = "disarmed"
        self.last_error = ""

    @property
    def url(self) -> str:
        return f"http://{self.host}:{self.port}"

    def _decode_camera(self, value: Any) -> Any:
        payload = dict(value) if isinstance(value, dict) else {}
        encoded = str(payload.get("jpeg_base64") or "")
        if not encoded:
            raise ValueError("camera payload is missing jpeg_base64")
        raw = base64.b64decode(encoded, validate=True)
        assert PILImage is not None and np is not None
        return np.asarray(PILImage.open(io.BytesIO(raw)).convert("RGB"), dtype=np.uint8)

    def ingest(self, payload: dict[str, Any]) -> dict[str, Any]:
        if payload.get("kind") != "blacknode.isaac-observation":
            raise ValueError("expected blacknode.isaac-observation")
        names = [str(name) for name in payload.get("joint_names") or []]
        if names != self.joint_names:
            raise ValueError(f"Isaac joint order does not match policy artifact: {names}")
        positions = list(payload.get("positions") or [])
        if len(positions) != len(names):
            raise ValueError("Isaac position count does not match joint_names")
        pose: dict[str, float] = {}
        for name, value in zip(names, positions):
            number = _finite(value)
            if number is None:
                raise ValueError(f"Isaac joint {name} is non-finite")
            pose[name] = number
        raw_limits = payload.get("joint_limits") if isinstance(payload.get("joint_limits"), dict) else {}
        limits: dict[str, tuple[float, float]] = {}
        for name in names:
            bounds = raw_limits.get(name)
            if not isinstance(bounds, (list, tuple)) or len(bounds) != 2:
                raise ValueError(f"Isaac joint {name} must provide USD limits [min,max] in radians")
            lower, upper = _finite(bounds[0]), _finite(bounds[1])
            if lower is None or upper is None or lower >= upper:
                raise ValueError(f"Isaac joint {name} has invalid USD limits")
            limits[name] = (lower, upper)
        raw_cameras = payload.get("cameras") if isinstance(payload.get("cameras"), dict) else {}
        missing = [name for name in self.camera_names if name not in raw_cameras]
        if missing:
            raise ValueError("Isaac observation is missing policy camera(s): " + ", ".join(missing))
        images = {name: self._decode_camera(raw_cameras[name]) for name in self.camera_names}
        workspace = payload.get("workspace") if isinstance(payload.get("workspace"), dict) else {}
        clean_workspace = {
            axis: float(value) for axis in ("x", "y", "z")
            if (value := _finite(workspace.get(axis))) is not None
        }
        now = time.monotonic()
        with self.lock:
            self.connected = True
            self.prim_path = str(payload.get("prim_path") or self.prim_path)
            self.pose = pose
            self.joint_limits = limits
            self.images = images
            self.workspace = clean_workspace
            self.observation_at = now
            self.camera_at = {name: now for name in self.camera_names}
            self.workspace_at = now if clean_workspace else 0.0
            self.observation_sequence = int(payload.get("sequence") or self.observation_sequence + 1)
            self.last_error = ""
            return self.exchange_payload()

    def exchange_payload(self) -> dict[str, Any]:
        with self.lock:
            return {
                "kind": "blacknode.isaac-command",
                "schema_version": 1,
                "run_id": self.run_id,
                "joint_names": list(self.joint_names),
                "command": dict(self.command),
                "command_sequence": self.command_sequence,
                "armed": self.control_mode == "armed",
                "control_mode": self.control_mode,
                "last_error": self.last_error,
            }

    def status(self) -> dict[str, Any]:
        with self.lock:
            age = time.monotonic() - self.observation_at if self.observation_at else float("inf")
            return {
                "kind": "blacknode.isaac-policy-bridge",
                "schema_version": 1,
                "run_id": self.run_id,
                "running": bool(self.thread and self.thread.is_alive()),
                "connected": self.connected,
                "bridge_url": self.url,
                "host": self.host,
                "port": self.port,
                "prim_path": self.prim_path,
                "joint_names": list(self.joint_names),
                "camera_names": list(self.camera_names),
                "observation_sequence": self.observation_sequence,
                "observation_age": age,
                "command_sequence": self.command_sequence,
                "control_mode": self.control_mode,
                "last_error": self.last_error,
                "elapsed_seconds": max(0.0, time.time() - self.started_at),
            }

    def start(self) -> None:
        state = self

        class Handler(BaseHTTPRequestHandler):
            def _json(self, status: int, value: dict[str, Any]) -> None:
                body = json.dumps(value, separators=(",", ":")).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                path = urlparse(self.path).path.rstrip("/")
                if path not in {"", "/status"}:
                    self._json(404, {"ok": False, "error": "not found"})
                    return
                self._json(200, {**state.status(), "artifact": {
                    "policy_type": state.artifact.get("policy_type"),
                    "joint_names": state.joint_names, "camera_names": state.camera_names,
                }})

            def do_POST(self) -> None:  # noqa: N802
                if urlparse(self.path).path.rstrip("/") != "/observation":
                    self._json(404, {"ok": False, "error": "not found"})
                    return
                try:
                    length = int(self.headers.get("Content-Length") or 0)
                    if length <= 0 or length > 64 * 1024 * 1024:
                        raise ValueError("observation body must be between 1 byte and 64 MiB")
                    payload = json.loads(self.rfile.read(length).decode("utf-8"))
                    self._json(200, state.ingest(dict(payload)))
                except Exception as exc:  # noqa: BLE001 - returned to the local Isaac client
                    with state.lock:
                        state.last_error = f"{type(exc).__name__}: {exc}"
                    self._json(400, {"ok": False, "error": state.last_error})

            def log_message(self, *_args: Any) -> None:
                pass

        self.server = ThreadingHTTPServer((self.host, self.port), Handler)
        self.port = int(self.server.server_address[1])
        self.thread = threading.Thread(
            target=self.server.serve_forever, daemon=True, name=f"blacknode-isaac-bridge-{self.run_id}",
        )
        self.thread.start()

    def stop(self) -> None:
        with self.lock:
            self.control_mode = "stopped"
            self.command = {}
        if self.server is not None:
            self.server.shutdown()
            self.server.server_close()
        if self.thread is not None and self.thread is not threading.current_thread():
            self.thread.join(timeout=2.0)


class IsaacBridgeIO:
    """I/O adapter consumed by the shared Blacknode PolicyRun."""

    def __init__(self, state: BridgeState) -> None:
        self.state = state

    def start(self) -> None:
        if not self.state.connected:
            raise RuntimeError("Isaac Sim has not sent an observation to the bridge")

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.state.lock:
            return {
                "pose": dict(self.state.pose),
                "pose_age": now - self.state.observation_at if self.state.observation_at else float("inf"),
                "images": {name: image.copy() for name, image in self.state.images.items()},
                "camera_ages": {
                    name: now - self.state.camera_at.get(name, 0.0)
                    if self.state.camera_at.get(name) else float("inf")
                    for name in self.state.camera_names
                },
                "workspace": dict(self.state.workspace),
                "workspace_age": now - self.state.workspace_at if self.state.workspace_at else float("inf"),
            }

    def publish(self, action: dict[str, float]) -> None:
        with self.state.lock:
            if not self.state.connected or self.state.control_mode != "armed":
                raise RuntimeError("Isaac bridge is disconnected or disarmed")
            self.state.command = {name: float(action[name]) for name in self.state.joint_names}
            self.state.command_sequence += 1

    def control(self, action: str) -> dict[str, Any]:
        if action not in {"exit_teach", "enter_teach"}:
            return {"ok": False, "error": f"unsupported Isaac control action: {action}"}
        with self.state.lock:
            self.state.control_mode = "armed" if action == "exit_teach" else "disarmed"
            if action == "enter_teach":
                self.state.command = {}
        return {"ok": True}

    def close(self) -> None:
        with self.state.lock:
            self.state.control_mode = "disarmed"
            self.state.command = {}


_bridges: dict[str, BridgeState] = {}
_policy_runs: dict[str, Any] = {}
_policy_bridges: dict[str, str] = {}
_lock = threading.RLock()


def start_bridge(run_id: str, artifact: dict[str, Any], host: str, port: int) -> dict[str, Any]:
    run_id = str(run_id or "isaac-policy-bridge").strip() or "isaac-policy-bridge"
    with _lock:
        current = _bridges.get(run_id)
        if current and current.thread and current.thread.is_alive():
            return current.status()
        state = BridgeState(run_id, artifact, host, port)
        _bridges[run_id] = state
    try:
        state.start()
    except Exception:
        with _lock:
            _bridges.pop(run_id, None)
        raise
    return state.status()


def bridge_status(run_id: str) -> dict[str, Any]:
    with _lock:
        state = _bridges.get(str(run_id or ""))
    if state is None:
        return {
            "kind": "blacknode.isaac-policy-bridge", "schema_version": 1,
            "run_id": str(run_id or ""), "running": False, "connected": False,
            "bridge_url": "", "joint_names": [], "camera_names": [], "last_error": "",
        }
    return state.status()


def stop_bridge(run_id: str) -> dict[str, Any]:
    with _lock:
        state = _bridges.get(str(run_id or ""))
        policy_ids = [key for key, bridge_id in _policy_bridges.items() if bridge_id == str(run_id or "")]
    for policy_id in policy_ids:
        control_policy(policy_id, "stop")
    if state is None:
        return bridge_status(run_id)
    state.stop()
    with _lock:
        _bridges.pop(str(run_id or ""), None)
    return {**state.status(), "running": False, "connected": False}


def _state_from_handle(bridge: dict[str, Any]) -> BridgeState:
    run_id = str(bridge.get("run_id") or "")
    with _lock:
        state = _bridges.get(run_id)
    if state is None or not state.thread or not state.thread.is_alive():
        raise ValueError("connect a running IsaacPolicyBridge")
    return state


def _contract_inputs(state: BridgeState, safety: dict[str, Any]) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    with state.lock:
        if not state.connected:
            raise ValueError("Isaac Sim has not connected to the bridge")
        limits = dict(state.joint_limits)
        prim_path = state.prim_path
    joints = [
        {
            "id": name,
            "safe_min_deg": math.degrees(limits[name][0]),
            "safe_max_deg": math.degrees(limits[name][1]),
        }
        for name in state.joint_names
    ]
    robot = {
        "driver": {
            "running": True,
            "calibration_path": f"isaac://{prim_path or 'articulation'}",
            "joints": joints,
        }
    }
    cameras = [
        {
            "kind": "blacknode.frame-stream", "schema_version": 1,
            "stream_id": name, "snapshot_url": f"isaac://{name}",
        }
        for name in state.camera_names
    ]
    return robot, cameras


def check_policy(artifact: dict[str, Any], bridge: dict[str, Any], safety: dict[str, Any]) -> dict[str, Any]:
    state = _state_from_handle(bridge)
    robot, cameras = _contract_inputs(state, safety)
    contract = _policy_module().validate_deployment_contract(artifact, robot, cameras, safety)
    return {**contract, "bridge_run_id": state.run_id, "prim_path": state.prim_path}


def start_policy(
    run_id: str, artifact: dict[str, Any], bridge: dict[str, Any], safety: dict[str, Any], device: str,
) -> dict[str, Any]:
    run_id = str(run_id or "isaac-policy").strip() or "isaac-policy"
    state = _state_from_handle(bridge)
    robot, cameras = _contract_inputs(state, safety)
    module = _policy_module()
    with _lock:
        current = _policy_runs.get(run_id)
        if current and current.thread.is_alive():
            raise RuntimeError(f"Isaac policy runtime {run_id!r} is already active")
        run = module.PolicyRun(
            run_id, artifact, robot, cameras, safety, device=device,
            io_factory=lambda _contract, _cameras, _safety: IsaacBridgeIO(state),
        )
        _policy_runs[run_id] = run
        _policy_bridges[run_id] = state.run_id
    try:
        run.start()
    except Exception:
        with _lock:
            _policy_runs.pop(run_id, None)
            _policy_bridges.pop(run_id, None)
        run.io.close()
        raise
    return run.status()


def policy_status(run_id: str) -> dict[str, Any]:
    with _lock:
        run = _policy_runs.get(str(run_id or ""))
    if run is None:
        return {
            "kind": "blacknode.policy-runtime", "schema_version": 1,
            "run_id": str(run_id or ""), "running": False, "phase": "not_started",
            "armed": False, "emergency_stop": False, "human_takeover": False,
            "inference_count": 0, "command_count": 0, "blocked_count": 0,
            "mean_inference_ms": 0.0, "last_prediction": {}, "last_action": {},
            "clamped": [], "last_error": "", "log_path": "",
        }
    return run.status()


def control_policy(run_id: str, action: str) -> dict[str, Any]:
    with _lock:
        run = _policy_runs.get(str(run_id or ""))
    if run is None:
        raise ValueError(f"Isaac policy runtime {run_id!r} was not found")
    if action == "stop":
        run.stop()
        with _lock:
            _policy_runs.pop(str(run_id or ""), None)
            _policy_bridges.pop(str(run_id or ""), None)
    else:
        run.control(action)
    return run.status()


def runtime_status() -> dict[str, Any]:
    with _lock:
        bridges = [state.status() for state in _bridges.values() if state.thread and state.thread.is_alive()]
        policies = [run.status() for run in _policy_runs.values() if run.thread.is_alive()]
    active = bool(bridges or policies)
    return {
        "ok": True,
        "active": active,
        "streams": [],
        "managed_runs": [
            *[{**item, "service": "isaac_bridge"} for item in bridges],
            *[{**item, "service": "isaac_policy"} for item in policies],
        ],
        "report": f"{len(bridges)} Isaac bridge(s), {len(policies)} policy runtime(s)",
    }


def stop_runtime_services() -> dict[str, Any]:
    with _lock:
        policy_ids = list(_policy_runs)
        bridge_ids = list(_bridges)
    errors: list[str] = []
    for run_id in policy_ids:
        try:
            control_policy(run_id, "stop")
        except Exception as exc:  # pragma: no cover - defensive shutdown
            errors.append(f"{run_id}: {exc}")
    for run_id in bridge_ids:
        try:
            stop_bridge(run_id)
        except Exception as exc:  # pragma: no cover
            errors.append(f"{run_id}: {exc}")
    return {
        "ok": not errors,
        "stopped": {"managed_runs": len(policy_ids) + len(bridge_ids)},
        "errors": errors,
        "report": f"stopped {len(policy_ids)} Isaac policy runtime(s) and {len(bridge_ids)} bridge(s)",
    }


atexit.register(stop_runtime_services)
