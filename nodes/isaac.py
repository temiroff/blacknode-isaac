"""Typed Blacknode nodes for direct Isaac Sim policy deployment."""
from __future__ import annotations

import base64
import html
from typing import Any

from blacknode.node import Any as AnyPort
from blacknode.node import Bool, Dict, Enum, Float, Image, Int, List, Text, node

from . import runtime

# Editor runtime status and Stop All resolve these through the registered node
# module so package reloads retain the exact state tables used by live nodes.
runtime_status = runtime.runtime_status
stop_runtime_services = runtime.stop_runtime_services

_CATEGORY = "Isaac Sim"


def _dashboard(status: dict[str, Any], title: str) -> str:
    running = bool(status.get("running"))
    connected = bool(status.get("connected", running))
    armed = bool(status.get("armed"))
    fault = bool(status.get("last_error")) or status.get("phase") == "fault"
    color = "#ef4444" if fault else "#22c55e" if armed else "#76b900" if connected else "#f59e0b"
    phase = str(status.get("phase") or status.get("control_mode") or ("connected" if connected else "waiting")).upper()
    error = html.escape(str(status.get("last_error") or "ready"))[:120]
    svg = (
        '<svg xmlns="http://www.w3.org/2000/svg" width="760" height="300" viewBox="0 0 760 300">'
        '<rect width="760" height="300" rx="20" fill="#0b1020"/>'
        f'<circle cx="38" cy="42" r="10" fill="{color}"/>'
        f'<text x="58" y="50" fill="#f8fafc" font-family="sans-serif" font-size="22" font-weight="800">{html.escape(title)} · {html.escape(phase)}</text>'
        f'<text x="34" y="116" fill="#94a3b8" font-family="sans-serif" font-size="13">OBSERVATIONS / INFERENCES</text>'
        f'<text x="34" y="150" fill="#f8fafc" font-family="monospace" font-size="28">{int(status.get("observation_sequence") or status.get("inference_count") or 0)}</text>'
        f'<text x="330" y="116" fill="#94a3b8" font-family="sans-serif" font-size="13">COMMANDS</text>'
        f'<text x="330" y="150" fill="#f8fafc" font-family="monospace" font-size="28">{int(status.get("command_sequence") or status.get("command_count") or 0)}</text>'
        f'<text x="34" y="210" fill="{color}" font-family="sans-serif" font-size="18" font-weight="800">{"ARMED" if armed else "SIM MOTION DISARMED"}</text>'
        f'<text x="34" y="260" fill="#fca5a5" font-family="sans-serif" font-size="13">{error}</text>'
        '</svg>'
    )
    return "data:image/svg+xml;base64," + base64.b64encode(svg.encode("utf-8")).decode("ascii")


@node(
    name="IsaacPolicySafetyGate", category=_CATEGORY,
    description="Configure USD joint, velocity, freshness, optional workspace, and replay-log safety for Isaac policy execution.",
    inputs={
        "trigger": AnyPort,
        "max_velocity_deg_s": Float(default=30.0),
        "max_step_deg": Float(default=3.0),
        "stale_after": Float(default=0.5),
        "loop_hz": Float(default=10.0),
        "workspace_limits": Dict(default={}),
        "log_dir": Text(default=""),
    },
    outputs={"ok": Bool, "safety": Dict, "report": Text},
    primary_inputs=["trigger"], primary_outputs=["safety", "report"],
)
def isaac_policy_safety_gate(ctx: dict) -> dict:
    workspace = dict(ctx.get("workspace_limits") or {})
    safety = {
        "kind": "blacknode.policy-safety-gate", "schema_version": 1,
        "max_velocity_deg_s": max(0.0, float(ctx.get("max_velocity_deg_s") or 0.0)),
        "max_step_deg": max(0.0, float(ctx.get("max_step_deg") or 0.0)),
        "stale_after": max(0.05, float(ctx.get("stale_after") or 0.5)),
        "loop_hz": max(1.0, min(60.0, float(ctx.get("loop_hz") or 10.0))),
        "request_timeout": 1.0,
        "require_calibration": True,
        "workspace_topic": "isaac://observation" if workspace else "",
        "workspace_limits": workspace,
        "log_dir": str(ctx.get("log_dir") or "").strip(),
    }
    return {
        "ok": True, "safety": safety,
        "report": "Isaac safety gate ready; motion remains disarmed until an explicit arm action",
    }


@node(
    name="IsaacPolicyBridge", live=True, category=_CATEGORY,
    description="Start a loopback bridge that receives Isaac articulation/RGB observations and returns safety-gated policy targets. Starts disarmed.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["status", "start", "stop"], default="status"),
        "run_id": Text(default="isaac-policy-bridge"),
        "artifact": Dict(default={}),
        "host": Text(default="127.0.0.1"),
        "port": Int(default=8770),
    },
    outputs={
        "ok": Bool, "running": Bool, "connected": Bool, "bridge": Dict,
        "bridge_url": Text, "status": Dict, "dashboard": Image, "report": Text,
    },
    primary_inputs=["trigger", "artifact"], primary_outputs=["bridge", "dashboard", "report"],
)
def isaac_policy_bridge(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "isaac-policy-bridge").strip() or "isaac-policy-bridge"
    action = str(ctx.get("action") or "status").lower()
    try:
        if action == "start":
            status = runtime.start_bridge(
                run_id, dict(ctx.get("artifact") or {}), str(ctx.get("host") or "127.0.0.1"),
                int(ctx.get("port") or 8770),
            )
        elif action == "stop":
            status = runtime.stop_bridge(run_id)
        else:
            status = runtime.bridge_status(run_id)
        running = bool(status.get("running"))
        report = (
            f"Isaac bridge {'connected' if status.get('connected') else 'waiting'} at {status.get('bridge_url')}"
            if running else "Isaac bridge stopped"
        )
        if status.get("last_error"):
            report += f"; {status['last_error']}"
        return {
            "ok": not bool(status.get("last_error")), "running": running,
            "connected": bool(status.get("connected")), "bridge": status,
            "bridge_url": str(status.get("bridge_url") or ""), "status": status,
            "dashboard": _dashboard(status, "ISAAC BRIDGE"), "report": report,
        }
    except Exception as exc:  # noqa: BLE001
        status = {**runtime.bridge_status(run_id), "last_error": str(exc)}
        return {
            "ok": False, "running": False, "connected": False, "bridge": {},
            "bridge_url": "", "status": status, "dashboard": _dashboard(status, "ISAAC BRIDGE"),
            "report": f"Isaac bridge FAILED: {exc}",
        }


@node(
    name="IsaacPolicyRuntime", live=True, category=_CATEGORY,
    description="Run a loaded policy continuously from Isaac RGB/state observations and apply targets only after explicit arming, with e-stop and takeover controls.",
    inputs={
        "trigger": AnyPort,
        "action": Enum(["status", "check", "start", "arm", "disarm", "estop", "reset_estop", "takeover", "reset_takeover", "stop"], default="status"),
        "run_id": Text(default="isaac-policy"),
        "artifact": Dict(default={}),
        "bridge": Dict(default={}),
        "safety": Dict(default={}),
        "device": Enum(["auto", "cuda", "cpu"], default="auto"),
    },
    outputs={
        "ok": Bool, "running": Bool, "armed": Bool, "emergency_stop": Bool,
        "human_takeover": Bool, "phase": Text, "prediction": Dict, "action": Dict,
        "clamped": List, "metrics": Dict, "dashboard": Image, "log_path": Text, "report": Text,
    },
    primary_inputs=["trigger", "artifact", "bridge", "safety"],
    primary_outputs=["dashboard", "metrics", "report"],
)
def isaac_policy_runtime(ctx: dict) -> dict:
    run_id = str(ctx.get("run_id") or "isaac-policy").strip() or "isaac-policy"
    action = str(ctx.get("action") or "status").lower()
    try:
        if action == "status":
            status = runtime.policy_status(run_id)
        elif action == "check":
            contract = runtime.check_policy(
                dict(ctx.get("artifact") or {}), dict(ctx.get("bridge") or {}),
                dict(ctx.get("safety") or {}),
            )
            status = {
                **runtime.policy_status(run_id), "phase": "ready",
                "joint_names": contract["joint_names"], "camera_names": contract["camera_names"],
                "prim_path": contract["prim_path"],
            }
        elif action == "start":
            status = runtime.start_policy(
                run_id, dict(ctx.get("artifact") or {}), dict(ctx.get("bridge") or {}),
                dict(ctx.get("safety") or {}), str(ctx.get("device") or "auto"),
            )
        else:
            status = runtime.control_policy(run_id, action)
        report = (
            f"Isaac policy {status.get('phase')}: "
            f"{'ARMED' if status.get('armed') else 'simulation motion disarmed'}; "
            f"{int(status.get('inference_count') or 0)} inference(s), "
            f"{int(status.get('command_count') or 0)} command(s)"
        )
        if status.get("last_error"):
            report += f"; {status['last_error']}"
        return {
            "ok": not bool(status.get("last_error")) and status.get("phase") != "fault",
            "running": bool(status.get("running")), "armed": bool(status.get("armed")),
            "emergency_stop": bool(status.get("emergency_stop")),
            "human_takeover": bool(status.get("human_takeover")),
            "phase": str(status.get("phase") or "unknown"),
            "prediction": dict(status.get("last_prediction") or {}),
            "action": dict(status.get("last_action") or {}),
            "clamped": list(status.get("clamped") or []), "metrics": status,
            "dashboard": _dashboard(status, "ISAAC POLICY"),
            "log_path": str(status.get("log_path") or ""), "report": report,
        }
    except Exception as exc:  # noqa: BLE001
        status = {**runtime.policy_status(run_id), "phase": "fault", "last_error": str(exc)}
        return {
            "ok": False, "running": bool(status.get("running")), "armed": False,
            "emergency_stop": bool(status.get("emergency_stop")),
            "human_takeover": bool(status.get("human_takeover")), "phase": "fault",
            "prediction": {}, "action": {}, "clamped": [], "metrics": status,
            "dashboard": _dashboard(status, "ISAAC POLICY"),
            "log_path": str(status.get("log_path") or ""),
            "report": f"Isaac policy runtime FAILED: {exc}",
        }
