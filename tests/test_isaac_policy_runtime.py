"""Synthetic closed-loop Isaac bridge and safety-runtime tests."""
from __future__ import annotations

import base64
import io
import json
import urllib.request
from pathlib import Path

import numpy as np
from PIL import Image

import blacknode  # noqa: F401 - discover extension packages
from blacknode.node import _NODE_REGISTRY
from blacknode.pkg.blacknode_isaac import runtime
from blacknode.workflow import validate_workflow


def _artifact() -> dict:
    return {
        "kind": "blacknode.policy-artifact", "schema_version": 1,
        "policy_type": "act", "backend": "blacknode-native",
        "action_mode": "absolute_joint_position", "units": "radians",
        "joint_names": ["shoulder", "gripper"], "camera_names": ["front"],
        "state_dim": 2, "action_dim": 2, "path": "synthetic",
    }


def _safety(tmp_path: Path | None = None) -> dict:
    return {
        "kind": "blacknode.policy-safety-gate", "schema_version": 1,
        "max_velocity_deg_s": 30.0, "max_step_deg": 3.0,
        "stale_after": 0.5, "loop_hz": 10.0, "request_timeout": 1.0,
        "require_calibration": True, "workspace_topic": "", "workspace_limits": {},
        "log_dir": str(tmp_path) if tmp_path else "",
    }


def _jpeg() -> str:
    buffer = io.BytesIO()
    Image.fromarray(np.full((8, 8, 3), 24, dtype=np.uint8)).save(buffer, format="JPEG")
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _observation(sequence: int = 1) -> dict:
    return {
        "kind": "blacknode.isaac-observation", "schema_version": 1,
        "sequence": sequence, "prim_path": "/World/SO_ARM101",
        "joint_names": ["shoulder", "gripper"], "positions": [0.0, 0.0],
        "joint_limits": {"shoulder": [-1.0, 1.0], "gripper": [0.0, 0.8]},
        "cameras": {"front": {"jpeg_base64": _jpeg()}}, "workspace": {},
    }


def _post(url: str, payload: dict) -> dict:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        url + "/observation", data=body, headers={"Content-Type": "application/json"}, method="POST",
    )
    with urllib.request.urlopen(request, timeout=3.0) as response:  # noqa: S310 - loopback test
        return json.loads(response.read().decode("utf-8"))


def test_nodes_registered_and_disarmed_by_default():
    assert "IsaacPolicySafetyGate" in _NODE_REGISTRY
    assert "IsaacPolicyBridge" in _NODE_REGISTRY
    assert "IsaacPolicyRuntime" in _NODE_REGISTRY
    assert _NODE_REGISTRY["IsaacPolicyBridge"]._bn_input_defaults["action"] == "status"
    assert _NODE_REGISTRY["IsaacPolicyRuntime"]._bn_input_defaults["action"] == "status"


def test_deployment_template_is_typed_and_safe_by_default():
    path = Path(__file__).resolve().parents[1] / "templates" / "isaac-act-policy-deployment.json"
    workflow = json.loads(path.read_text(encoding="utf-8"))
    assert validate_workflow(workflow).ok
    assert workflow["entrypoint"] == {"node_id": "runtime", "port": "dashboard"}
    assert workflow["node_meta"]["bridge"]["params"]["action"] == "status"
    assert workflow["node_meta"]["runtime"]["params"]["action"] == "status"
    assert {"blacknode-training", "blacknode-ros2", "blacknode-isaac"} <= set(
        workflow["metadata"]["required_packages"]
    )


def test_loopback_bridge_accepts_rgb_and_reports_commands():
    status = runtime.start_bridge("http-test", _artifact(), "127.0.0.1", 0)
    try:
        assert status["running"] and not status["connected"]
        response = _post(status["bridge_url"], _observation())
        assert response["kind"] == "blacknode.isaac-command"
        assert not response["armed"]
        connected = runtime.bridge_status("http-test")
        assert connected["connected"]
        assert connected["prim_path"] == "/World/SO_ARM101"
        with runtime._lock:
            state = runtime._bridges["http-test"]
        assert state.images["front"].shape == (8, 8, 3)
    finally:
        runtime.stop_bridge("http-test")


class _FakePolicy:
    def predict(self, qpos, images):
        assert qpos == [0.0, 0.0]
        assert images["front"].shape == (8, 8, 3)
        return {
            "kind": "blacknode.policy-prediction",
            "joint_names": ["shoulder", "gripper"], "action": [1.0, 0.5],
        }


def test_shared_policy_run_previews_arms_synchronizes_and_estops(tmp_path: Path):
    status = runtime.start_bridge("closed-loop", _artifact(), "127.0.0.1", 0)
    try:
        _post(status["bridge_url"], _observation())
        with runtime._lock:
            state = runtime._bridges["closed-loop"]
        contract = runtime.check_policy(_artifact(), runtime.bridge_status("closed-loop"), _safety(tmp_path))
        assert contract["joint_names"] == ["shoulder", "gripper"]
        robot, cameras = runtime._contract_inputs(state, _safety(tmp_path))
        policy_module = runtime._policy_module()
        run = policy_module.PolicyRun(
            "synthetic-isaac", _artifact(), robot, cameras, _safety(tmp_path), device="cpu",
            policy_loader=lambda _artifact, _device: _FakePolicy(),
            io_factory=lambda _contract, _cameras, _safety: runtime.IsaacBridgeIO(state),
        )
        run.phase = "preview"
        preview = run.step()
        assert not preview["commanded"]
        assert state.command == {}
        run.control("arm")
        _post(status["bridge_url"], _observation(2))
        synchronized = run.step()
        assert synchronized["commanded"]
        assert state.command == {"shoulder": 0.0, "gripper": 0.0}
        run.last_command_at -= 0.1
        _post(status["bridge_url"], _observation(3))
        run.step()
        assert state.command["shoulder"] < 1.0
        assert state.command["shoulder"] > 0.0
        run.control("estop")
        assert run.estop and not run.armed
        assert state.control_mode == "disarmed"
        assert state.command == {}
    finally:
        runtime.stop_bridge("closed-loop")


def test_isaac_client_is_closed_loop_and_clamps_usd_targets():
    source = (
        Path(__file__).resolve().parents[1] / "clients" / "isaac_policy_client.py"
    ).read_text(encoding="utf-8")
    assert "blacknode.isaac-observation" in source
    assert "response.get(\"armed\")" in source
    assert "set_dof_position_targets" in source
    assert "min(float(upper[index]), max(float(lower[index]), value))" in source
    assert "isaacsim.core.experimental.prims import Articulation" in source
