"""Closed-loop Blacknode policy client for the Isaac Sim Script Editor.

Run inside Isaac Sim after starting ``IsaacPolicyBridge`` in Blacknode::

    exec(open(r"<repo>/packages/blacknode-isaac/clients/isaac_policy_client.py").read())
    start_blacknode_isaac_policy(
        "http://127.0.0.1:8770",
        "/World/so_arm101",
        {"front": "/World/front_camera"},
    )

The client sends measured articulation state and rendered RGB observations to
Blacknode. It applies returned position targets only while the response is
explicitly armed, and clamps them again to the articulation's USD limits.
"""
from __future__ import annotations

import asyncio
import base64
import io
import json
import math
import threading
import urllib.request

_state = {
    "running": False,
    "generation": 0,
    "task": None,
    "status": "stopped",
    "observations": 0,
    "commands": 0,
    "last_command_sequence": 0,
}


def _numpy(value):
    return value.numpy() if hasattr(value, "numpy") else value


def _request_json(url, payload=None, timeout=5.0):
    body = None if payload is None else json.dumps(payload, separators=(",", ":")).encode("utf-8")
    request = urllib.request.Request(
        url, data=body,
        headers={"Content-Type": "application/json"} if body is not None else {},
        method="POST" if body is not None else "GET",
    )
    with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310 - explicit loopback bridge
        return json.loads(response.read().decode("utf-8"))


def _encode_rgb(array):
    import numpy as np
    from PIL import Image

    image = np.asarray(array)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError(f"Isaac RGB annotator returned invalid shape {image.shape}")
    image = image[:, :, :3]
    if image.dtype != np.uint8:
        image = np.clip(image, 0, 255).astype(np.uint8)
    buffer = io.BytesIO()
    Image.fromarray(image, mode="RGB").save(buffer, format="JPEG", quality=90)
    return base64.b64encode(buffer.getvalue()).decode("ascii")


def _policy_to_dof(joint_names, dof_names, joint_mapping):
    lookup = {str(name): index for index, name in enumerate(dof_names)}
    mapping = {}
    for policy_name in joint_names:
        dof_name = str((joint_mapping or {}).get(policy_name) or policy_name)
        if dof_name not in lookup:
            raise ValueError(f"policy joint {policy_name!r} has no Isaac DOF {dof_name!r}")
        mapping[policy_name] = lookup[dof_name]
    return mapping


async def _run(bridge_url, prim_path, camera_paths, resolution, hz, joint_mapping, workspace_prim_path, generation):
    import numpy as np
    import omni.kit.app
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from pxr import Usd, UsdGeom
    from isaacsim.core.experimental.prims import Articulation

    bridge_url = str(bridge_url).rstrip("/")
    config = await asyncio.to_thread(_request_json, bridge_url)
    joint_names = [str(name) for name in config.get("joint_names") or []]
    camera_names = [str(name) for name in config.get("camera_names") or []]
    missing = [name for name in camera_names if name not in camera_paths]
    if missing:
        raise ValueError("camera_paths is missing policy camera(s): " + ", ".join(missing))

    timeline = omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
    await omni.kit.app.get_app().next_update_async()
    articulation = Articulation(str(prim_path))
    await omni.kit.app.get_app().next_update_async()
    if not articulation.is_physics_tensor_entity_valid():
        raise RuntimeError(f"{prim_path} is not a valid initialized articulation root")
    dof_names = list(articulation.dof_names)
    mapping = _policy_to_dof(joint_names, dof_names, joint_mapping)
    lower, upper = articulation.get_dof_limits()
    lower = np.asarray(_numpy(lower), dtype=np.float32).reshape(-1)
    upper = np.asarray(_numpy(upper), dtype=np.float32).reshape(-1)
    limits = {
        name: [float(lower[index]), float(upper[index])]
        for name, index in mapping.items()
    }
    for name, bounds in limits.items():
        if not all(math.isfinite(value) for value in bounds) or bounds[0] >= bounds[1]:
            raise ValueError(f"Isaac DOF limits are invalid for policy joint {name}")

    annotators = {}
    render_products = []
    for name in camera_names:
        product = rep.create.render_product(str(camera_paths[name]), tuple(resolution))
        annotator = rep.AnnotatorRegistry.get_annotator("rgb")
        annotator.attach([product])
        render_products.append(product)
        annotators[name] = annotator

    period = 1.0 / max(1.0, min(60.0, float(hz)))
    sequence = 0
    _state["status"] = f"connected: {len(joint_names)} joints, {len(camera_names)} cameras"
    try:
        while _state["running"] and generation == _state["generation"]:
            started = asyncio.get_running_loop().time()
            await omni.kit.app.get_app().next_update_async()
            measured = np.asarray(_numpy(articulation.get_dof_positions()), dtype=np.float32).reshape(-1)
            positions = [float(measured[mapping[name]]) for name in joint_names]
            cameras = {
                name: {"jpeg_base64": _encode_rgb(annotators[name].get_data())}
                for name in camera_names
            }
            workspace = {}
            if workspace_prim_path:
                prim = omni.usd.get_context().get_stage().GetPrimAtPath(str(workspace_prim_path))
                if not prim.IsValid():
                    raise ValueError(f"workspace_prim_path is invalid: {workspace_prim_path}")
                translation = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                ).ExtractTranslation()
                workspace = {"x": float(translation[0]), "y": float(translation[1]), "z": float(translation[2])}
            sequence += 1
            payload = {
                "kind": "blacknode.isaac-observation",
                "schema_version": 1,
                "sequence": sequence,
                "prim_path": str(prim_path),
                "joint_names": joint_names,
                "positions": positions,
                "joint_limits": limits,
                "cameras": cameras,
                "workspace": workspace,
            }
            response = await asyncio.to_thread(
                _request_json, bridge_url + "/observation", payload, max(2.0, period * 4.0),
            )
            _state["observations"] = sequence
            command_sequence = int(response.get("command_sequence") or 0)
            command = response.get("command") if isinstance(response.get("command"), dict) else {}
            if response.get("armed") and command and command_sequence > int(_state["last_command_sequence"]):
                targets = measured.copy()
                for name in joint_names:
                    value = float(command[name])
                    index = mapping[name]
                    targets[index] = min(float(upper[index]), max(float(lower[index]), value))
                articulation.set_dof_position_targets(targets)
                _state["last_command_sequence"] = command_sequence
                _state["commands"] = int(_state["commands"]) + 1
                _state["status"] = f"ARMED · applied command {command_sequence}"
            else:
                _state["status"] = (
                    f"{str(response.get('control_mode') or 'disarmed').upper()} · "
                    f"observation {sequence} · commands {int(_state['commands'])}"
                )
            delay = period - (asyncio.get_running_loop().time() - started)
            if delay > 0:
                await asyncio.sleep(delay)
    finally:
        for annotator in annotators.values():
            try:
                annotator.detach(render_products)
            except Exception:
                pass
        _state["running"] = False
        if generation == _state["generation"]:
            _state["status"] = "stopped"


async def _guarded_run(*args):
    try:
        await _run(*args)
    except Exception as exc:  # noqa: BLE001 - shown through status()
        _state["running"] = False
        _state["status"] = f"error: {type(exc).__name__}: {exc}"


def start_blacknode_isaac_policy(
    bridge_url,
    prim_path,
    camera_paths,
    *,
    resolution=(320, 240),
    hz=10.0,
    joint_mapping=None,
    workspace_prim_path=None,
):
    """Start closed-loop policy I/O inside Isaac Sim; returns immediately."""
    stop_blacknode_isaac_policy()
    if not str(bridge_url).startswith("http://127.0.0.1:") and not str(bridge_url).startswith("http://localhost:"):
        raise ValueError("bridge_url must point to the local Blacknode Isaac bridge")
    if not str(prim_path).startswith("/"):
        raise ValueError("prim_path must be an absolute USD path")
    if not isinstance(camera_paths, dict) or not camera_paths:
        raise ValueError("camera_paths must map policy camera names to USD camera prim paths")
    _state["generation"] = int(_state["generation"]) + 1
    generation = int(_state["generation"])
    _state.update(
        running=True, status="starting", observations=0, commands=0,
        last_command_sequence=0,
    )
    task = asyncio.ensure_future(_guarded_run(
        str(bridge_url), str(prim_path), dict(camera_paths), tuple(resolution),
        float(hz), dict(joint_mapping or {}),
        str(workspace_prim_path or ""), generation,
    ))
    _state["task"] = task
    return status_blacknode_isaac_policy()


def stop_blacknode_isaac_policy():
    _state["running"] = False
    _state["generation"] = int(_state["generation"]) + 1
    task = _state.get("task")
    if task is not None and hasattr(task, "cancel"):
        task.cancel()
    _state["task"] = None
    if not str(_state.get("status") or "").startswith("error:"):
        _state["status"] = "stopped"
    return status_blacknode_isaac_policy()


def status_blacknode_isaac_policy():
    return {
        "running": bool(_state["running"]),
        "status": str(_state["status"]),
        "observations": int(_state["observations"]),
        "commands": int(_state["commands"]),
        "last_command_sequence": int(_state["last_command_sequence"]),
    }
