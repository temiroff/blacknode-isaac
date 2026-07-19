# blacknode-isaac

`blacknode-isaac` deploys Blacknode policy artifacts in a live Isaac Sim loop.
It receives measured articulation positions and named RGB sensors, runs the
policy through Blacknode's existing inference and safety engine, and returns
absolute joint-position targets to the simulator.

```text
Isaac RGB cameras + articulation state
                 ↓
        IsaacPolicyBridge
                 ↓
        IsaacPolicyRuntime
                 ↓
         shared SafetyGate
                 ↓
      Isaac articulation targets
```

The bridge binds to loopback only. Starting it does not authorize motion.
Starting `IsaacPolicyRuntime` begins continuous disarmed inference preview;
only an explicit `arm` action permits targets to be applied.

## Requirements

Install `blacknode-training`, `blacknode-ros2`, and `blacknode-isaac` in the
Blacknode environment. The Isaac-side client runs inside Isaac Sim's Python
environment and uses its built-in articulation and Replicator APIs.

## Nodes

| Node | Purpose |
| --- | --- |
| `IsaacPolicySafetyGate` | Configure USD joint/velocity/step bounds, source freshness, optional workspace limits, loop rate, and replay logs. |
| `IsaacPolicyBridge` | Host the loopback observation/command exchange for one Isaac client. |
| `IsaacPolicyRuntime` | Check, start, preview, arm, disarm, e-stop, take over, reset, or stop the managed policy loop. |

## Deploy an ACT artifact

1. Open **Isaac Sim ACT Policy Deployment**.
2. Set `artifact_path` to the exported policy directory.
3. Set `IsaacPolicyBridge.action=start`, cook once, then return it to `status`.
4. In Isaac Sim, load the robot articulation and the camera prims that match the
   policy's `camera_names`.
5. Run the client from Isaac's Script Editor:

```python
exec(open(r"<repo>/packages/blacknode-isaac/clients/isaac_policy_client.py").read())
start_blacknode_isaac_policy(
    "http://127.0.0.1:8770",
    "/World/so_arm101",
    {"front": "/World/front_camera"},
)
```

If policy joint names differ from USD DOF names, pass an explicit mapping:

```python
start_blacknode_isaac_policy(
    "http://127.0.0.1:8770",
    "/World/so_arm101",
    {"front": "/World/front_camera", "wrist": "/World/wrist_camera"},
    joint_mapping={"shoulder_pan": "Rotation", "gripper": "GripperJoint"},
    workspace_prim_path="/World/so_arm101/end_effector",
)
```

`workspace_prim_path` is optional. Provide it when `workspace_limits` are
configured so the client includes live world-space x/y/z telemetry.

6. Confirm the bridge reports `connected` and fresh observations.
7. Set `IsaacPolicyRuntime.action=check` and resolve joint, camera, or USD-limit
   mismatches.
8. Set it to `start`, cook once, then return it to `status`. This starts
   continuous inference with simulation motion disarmed.
9. Inspect predictions, action scale, inference time, and the Isaac stage.
10. Set the action to `arm` in a separate cook. The first target synchronizes to
    the measured pose; subsequent targets pass through USD joint limits,
    velocity, per-step, workspace, and freshness checks.

Use `disarm` for a normal stop, `estop` for a latched emergency stop, and
`takeover` to suppress policy commands while manually changing the scene.
Reset actions return to disarmed preview and never re-arm automatically.
`stop`, **Stop All**, server shutdown, stale observations, and inference faults
suppress future articulation targets.

The runtime writes the same `.blacknode/policy-runs/<run_id>.jsonl` decisions,
metrics, clamps, faults, and control events used by physical deployment.

## Bridge contract

The Isaac client posts `blacknode.isaac-observation` values containing exact
policy joint order, measured positions in radians, USD limits, named JPEG RGB
sensors, and optional workspace coordinates. The response is a
`blacknode.isaac-command` with the gated target, sequence, and explicit armed
state. The client ignores commands unless `armed=true` and clamps accepted
targets against the USD limits again before applying them.

## Verification boundary

The package tests use a real loopback HTTP bridge, synthetic JPEG frames,
synthetic articulation state, and the shared Blacknode policy safety engine.
They verify disarmed preview, first-command pose synchronization, safety clamps,
emergency stop, and command suppression. Running an actual USD stage, renderer,
or GPU requires operator-led validation inside Isaac Sim.
