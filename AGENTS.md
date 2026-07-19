# blacknode-isaac Agent Instructions

This is an independent Blacknode extension-package repository. Check and
commit its Git state separately from the containing Blacknode checkout.

## Scope

Own the direct Isaac Sim policy bridge, simulated sensor/articulation adapter,
managed closed-loop policy execution, and Isaac deployment templates. Reuse
Blacknode policy artifacts and the shared safety/runtime contract. Dataset
recording and offline replay remain in `blacknode-dataset`; physical robot and
ROS transports remain in their respective packages.

## Rules

- Load and test without Isaac Sim installed. Isaac imports belong inside the
  client functions that run in Isaac's Python environment.
- Bind the policy bridge to loopback only. Do not expose unauthenticated
  articulation control on a network interface.
- Start the bridge and policy runtime disarmed. Only an explicit `arm` action
  permits target application.
- Preserve exact policy joint and camera ordering. Reject missing sensors,
  invalid USD limits, stale observations, and non-finite values.
- Apply policy limits in Blacknode and clamp again to USD articulation limits
  in the Isaac client.
- Emergency stop, takeover, faults, runtime stop, and server shutdown must
  suppress future articulation commands.
- Tests use synthetic RGB frames and articulation state only. Never claim an
  Isaac stage or GPU renderer was tested without running inside Isaac Sim.

## Verification

From the Blacknode root:

```powershell
$env:PYTHONPATH="python"
python -m pytest packages/blacknode-isaac/tests
python -m blacknode.cli validate packages/blacknode-isaac/templates/isaac-act-policy-deployment.json
```
