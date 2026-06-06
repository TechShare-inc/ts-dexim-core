# Vendor-Node Mixin Pattern

> **Status**: Reference Design  
> **Created**: 2026-03-12  
> **Scope**: All vendor node packages — `dexim-nova`, `dexim-inspire`, `dexim-dh5`, `dexim-g1`, `dexim-realsense`  
> **Worked example**: `ManusNode` in `dexim-manus`

---

## 1. Problem Statement

The historical inheritance chain for vendor nodes was:

```
ManagedNode → HardwarePublisherNode → TeleopNode → HandTeleopNode → VendorNode
```

This creates compounding problems as nodes grow:

| Problem                 | Symptom                                                                          |
| ----------------------- | -------------------------------------------------------------------------------- |
| Cross-cutting state     | Adding a metric counter or a ZMQ socket touches 3 base classes                   |
| Untestable in isolation | `HandTeleopNode` cannot be tested without instantiating a full `ManagedNode`     |
| Feature entanglement    | Switching from TCP to serial requires understanding the entire inheritance chain |
| Opaque `super()` chains | Lifecycle hooks (`on_start`, `on_shutdown`) silently skip intermediate classes   |

**Target**: Replace the chain with a **flat composition** model — one thin `ManagedNode` plus `N` single-responsibility mixins.

---

## 2. Pattern Overview

```
┌───────────────────────────────────────────────────────────────┐
│                     VendorNode (final class)                  │
│                                                               │
│  Base:  ManagedNode       (ZMQ control plane, lifecycle FSM)  │
│                                                               │
│  Mixins (each owns one responsibility):                       │
│  ┌─────────────┐ ┌──────────────┐ ┌──────────────────┐       │
│  │ PublisherMx │ │ MetricsMixin │ │ DeviceClientMixin│  ...  │
│  └─────────────┘ └──────────────┘ └──────────────────┘       │
│  ┌──────────────────┐ ┌────────────────┐                      │
│  │ DataSourceMixin  │ │ CalibrationMx  │  (device-specific)   │
│  └──────────────────┘ └────────────────┘                      │
│                                                               │
│  Contract: VendorNodeProtocol  (TYPE_CHECKING only)           │
│  Config:   VendorNodeConfig    (aggregated dataclasses)       │
└───────────────────────────────────────────────────────────────┘
```

### Key principles

- **One responsibility per mixin** — each mixin owns exactly one set of attributes and one namespace of methods.
- **Composition over inheritance** — `VendorNode` MRO is shallow: `PublisherMixin, ManagedNode, OtherMixin, …`.
- **Co-operative `super()`** — lifecycle hooks (`on_start`, `on_shutdown`, `_main_loop_iteration`) call `super()` at every level. `PublisherMixin` is listed _before_ `ManagedNode` in the class definition so its overrides are resolved first in MRO.
- **No cross-mixin attribute access without the Protocol** — mixins that read each other's state annotate `self` with `VendorNodeProtocol` (see §4).

### MRO declaration pattern

```python
class VendorNode(
    VendorPublisherMixin,   # first — overrides lifecycle hooks before ManagedNode abstract declarations
    ManagedNode,            # ZMQ control plane
    DeviceClientMixin,
    CalibrationMixin,       # optional — only if hardware requires world-frame rebasing
    DataSourceMixinA,
    DataSourceMixinB,
    MetricsMixin,
):
    ...
```

### `__init__` setup order

Always initialise mixins in **dependency order**:

```python
def __init__(self, node_id: str, config: VendorNodeConfig) -> None:
    super().__init__(node_id=node_id, ...)
    self.config = config

    self.setup_publisher(...)   # ZMQ sockets first (other mixins may log via status endpoint)
    self.setup_metrics(...)     # counters + intervals
    self.setup_client(...)      # hardware connection (may raise — abort-safe)
    self.setup_calibration(...) # depends on client being live (optional)
```

---

## 3. Anatomy of a Vendor Node

The table below defines the canonical mixin roles. Not every vendor requires all of them.

| Mixin role        | Typical class name       | Owns                                                                                                                                                                 | Required                           |
| ----------------- | ------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------- |
| **Publisher**     | `{Device}PublisherMixin` | ZMQ PUB socket, topic bytes, change-detection digest cache, rate-limiter, `_main_loop_iteration`, lifecycle hooks (`on_start`, `on_pause`, `on_stop`, `on_shutdown`) | Yes                                |
| **Metrics**       | `MetricsMixin`           | Uptime counter, error count, publish success/skip counters, rate-limited log lines, sensor landscape tracking                                                        | Yes                                |
| **Device client** | `{Device}ClientMixin`    | SDK object lifecycle (`initialize`, `connect`, `shutdown`), periodic connection health check                                                                         | Yes (unless device has no SDK)     |
| **Calibration**   | `CalibrationMixin`       | World-frame 4×4 transform, JSON file loader, identity-fallback                                                                                                       | Only if spatial rebasing is needed |
| **Data source A** | `{DataType}Mixin`        | One `_get_{data_type}_packet()` method returning `(topic, payload) \| None`                                                                                          | Device-specific, one per stream    |
| **Data source B** | `{DataType}Mixin`        | Same pattern — second stream                                                                                                                                         | Device-specific                    |

### What a data-source mixin must do

```
1. Guard: if self._client is None → log error, increment _error_count, return None
2. Poll the SDK for new data
3. If no data → return None
4. Build a typed message object (from dexim.core.messages)
5. Call self._has_changed(topic, payload) — if unchanged → increment _publish_skip_count, return None
6. Increment _publish_success_count, update _last_publish_time[topic]
7. Return (topic, payload)
```

This keeps publishing logic (ZMQ send, rate limit) isolated in the Publisher mixin while data assembly stays in the data-source mixin.

---

## 4. Cross-Mixin Typing: The Protocol Pattern

Mixins that access attributes from _other_ mixins via `self` would cause `attr-defined` type errors without some contract. The solution is a `VendorNodeProtocol` — a `typing.Protocol` that declares the full combined interface of `VendorNode`.

### Why a Protocol and not a common base class?

A shared base class would re-introduce coupling at import time and break the single-responsibility boundary. A `Protocol` is purely a static-analysis construct — zero runtime overhead.

### Usage in a mixin

```python
# In any mixin file:
from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from dexim.{vendor}.node.mixins._protocol import {Vendor}NodeProtocol


class SomeMixin:
    def my_method(self: {Vendor}NodeProtocol, ...) -> ...:
        # Self is typed as the full node; cross-mixin access type-checks cleanly.
        self._client.is_connected   # from ClientMixin
        self._error_count += 1      # from MetricsMixin
        ...
```

The `if TYPE_CHECKING` guard means the `_protocol` module is **never imported at runtime** — it only exists for `mypy` and `pyright`.

### Protocol file structure

`_protocol.py` is organised into labelled sections, one per source class:

```python
class {Vendor}NodeProtocol(Protocol):
    """Protocol describing the combined {Vendor}Node interface."""

    # ---- ManagedNode --------------------------------------------------
    node_id: str
    is_recording: bool
    ...
    def run(self) -> None: ...

    # ---- MetricsMixin -------------------------------------------------
    _error_count: int
    ...
    def setup_metrics(self: {Vendor}NodeProtocol, config: MetricsConfig) -> None: ...

    # ---- {Vendor}PublisherMixin ---------------------------------------
    _pub_data: zmq.Socket | None
    ...

    # ---- {Device}ClientMixin -----------------------------------------
    _client: {SDK}Client | None
    ...

    # ---- CalibrationMixin --------------------------------------------
    _wM_base: np.ndarray
    ...

    # ---- DataSourceMixinA --------------------------------------------
    def _get_{data_type}_packet(self: {Vendor}NodeProtocol) -> tuple[bytes, Any] | None: ...
```

All type-annotated imports that reference external packages (`numpy`, `zmq`, `dexim.*`, the vendor SDK) live inside the Protocol file's own `if TYPE_CHECKING:` block.

---

## 5. Auto-Generating the Protocol

Maintaining `_protocol.py` by hand is error-prone. Each vendor package **should include a `scripts/gen_protocol.py`** that auto-generates the Protocol from the actual mixin source files using Python's `ast` module.

### What the generator does

1. Parses each mixin file (and `ManagedNode`) with `ast.parse`.
2. Collects class-level `AnnAssign` annotations and `__init__` `self.*` annotations.
3. Collects all non-dunder method signatures.
4. Deduplicates globally across sections (a name appearing in an earlier section is skipped in later ones).
5. Resolves imports: moves heavy/external modules (`numpy`, `zmq`, vendor SDK) behind `TYPE_CHECKING`; keeps stdlib (`pathlib`, `typing`) as direct imports.
6. Writes the result to `src/dexim/{vendor}/node/mixins/_protocol.py`.

### When to regenerate

- After adding or removing an attribute declaration in any mixin.
- After changing a method signature that another mixin depends on.
- After adding a new mixin.

**Regeneration command** (run from the vendor package root):

```bash
python scripts/gen_protocol.py
```

Dry-run (prints to stdout without writing):

```bash
python scripts/gen_protocol.py --dry-run
```

### Adapting the generator for a new vendor

1. Copy `packages/dexim-manus/scripts/gen_protocol.py` into the new package.
2. Update the `_SOURCES` list at the top — each entry is `(filepath, ClassName, "SectionLabel")`.
3. Update `_DEFAULT_OUTPUT` to point at the new package's `_protocol.py`.
4. Update the generated class and module docstring references (`ManusNodeProtocol` → `{Vendor}NodeProtocol`).

---

## 6. Config Hierarchy Pattern

Each mixin sub-system gets its own `@dataclass` config. The top-level `VendorNodeConfig` aggregates all sub-configs.

### Example hierarchy

```
VendorNodeConfig
├── data_endpoint: str           # ZMQ endpoints (top-level, not nested)
├── control_endpoint: str
├── status_endpoint: str
├── rate_hz: float
├── bind_data: bool
├── client: ClientConfig         # sub-config per mixin
├── calibration: CalibrationConfig
├── publisher: PublisherConfig
└── metrics: MetricsConfig
```

### Sub-config rules

- Each sub-config is a standalone `@dataclass` with `__post_init__` validation.
- Values that are `Path` objects must be normalised: `self.some_path = Path(self.some_path)`.
- Mutually-exclusive or range-bounded fields raise `ValueError` in `__post_init__`.

### Top-level config normalisation

`VendorNodeConfig.__post_init__` converts plain `dict` entries to their dataclass types. This allows YAML-loaded configs to pass through without a separate parsing step:

```python
def __post_init__(self) -> None:
    if isinstance(self.client, dict):
        self.client = ClientConfig(**self.client)
    if isinstance(self.calibration, dict):
        self.calibration = CalibrationConfig(**self.calibration)
    ...
```

### YAML loading with deep-merge

Provide a `load_config(path: str | Path) -> VendorNodeConfig` function that:

1. Serialises `VendorNodeConfig()` defaults to a dict.
2. Deep-merges the YAML content on top.
3. Constructs `VendorNodeConfig(**merged)` — `__post_init__` handles nested dict→dataclass conversion.

This means YAML files only need to specify the keys they override.

---

## 7. Worked Example: ManusNode

### Directory layout

```
packages/dexim-manus/
  scripts/
    gen_protocol.py             # Protocol auto-generator
  src/dexim/manus/
    node/
      __init__.py
      config.py                 # ClientConfig, CalibrationConfig, PublisherConfig,
                                #   MetricsConfig, ManusNodeConfig, load_config
      node.py                   # ManusNode — thin orchestrator (~100 lines)
      mixins/
        __init__.py             # Re-exports all mixins
        _protocol.py            # AUTO-GENERATED — ManusNodeProtocol
        client.py               # ClientMixin
        calibration.py          # CalibrationMixin
        publisher.py            # ManusPublisherMixin
        metrics.py              # MetricsMixin
        skeleton.py             # SkeletonMixin
        tracker.py              # TrackerMixin
```

### Mixin responsibility table

| Mixin                 | Attributes owned                                                                                                                                                     | Methods exposed                                                                                                                           | Cross-mixin dependencies                                                                                                                                                                                                                |
| --------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `ManusPublisherMixin` | `_pub_data`, `_tracker_topic`, `_skeleton_topic`, `_last_digest`, `_last_publish_time`, `_publish_success_count`, `_publish_skip_count`, `_rate_hz`, `_next_tick_ts` | `setup_publisher()`, `_send()`, `pack_message()`, `_has_changed()`, `reset_publish_counters()`, lifecycle hooks, `_main_loop_iteration()` | `get_data()` (ManusNode)                                                                                                                                                                                                                |
| `MetricsMixin`        | `_start_time`, `_error_count`, `_last_error_time`, `_last_landscape`, `_metrics_log_interval`, `_landscape_log_interval`, `_last_metrics_log`, `_last_landscape_log` | `setup_metrics()`, `reset_metrics()`, `_maybe_log_metrics()`, `_update_landscape()`, `get_debug_state()`                                  | `_publish_success_count`, `_publish_skip_count` (Publisher); `node_id`, `config` (ManusNode); `_client`, `_using_identity_transform`, `_wM_base` (Client, Calibration)                                                                  |
| `ClientMixin`         | `_client`, `_connection_check_interval`, `_last_connection_check`                                                                                                    | `setup_client()`, `cleanup_client()`, `_maybe_check_connection()`, `_check_connection_health()`                                           | `report_status()` (ManagedNode); `_error_count` (Metrics)                                                                                                                                                                               |
| `CalibrationMixin`    | `_calibration_dir`, `_wM_base`, `_wM_base_transform`, `_using_identity_transform`, `_calibration_file`                                                               | `setup_calibration()`, `_reset_to_identity()`, `_load_calibration()`                                                                      | None                                                                                                                                                                                                                                    |
| `SkeletonMixin`       | _(none — all borrowed via Protocol)_                                                                                                                                 | `_get_skeleton_packet()`                                                                                                                  | `_client` (Client); `_skeleton_topic`, `_has_changed()`, `_publish_success_count`, `_publish_skip_count`, `_last_publish_time` (Publisher); `_error_count`, `_last_error_time` (Metrics)                                                |
| `TrackerMixin`        | _(none — all borrowed via Protocol)_                                                                                                                                 | `_get_tracker_packet()`                                                                                                                   | `_client` (Client); `_wM_base`, `_wM_base_transform` (Calibration); `_tracker_topic`, `_has_changed()`, `_publish_success_count`, `_publish_skip_count`, `_last_publish_time` (Publisher); `_error_count`, `_last_error_time` (Metrics) |

### Config table

| Dataclass           | Key fields                                                                                        | Default                         |
| ------------------- | ------------------------------------------------------------------------------------------------- | ------------------------------- |
| `ClientConfig`      | `connection_check_interval: float`                                                                | `5.0 s`                         |
| `CalibrationConfig` | `calibration_dir: Path`, `skip: bool`                                                             | `Path("calibrations")`, `False` |
| `PublisherConfig`   | `publish_trackers: bool`, `publish_skeletons: bool`, `cache_ttl: float`                           | `True`, `True`, `0.1 s`         |
| `MetricsConfig`     | `metrics_log_interval: float`, `landscape_log_interval: float`                                    | `10.0 s`, `60.0 s`              |
| `ManusNodeConfig`   | `data_endpoint`, `control_endpoint`, `status_endpoint`, `rate_hz`, `bind_data`, + all sub-configs | port 5571, 60 Hz                |

### ManusNode MRO and data flow

```python
class ManusNode(
    ManusPublisherMixin,  # lifecycle hooks resolved first
    ManagedNode,
    ClientMixin,
    CalibrationMixin,
    TrackerMixin,
    SkeletonMixin,
    MetricsMixin,
):
    ...
```

`get_data()` pipeline per frame:

```
_maybe_check_connection(now)          → ClientMixin     — periodic health check
_get_tracker_packet()                 → TrackerMixin    → (topic, [RigidPose, ...]) | None
_get_skeleton_packet()                → SkeletonMixin   → (topic, [HandState, ...]) | None
_maybe_log_metrics(now)               → MetricsMixin    — rate-limited log line
_update_landscape(tracker_types, n)   → MetricsMixin    — topology change detection
```

The `_main_loop_iteration` (in `ManusPublisherMixin`) calls `get_data()` and dispatches each `(topic, payload)` pair to `_send()`.

### Regenerating the Protocol

```bash
# From packages/dexim-manus/:
python scripts/gen_protocol.py
```

---

## 8. Applying to Other Vendors

### Common mixins (reuse across all vendors)

| Mixin              | Source                   | Notes                                                                                                            |
| ------------------ | ------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| `MetricsMixin`     | Adapt from `dexim-manus` | Remove Manus-specific `_update_landscape` landscape fields if not needed; keep counters and log-interval pattern |
| `CalibrationMixin` | Copy from `dexim-manus`  | Reuse verbatim if the device needs world-frame rebasing; omit otherwise                                          |

### dexim-nova (Dobot Nova arm)

Nova publishes joint states and receives joint commands. Suggested mixin split:

| Mixin                 | Responsibility                                                              |
| --------------------- | --------------------------------------------------------------------------- |
| `NovaPublisherMixin`  | ZMQ PUB for `JointState` observations; rate-limiter; `_main_loop_iteration` |
| `NovaSubscriberMixin` | ZMQ SUB for downstream `JointCommand` targets (if closed-loop)              |
| `NovaClientMixin`     | `DobotNovaClient` (or underlying TCP API) lifecycle                         |
| `JointStateMixin`     | Poll arm joint positions/velocities; assemble `JointState` packet           |
| `MetricsMixin`        | Reuse pattern from Manus                                                    |

No `CalibrationMixin` needed — arm FK is in `dexim.nova.model`, not in the node.

### dexim-inspire (Inspire hand)

Inspire is a _consumer_ of Manus glove data and a _controller_ of finger joints. The inspire-architecture-redesign.md documents a parallel mixin split. Key differences from Manus:

| Mixin                    | Responsibility                                                     |
| ------------------------ | ------------------------------------------------------------------ |
| `PublisherMixin`         | Publish `JointState` observations and `JointCommand` actions       |
| `SubscriberMixin`        | Subscribe to `HandState` from `dexim-manus`                        |
| `ControlLoopMixin`       | Rate limiting, safe-position interpolation, data-timeout detection |
| `FeatureExtractionMixin` | `HandState` → finger direction vectors                             |
| `RetargetingMixin`       | Finger vectors → `JointCommand` via `VectorOptimizer`              |
| `FilteringMixin`         | `WeightedMovingFilter` smoothing on joint angles                   |
| `VisualizationMixin`     | Viser 3D display; always active in all modes                       |

No `CalibrationMixin` (world-frame calibration lives in the Manus node, not here). No `DeviceClientMixin` — the serial/DDS interface is injected via `factory.py`, not owned by a mixin.

### dexim-dh5 (DH5 hand)

DH5 is structurally identical to Inspire — same data flow (Manus → retargeting → hand actuator). Copy the Inspire mixin layout; replace `InspireInterface` with `DH5Interface` and update the retargeting config.

Differences to note:

- DH5 uses Modbus RTU (pymodbus) rather than TCP — `DH5ClientMixin` wraps a `ModbusClient` lifecycle.
- Joint count and limits differ — `RetargetingMixin` config is device-specific.

### dexim-g1 (Unitree G1 humanoid)

G1 is a full bimanual + locomotion robot. The node is more complex:

| Mixin               | Responsibility                                           |
| ------------------- | -------------------------------------------------------- |
| `G1PublisherMixin`  | ZMQ PUB for head, arm, hand, and locomotion joint states |
| `G1SubscriberMixin` | ZMQ SUB for teleoperation commands (head, arms, hands)   |
| `G1ClientMixin`     | Unitree DDS session lifecycle                            |
| `ArmStateMixin`     | Poll arm joint positions; assemble `JointState`          |
| `HandStateMixin`    | Poll dexterous hand joints                               |
| `LocomotionMixin`   | Foot-contact, base velocity state                        |
| `MetricsMixin`      | Reuse pattern                                            |

`CalibrationMixin` may be needed for world-frame IMU → global transform rebasing on the base.

### dexim-realsense (Intel RealSense camera)

RealSense is a pure _publisher_ node — no external device client SDK per se (pyrealsense2 replaces a custom client). Suggested split:

| Mixin                     | Responsibility                                               |
| ------------------------- | ------------------------------------------------------------ |
| `RealSensePublisherMixin` | ZMQ PUB for colour and depth frames; rate-limiter            |
| `RealSensePipelineMixin`  | `rs.pipeline` lifecycle (`start`, `stop`, `wait_for_frames`) |
| `ColourFrameMixin`        | Convert colour frame → numpy array → ZMQ payload             |
| `DepthFrameMixin`         | Convert depth frame → numpy array → ZMQ payload              |
| `MetricsMixin`            | Reuse pattern — track frame drop rate                        |

No `CalibrationMixin` needed (intrinsics come from the device SDK and are published with each frame).

---

## 9. Step-by-Step Implementation Checklist

Use this checklist when adding a new vendor node.

### Step 1 — Create mixin files

- [ ] Identify the device's data streams — one `DataSourceMixin` per stream.
- [ ] Create `src/dexim/{vendor}/node/mixins/` directory.
- [ ] Write `{Device}PublisherMixin` — ZMQ PUB socket, topics, change-detection, lifecycle hooks, `_main_loop_iteration`.
- [ ] Write `MetricsMixin` (adapt from Manus — adjust landscape-tracking fields to match device topology).
- [ ] Write `{Device}ClientMixin` — SDK `initialize` / `connect` / `shutdown` / health check.
- [ ] Write `CalibrationMixin` if spatial rebasing is required (may copy verbatim from Manus).
- [ ] Write one `{DataType}Mixin` per data stream.
- [ ] Write `mixins/__init__.py` — re-export all mixins; no other logic.

### Step 2 — Create the Protocol

- [ ] Copy `gen_protocol.py` from `dexim-manus/scripts/` into the new package's `scripts/`.
- [ ] Update `_SOURCES`, `_DEFAULT_OUTPUT`, and the generated class name.
- [ ] Run `python scripts/gen_protocol.py` from the package root — verify `_protocol.py` is written.
- [ ] Import `_protocol.py` in a throwaway `python -c "..."` to confirm no syntax errors.
- [ ] Update each mixin to annotate `self: {Vendor}NodeProtocol` on cross-mixin methods.

### Step 3 — Create config dataclasses

- [ ] Create `node/config.py`.
- [ ] One `@dataclass` per mixin with `__post_init__` validation.
- [ ] Aggregate into `{Vendor}NodeConfig` with dict→dataclass normalisation in `__post_init__`.
- [ ] Write `load_config(path) -> {Vendor}NodeConfig` using deep-merge YAML load.
- [ ] Add a `get_default_{vendor}_data_endpoint()` helper that reads from `NODE_DATA_PORTS`.

### Step 4 — Write the node

- [ ] Create `node/node.py` — target **<150 lines**.
- [ ] Declare MRO: `PublisherMixin` first, then `ManagedNode`, then remaining mixins.
- [ ] `__init__` calls `setup_*()` in dependency order (publisher → metrics → client → calibration).
- [ ] `get_data()` pipelines the data-source packets; hands results to `_main_loop_iteration` via return value.
- [ ] Implement `on_start`, `on_shutdown` — call `super()` and mixin cleanup in shutdown.

### Step 5 — Wire up `__init__.py` exports

- [ ] `node/__init__.py` — export `{Vendor}Node`, `{Vendor}NodeConfig`, `load_config`.
- [ ] `{vendor}/__init__.py` — surface node and config at the package level.

### Step 6 — Write tests

- [ ] Unit-test each mixin in isolation using `unittest.mock.MagicMock` for cross-mixin state.
- [ ] Mock-test: instantiate `{Vendor}Node` with a mock SDK client; assert `get_data()` round-trips.
- [ ] Hardware integration test (optional, guarded behind a `hardware` marker).

### Step 7 — Regenerate Protocol after any signature change

- [ ] `python scripts/gen_protocol.py` — run this any time a mixin attribute or method changes.
- [ ] Add this as a step in CI (dry-run + diff assertion) to catch stale Protocol files.

---

## 10. Naming Conventions

| Artifact                    | Pattern                                                    | Example                                              |
| --------------------------- | ---------------------------------------------------------- | ---------------------------------------------------- |
| Mixin class                 | `{Device}{Role}Mixin`                                      | `ManusPublisherMixin`, `NovaClientMixin`             |
| Generic role mixin (reused) | `{Role}Mixin`                                              | `MetricsMixin`, `CalibrationMixin`                   |
| Protocol class              | `{Vendor}NodeProtocol`                                     | `ManusNodeProtocol`, `NovaNodeProtocol`              |
| Protocol file               | `_protocol.py` (underscore prefix = auto-generated)        | `mixins/_protocol.py`                                |
| Node class                  | `{Vendor}Node` or `{Device}ControlNode`                    | `ManusNode`, `InspireControlNode`                    |
| Config aggregate            | `{Vendor}NodeConfig`                                       | `ManusNodeConfig`, `InspireNodeConfig`               |
| Sub-config                  | `{Role}Config`                                             | `ClientConfig`, `CalibrationConfig`, `MetricsConfig` |
| Setup method                | `setup_{role}(self, config: {Role}Config)`                 | `setup_client(...)`, `setup_metrics(...)`            |
| Cleanup method              | `cleanup_{role}(self)`                                     | `cleanup_client()`, `cleanup_publisher()`            |
| Data-source method          | `_get_{data_type}_packet(self) → (topic, payload) \| None` | `_get_tracker_packet()`, `_get_skeleton_packet()`    |
| Generator script            | `scripts/gen_protocol.py`                                  | —                                                    |

### What NOT to name things

- Do **not** use the old prefixes `core-`, `ts-`, `shared-` in any new class or module name.
- Do **not** name a mixin class `*Node` — nodes are the final composed class only.
- Do **not** name a sub-config `{Vendor}Config` — that pattern is reserved for the top-level aggregator.
- Do **not** create `api/` submodules — vendor SDK imports live directly in the `ClientMixin` or interface files.

---

## Appendix: Related Design Documents

| Document                                                                                  | Description                                                                                  |
| ----------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------- |
| [`inspire-architecture-redesign.md`](inspire-architecture-redesign.md)                    | Phase-by-phase migration plan for `dexim-inspire` from deep inheritance to mixin composition |
| [`cli-framework.md`](cli-framework.md)                                                    | CLI entry-point design for `dexim-manus`, `dexim-nova`, `dexim-inspire` umbrella             |
| [`architecture.instructions.md`](../../.github/instructions/architecture.instructions.md) | Repo-wide naming conventions, package layout rules, dependency strategy                      |
