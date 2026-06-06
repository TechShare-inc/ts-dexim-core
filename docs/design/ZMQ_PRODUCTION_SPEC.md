# ZMQ Publisher & Subscriber — Production Implementation Spec

This document describes the actual production ZMQ architecture as implemented in the
`core-node-framework`, `manus-subscriber`, and related packages. It is derived
directly from the source code and supersedes `zmq_spec.md` where the two conflict.

---

## 1. Three-Plane Architecture

Every process in this system uses three independent ZMQ communication planes.
The orchestrator owns the server side; all `ManagedNode` subclasses own the client side.

```
         TeleopOrchestrator (server)
         PUB  tcp://0.0.0.0:5550   ──────────────────────────────────────────►
         PULL tcp://0.0.0.0:5551   ◄──────────────────────────────────────────

         ManagedNode (every node, client side)
         SUB  connect tcp://localhost:5550   ◄─── Control Plane (commands)
         PUSH connect tcp://localhost:5551   ───► Status Plane  (heartbeats)
         [+ Data PUB or Data SUB per subclass]     Data Plane
```

| Plane       | ZMQ Pattern | Serialization | Frame count | Direction                   |
|-------------|-------------|---------------|-------------|-----------------------------|
| **Control** | PUB/SUB     | Raw UTF-8     | 2           | Orchestrator → all nodes    |
| **Status**  | PUSH/PULL   | JSON          | 1           | Each node → Orchestrator    |
| **Data**    | PUB/SUB     | msgpack       | 2           | Publisher node → subscriber nodes |

---

## 2. ZMQ Context Policy

All nodes use `zmq.Context.instance()` — the global shared singleton.
No node creates a private `zmq.Context()` independently.

**Exception**: `ManusSubscriber` creates a fresh `zmq.Context()` if no external
context is passed to its constructor. Callers that need the shared context must
pass it explicitly:

```python
ManusSubscriber(address="tcp://localhost:5555", context=zmq.Context.instance())
```

**Context is never terminated.** `_cleanup_zmq()` in both `ManagedNode` and
`TeleopOrchestrator` explicitly skips `ctx.term()` to avoid disrupting other sockets
that share the same context within the same process.

---

## 3. Control Plane

### 3.1 Orchestrator — Binding Side

Source: `packages/core-node-framework/core_node_framework/orchestrator.py`

```python
self._pub_control = ctx.socket(zmq.PUB)
bind_addr = control_endpoint.replace("localhost", "0.0.0.0")
self._pub_control.bind(bind_addr)      # always binds to 0.0.0.0, never *

self._pull_status = ctx.socket(zmq.PULL)
self._pull_status.bind(status_endpoint.replace("localhost", "0.0.0.0"))

time.sleep(0.2)   # mandatory settle delay after binding
```

The orchestrator replaces `localhost` → `0.0.0.0` and `*` → `0.0.0.0` before
binding. This is intentional: `0.0.0.0` is explicit and unambiguous about
accepting connections from all interfaces.

**Sending a command:**

```python
self._pub_control.send_multipart(
    [TOPIC_CTRL, command.encode("utf-8")],
    flags=zmq.DONTWAIT
)
```

**Draining all queued status messages:**

```python
events = dict(self._poller.poll(timeout=timeout_ms))
while self._pull_status in events:
    payload = self._pull_status.recv(flags=zmq.NOBLOCK)
    msg = json.loads(payload.decode("utf-8"))
    # update node_statuses ...
    events = dict(self._poller.poll(timeout=0))   # check for more
```

### 3.2 Node — Connecting Side

Source: `packages/core-node-framework/core_node_framework/managed.py`

```python
ctx = zmq.Context.instance()

sub = ctx.socket(zmq.SUB)
sub.connect(self._control_endpoint)           # tcp://localhost:5550
sub.setsockopt(zmq.SUBSCRIBE, TOPIC_CTRL)     # b"CTRL"

push = ctx.socket(zmq.PUSH)
push.connect(self._status_endpoint)           # tcp://localhost:5551

poller = zmq.Poller()
poller.register(sub, zmq.POLLIN)
```

No HWM is set on either control-plane socket. ZMQ default (1000) applies.

**Main loop timing:**

```python
while self.running:
    self._poll_once(timeout_ms=5)          # 5 ms poll on control SUB
    self._main_loop_iteration()            # subclass work
    self.send_heartbeat_if_needed()        # every 1.0 s
```

**Receiving a control message:**

```python
parts = self._sub_control.recv_multipart(flags=zmq.NOBLOCK)
topic, command = parts[0], parts[1]
cmd = command.decode("utf-8", errors="ignore").strip().upper()
```

**Sending status:**

```python
payload = pack_status_message(
    node_id, status, is_recording, time.time(),
    {"is_publishing": self.is_publishing, **(info or {})}
)
self._push_status.send(payload, flags=zmq.DONTWAIT)  # non-blocking, drop on fail
```

**Socket teardown — always with `linger=0`:**

```python
self._sub_control.close(linger=0)
self._push_status.close(linger=0)
# context is NOT terminated
```

---

## 4. Control Plane Wire Format

### 4.1 Command Message — 2 Frames

```
Frame 1: b"CTRL"
Frame 2: command string as UTF-8 bytes   e.g.  b"START"
```

### 4.2 Command Reference

| Command      | `_teleop_active` | Notes |
|--------------|-----------------|-------|
| `START`      | → `True`        | `on_start()` fires **before** flag is set; `is_publishing` → `True` |
| `PAUSE`      | → `False`       | `on_pause()` fires after flag clears |
| `STOP`       | → `False`       | `on_stop()` fires after flag clears; node moves to safe position |
| `SHUTDOWN`   | unchanged       | `running = False`; `on_shutdown()` fires in `finally` |
| `START_PUB`  | unchanged       | `is_publishing` → `True` |
| `PAUSE_PUB`  | unchanged       | `is_publishing` → `False` |
| `STOP_PUB`   | unchanged       | `is_publishing` → `False` |
| `START_REC`  | unchanged       | `is_recording` → `True`; `on_start_recording()` fires |
| `STOP_REC`   | unchanged       | `is_recording` → `False`; `on_stop_recording()` fires |

> **Critical ordering in `START`**: `on_start()` is called before
> `self._teleop_active = True`. This guarantees that reference-pose capture
> (home configuration, tracker origin) happens before any control iteration
> reads the `_teleop_active=True` flag.

### 4.3 Status Message — 1 Frame (JSON)

```json
{
  "node_id":      "nova_left",
  "status":       "HEALTHY",
  "is_recording": false,
  "timestamp":    1742000000.123,
  "info":         {"is_publishing": true}
}
```

Valid status strings: `INITIALIZED`, `STARTED`, `PAUSED`, `HEALTHY`, `ERROR`,
`SHUTTING_DOWN`.

Node health check: `time.time() - last_heartbeat > 5.0` → node considered unhealthy.

---

## 5. Data Plane

### 5.1 Publisher — `HardwarePublisherNode`

Source: `packages/core-node-framework/core_node_framework/hardware_publisher.py`

```python
pub = ctx.socket(zmq.PUB)
pub.setsockopt(zmq.LINGER, 0)    # drop unsent messages on close
if self._bind:
    pub.bind(data_endpoint)      # default: bind=True
else:
    pub.connect(data_endpoint)
```

**Publishing:**

```python
def _send(self, topic: bytes, data: Any) -> None:
    topic_frame, payload_frame = self.pack_message(topic, data)
    self._pub_data.send_multipart([topic_frame, payload_frame], flags=zmq.DONTWAIT)
    # EAGAIN or any error is caught, logged as non-fatal, cycle continues
```

`pack_message()` is overridable. The default calls `pack_data_message(topic, time.time(), data)`.

**Subclass API:**

```python
def get_data(self) -> tuple[bytes, Any] | list[tuple[bytes, Any]] | None:
    # Return (topic, data)          → single message published
    # Return [(topic, data), ...]   → batch published
    # Return None                   → skip this cycle
```

### 5.2 Rate Limiter — Drift-Corrected

```python
# On each iteration:
if now < self._next_tick_ts:
    return   # not yet time

period = 1.0 / self._rate_hz
self._next_tick_ts += period              # absolute step, not now + period
if self._next_tick_ts < now:
    self._next_tick_ts = now + period     # catch up if far behind
```

Using `+= period` (not `= now + period`) means the scheduler self-corrects
for slow cycles rather than accumulating drift.

### 5.3 Subscriber — `ManusSubscriber`

Source: `packages/manus-subscriber/manus_subscriber/subscriber.py`

```python
self.socket = self.context.socket(zmq.SUB)
self.socket.setsockopt(zmq.RCVHWM, 0)        # unlimited receive buffer
self.socket.setsockopt(zmq.SUBSCRIBE, topic)  # e.g. b"obs_manus_trackers"
self.socket.connect(address)
```

**Why `RCVHWM=0`**: ZMQ's `CONFLATE` option (single-slot buffer) silently drops
frames within a multipart message, corrupting the `[topic, payload]` pair.
`RCVHWM=0` (unlimited buffer) combined with application-level draining via
`receive_latest()` achieves the same "always latest" behaviour without data
corruption.

### 5.4 `receive_latest()` — The Core Drain Pattern

```python
def receive_latest(self) -> Any:
    latest_frames = None
    message_count = 0

    while True:
        try:
            frames = self.socket.recv_multipart(flags=zmq.NOBLOCK)
            latest_frames = frames          # overwrite — discard previous
            message_count += 1
        except zmq.Again:
            break
        except Exception:
            break

    if message_count > 1:
        logger.info(
            f"Received {message_count} messages, "
            f"discarded {message_count - 1} older..."
        )

    if latest_frames:
        return self._decode_frames(latest_frames)
    return {}
```

The decode step runs **only once**, on the last set of frames. Intermediate
frames are read but not deserialised, keeping drain-loop overhead minimal.

### 5.5 `receive()` — Single Message with Optional Timeout

```python
def receive(self, timeout: int | None = None) -> Any:
    # timeout=None → NOBLOCK (returns {} immediately if no message)
    # timeout=1000 → wait up to 1000 ms
    try:
        flags = zmq.NOBLOCK if timeout is None else 0
        if timeout is not None:
            self.socket.setsockopt(zmq.RCVTIMEO, timeout)
        frames = self.socket.recv_multipart(flags=flags)
        return self._decode_frames(frames)
    except zmq.Again:
        return {}
    finally:
        if timeout is not None:
            self.socket.setsockopt(zmq.RCVTIMEO, -1)   # always reset
```

### 5.6 `read()` — High-Level Entry Point

```python
def read(self) -> dict[str, Any]:
    raw = self.receive_latest()
    if not raw:
        return {"skeletons": {}, "trackers": [], "raw": {}}

    parsed_skeletons, parsed_trackers = self._parse_raw_data(raw)
    # Auto-detects by first-element keys:
    #   "tracker_id" / "tracker_type"  → tracker path
    #   "glove_id"   / "nodes"         → skeleton path
    self._update_landscape_from_parsed(parsed_skeletons, parsed_trackers)
    return {"skeletons": parsed_skeletons, "trackers": parsed_trackers, "raw": raw}
```

Return structure used by all `TeleopNode` subclasses:

| Key         | Type                          | Description                         |
|-------------|-------------------------------|-------------------------------------|
| `skeletons` | `dict[int, ManusSkeletonData]`| Keyed by glove_id                   |
| `trackers`  | `list[TrackerData]`           | All visible trackers                |
| `raw`       | `list[dict]`                  | Original deserialized payload       |

---

## 6. Data Plane Wire Format

### 6.1 Frame Layout

```
Frame 1:  topic bytes            e.g.  b"obs_manus_trackers"
Frame 2:  msgpack({
    "topic":      str,           e.g.  "obs_manus_trackers"   (repeated)
    "data":       Any,           the payload — must be msgpack-serialisable
    "timestamp":  float          Unix epoch seconds  (NOT ISO string)
})
```

Source: `packages/shared_messages/src/shared_messages/serialization.py`

### 6.2 NumPy Array Policy

`pack_data_message()` has no numpy dependency. Callers **must** call `.tolist()`
on numpy arrays before packing, **except** in `perception_service` where
`msgpack_numpy.patch()` is applied at module level:

```python
# perception_service/main.py — only location where this is patched
import msgpack_numpy as mpnp
mpnp.patch()
```

All other services and nodes convert manually.

---

## 7. Data Plane Topics and Ports

### 7.1 Topic Registry

Source: `packages/shared_messages/src/shared_messages/constants.py`

| Constant                     | Bytes value                  | Publisher          | Port | Payload shape |
|------------------------------|------------------------------|--------------------|------|---------------|
| `TOPIC_CTRL`                 | `b"CTRL"`                    | `TeleopOrchestrator` | 5550 | — |
| `TOPIC_MANUS_TRACKERS`       | `b"obs_manus_trackers"`      | `ManusNode`        | 5555 | `list[{tracker_id, tracker_type, position[3], rotation[4 wxyz], ...}]` |
| `TOPIC_MANUS_RAW_SKELETONS`  | `b"obs_manus_raw_skeletons"` | `ManusNode`        | 5555 | `list[{glove_id, nodes: list[{id, position[3], rotation[4], scale[3]}]}]` |
| `TOPIC_HAND_SKELETON`        | `b"act_hand_skeleton"`       | `HandTrackingNode` | 5556 | `HandSkeletonMessage` as dict |
| `TOPIC_IMAGE_CAM1`           | `b"obs_image_cam1"`          | `CameraNode`       | 5557 | raw image bytes |
| `TOPIC_IMAGE_CAM2`           | `b"obs_image_cam2"`          | `CameraNode`       | 5557 | raw image bytes |
| `TOPIC_DEPTH_CAM1`           | `b"obs_depth_cam1"`          | `CameraNode`       | 5557 | aligned depth bytes + intrinsics |
| `TOPIC_CAMERA_INFO`          | `b"obs_camera_info"`         | `CameraNode`       | 5557 | `CameraIntrinsics` as dict |
| `b"KEYBOARD_STATE"`          | `b"KEYBOARD_STATE"`          | `homie_app_service` inline | 5556 | `{x, y, yaw, height}` msgpack dict |

### 7.2 Port Registry

Source: `packages/core-utility/src/core_utility/ports.py`

| Port | Name           | Role |
|------|----------------|------|
| 5550 | control        | Orchestrator Control PUB |
| 5551 | status         | Orchestrator Status PULL |
| 5555 | manus          | ManusNode data PUB |
| 5556 | hand_tracking  | HandTrackingNode / Homie keyboard state |
| 5557 | camera         | CameraNode |
| 5558 | robot_state    | (generic) |
| 5559 | nova_left      | NovaControlNode left |
| 5560 | nova_right     | NovaControlNode right |
| 5561 | inspire_left   | |
| 5562 | inspire_right  | |
| 5563 | dh5_left       | |
| 5564 | dh5_right      | |
| 5565 | g1 / homie     | |
| 5566 | gripper_left   | |
| 5567 | gripper_right  | |
| 8080–8086 | sim interfaces | WebSocket/TCP simulation ports |
| 8765 | homie_recv     | `HomieCommandReceiver` ZMQ SUB (binds) |
| 8766 | homie_send     | `HomieCommandPublisher` raw TCP (connects) |

---

## 8. TeleopNode Control Loop

Source: `packages/core-node-framework/core_node_framework/teleop_node.py`

```python
def _main_loop_iteration(self) -> None:
    # One-time init on first call
    if not self._control_loop_initialized:
        self._initialize_control_loop()

    if not self._teleop_active:
        # CRITICAL: drain subscriber even when paused.
        # Prevents stale buffered frames from replaying on resume.
        try:
            self.subscriber.read()
        except Exception:
            pass
        self.rate_limiter.sleep()
        return

    data = self._get_data()

    if self._check_timeout():
        self.go_to_safe_position()
        self.last_data_time = time.time()
    else:
        if data:
            target_joints = self.process_data(data)
            if target_joints is not None:
                safe_joints = self._apply_velocity_limiting(target_joints)
                self._send_command(safe_joints)
                self.last_data_time = time.time()

    timing = self.rate_limiter.sleep()
    # logs rate stats every 100 iterations
```

### Key Behaviours

| Behaviour | Detail |
|-----------|--------|
| **Drain while paused** | `subscriber.read()` runs every iteration regardless of `_teleop_active`. Keeps the socket buffer empty so the first frame after `START` is fresh. |
| **Velocity limiting** | `_apply_velocity_limiting()` is currently **disabled** — passes through unchanged. Implementation is commented out with `# TODO: TEMPORARILY DISABLED FOR DEBUGGING`. |
| **Data timeout** | `time.time() - last_data_time > config.control.timeout_sec` triggers `go_to_safe_position()`. Configurable per node. |
| **Rate limiter** | `RateLimiter(rate_hz=...)` with drift correction; logs actual rate, jitter, overtime count every 100 iterations. |

### 8.1 Safe Position Movement

Uses Ken Perlin's improved smoothstep (5th-order) for smooth acceleration /
deceleration at the motion boundaries:

```python
def _move_to_position_safely(self, target_joints, timeout_sec=5.0, max_velocity_rad_s=None):
    delta = target_joints - current_joints
    num_steps = ceil(max_joint_delta / max_delta_per_step) + 5   # +5 safety margin
    for i in range(num_steps):
        t     = (i + 1) / num_steps
        alpha = t*t*t * (t*(t*6.0 - 15.0) + 10.0)   # smootherstep
        self._send_command(current_joints + alpha * delta)
        self.rate_limiter.sleep()
```

Velocity limit priority (highest wins):

1. `max_velocity_rad_s` argument
2. `config.control.safe_position_max_velocity_rad_s`
3. `config.control.max_joint_velocity_rad_s`

---

## 9. RecorderNode — Multi-Endpoint Subscriber

Source: `packages/core-node-framework/core_node_framework/recorder_node.py`

```python
# One SUB socket per endpoint; all share the same poller as the control SUB
for endpoint in self._data_endpoints:
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt(zmq.SUBSCRIBE, b"")      # subscribe to ALL topics
    self._poller.register(sub, zmq.POLLIN)  # same poller as control SUB
```

A single `zmq.Poller` covers all sockets (control + all data endpoints). One
`poller.poll(10 ms)` call fires for whichever socket has data first.

**Main loop:**

```python
events = dict(self._poller.poll(timeout=10))
for sock in self._sub_data_sockets:
    if sock in events:
        self._handle_data_message(sock)
```

**Message handling:**

```python
def _handle_data_message(self, sock):
    if not self.is_recording:
        sock.recv_multipart(flags=zmq.NOBLOCK)   # drain without buffering
        return
    frames = sock.recv_multipart(flags=zmq.NOBLOCK)
    msg    = unpack_data_message(frames)
    topic  = msg["topic"].encode("utf-8")
    with self._buffer_lock:
        self._buffers[topic].append((float(msg["timestamp"]), msg["data"]))
```

**Episode save on `STOP_REC`:**

```python
def on_stop_recording(self):
    with self._buffer_lock:
        episode_buffers = copy.deepcopy(self._buffers)
        self._buffers.clear()
    self._episode_counter += 1
    writer = self._create_episode_writer(episode_buffers, self._episode_counter)
    writer.start()   # background thread — non-blocking
```

Buffer structure: `dict[topic_bytes → list[(timestamp_float, data_any)]]`

---

## 10. CommandNode — Thread-Safe Command Publisher

Source: `packages/core-node-framework/core_node_framework/command_node.py`

```python
self._command_queue: Queue[str | None] = Queue()

def send_command(self, command: str) -> bool:
    """Thread-safe; can be called from any thread."""
    if not command or not command.strip():
        return False
    self._command_queue.put(command)
    return True

def _main_loop_iteration(self):
    if not self.is_publishing:
        time.sleep(0.01)
        return
    try:
        command = self._command_queue.get_nowait()
        if command is None:
            self.running = False   # None is the shutdown sentinel
            return
        self._publish_command(command)
    except Empty:
        pass
```

`_publish_command()` uses `pack_data_message(self._topic, time.time(), command)`
and `send_multipart(flags=zmq.DONTWAIT)`.

---

## 11. TeleopOrchestrator — Process Management

Source: `packages/core-node-framework/core_node_framework/orchestrator.py`

The base `TeleopOrchestrator` launches nodes as **subprocesses** via
`subprocess.Popen`. Service-level launchers (`g1_app_service`,
`main_app_service`) use `multiprocessing.Process` instead, via a `NodeLauncher`
class that sets `mp.set_start_method("spawn", force=True)` on Windows.

**Shutdown sequence:**

```python
orch.send_command(CTRL_SHUTDOWN)    # broadcast to all nodes
time.sleep(wait_timeout)            # grace period (default 5 s)
for process in still_running:
    process.kill()
self._cleanup_zmq()                 # linger=0 on both sockets; context NOT terminated
```

---

## 12. Homie Hardware — Non-ZMQ Command Path

The Homie G1 command path bypasses ZMQ entirely on the send side because the
hardware-side `transfer_upper.py` reads raw TCP bytes with `np.frombuffer`.

**Sender** (`packages/homie-interfaces/homie_interfaces/publisher.py`):

```python
self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
self.sock.connect((host, port))
self.sock.sendall(command_array.astype(np.float32).tobytes())  # 72 raw bytes
```

**Receiver** (`packages/homie-interfaces/homie_interfaces/receiver.py`):

```python
sub = ctx.socket(zmq.SUB)
sub.bind("tcp://0.0.0.0:8765")   # INVERTED: SUB binds — hardware is the stable server
sub.setsockopt(zmq.SUBSCRIBE, b"HOMIE_CMD")
# Decode: np.frombuffer(data, dtype=np.float32)
```

**Wire layout** — 18 × `float32` = 72 bytes:

| Indices  | Content |
|----------|---------|
| `[0]`    | x velocity |
| `[1]`    | y velocity |
| `[2]`    | yaw velocity |
| `[3]`    | height |
| `[4:11]` | left arm joints (7 DOF) |
| `[11:18]`| right arm joints (7 DOF) |

The inverted bind/connect pattern (SUB binds, publisher connects) exists because
the hardware unit has a stable IP/port while the teleop PC acts as the volatile
client.

---

## 13. ManusNode Publish Deduplication

Before each publish cycle ManusNode hashes the payload with BLAKE2b and
compares it to the previous digest for that topic:

```python
digest = hashlib.blake2b(msgpack.packb(data), digest_size=16).digest()
if digest == self._last_digest[topic]:
    return   # identical frame — skip publish
self._last_digest[topic] = digest
self._send(topic, data)
```

This prevents redundant traffic when the Manus SDK returns the same frame on
consecutive polls (common during idle or slow motion). No other publisher nodes
implement deduplication.

---

## 14. Sensor Landscape Tracking

`ManusSubscriber` maintains a live map of detected sensors updated on every
`read()` call:

```python
sensor_landscape = {
    "trackers": {
        tracker_id: {"handedness", "last_seen_timestamp", "is_shown"}
    },
    "skeletons": {
        glove_id: {"handedness", "last_seen_timestamp", "is_shown"}
    }
}
```

Entries older than `landscape_timeout_seconds` (default 30 s) are purged on
each `read()`.

**`wait_for_sensor()`** blocks until a sensor matching `(sensor_type, handedness)`
has been seen for at least `require_stable_frames=2` consecutive reads (default).
Logs progress every 5 s. Used by hardware nodes that must wait for specific Manus
gloves before entering the control loop.

```python
subscriber.wait_for_sensor(
    sensor_type="tracker",
    handedness="left",
    timeout_sec=30.0,
    require_stable_frames=2,
)
```

---

## 15. Error Handling Reference

| Situation | Behaviour |
|-----------|-----------|
| `send_multipart` fails (EAGAIN or other) | Caught, logged as error, **cycle continues** — never fatal |
| `recv_multipart` raises `zmq.Again` | Caught as normal "no message" — returns `{}` |
| `get_data()` raises exception | Logged, cycle skips publish — node stays alive |
| `process_data()` raises exception | Logged, frame skipped — `last_data_time` not updated |
| Data timeout exceeded | `go_to_safe_position()` triggered; `last_data_time` reset after |
| Node no heartbeat for 5 s | Orchestrator marks `NodeStatus.is_healthy()` → `False` |
| Socket `close()` | Always `linger=0` — unsent messages dropped immediately |
| Context cleanup | `zmq.Context.instance()` is **never** terminated |

---

## 16. Class Hierarchy

```
ManagedNode  [managed.py]
│  Owns: SUB(ctrl:5550) + PUSH(status:5551)
│  Provides: control command dispatch, status/heartbeat, run() loop
│
├── HardwarePublisherNode  [hardware_publisher.py]
│   Adds: PUB(data) + drift-corrected rate limiter
│   Abstract: get_data() → (topic, data) | list | None
│   ├── ManusNode               PUB :5555 — two topics, BLAKE2b dedup
│   └── CameraNode              PUB :5557 — colour + depth topics
│
├── TeleopNode  [teleop_node.py]
│   Adds: subscriber.read() each tick, safe-position interpolation
│   Abstract: setup(), process_data(), get_safe_position()
│   ├── ArmTeleopNode           IK-based tracker → joint angles
│   │   └── NovaControlNode
│   ├── DualArmTeleopNode       dual IK + base motion
│   │   ├── G1ControlNode
│   │   └── HomieControlNode    + extra keyboard SUB :5556
│   └── HandTeleopNode          optimizer-based skeleton → joint angles
│       ├── InspireControlNode
│       ├── DH5ControlNode
│       └── GripperControlNode  200 Hz, pinch-distance mapping
│
├── CommandNode  [command_node.py]
│   Adds: PUB(data) + thread-safe Queue
│   API: send_command(str) callable from any thread
│
└── RecorderNode  [recorder_node.py]
    Adds: N × SUB sockets, all on shared poller, episode buffering
    Abstract: _create_episode_writer(buffers, episode_number) → Thread
    └── DataRecorderNode        HDF5 / LeRobot v3 episode writer

TeleopOrchestrator  [orchestrator.py]
    Owns: PUB(ctrl:5550 bind) + PULL(status:5551 bind)
    Manages: subprocess / mp.Process lifecycle, NodeStatus tracking

ManusSubscriber  [manus-subscriber/subscriber.py]
    RCVHWM=0, receive_latest() drain, landscape tracking, wait_for_sensor()
```

---

## 17. Implementing a New Node — Quick Reference

### New publisher node

```python
class MyPublisherNode(HardwarePublisherNode):
    def __init__(self):
        super().__init__(
            node_id="my_publisher",
            data_endpoint="tcp://*:5558",   # pick an unused port
            bind=True,
            rate_hz=60.0,
        )

    def get_data(self):
        data = self.hardware.read()         # must be msgpack-safe (no raw numpy)
        if data is None:
            return None
        return TOPIC_MY_DATA, data.to_dict()
```

### New subscriber / control node

```python
class MyControlNode(TeleopNode):
    def setup(self):
        self.subscriber = ManusSubscriber(
            address="tcp://localhost:5555",
            context=zmq.Context.instance(),  # use shared context
            topic=TOPIC_MANUS_TRACKERS,
        )
        self.interface = MyRobotInterface(...)

    def process_data(self, data):
        trackers = data["trackers"]
        # compute joints ...
        return np.array([...])

    def get_safe_position(self):
        return np.zeros(self.n_joints)
```

> **Mandatory**: call `subscriber.read()` on every control iteration
> **including** when `_teleop_active=False`.  
> `TeleopNode._main_loop_iteration()` already does this automatically —
> only override if you bypass the base class.

### New recorder

```python
class MyRecorder(RecorderNode):
    def _create_episode_writer(self, episode_buffers, episode_number):
        return MyWriterThread(episode_buffers, episode_number, self._output_dir)

recorder = MyRecorder(
    node_id="recorder",
    data_endpoints=["tcp://localhost:5555", "tcp://localhost:5557"],
    output_dir="output/",
)
recorder.run()
```

---

## 18. Full Socket Configuration Summary

| Socket | Type | bind/connect | HWM | LINGER | RCVTIMEO | Topic filter |
|--------|------|-------------|-----|--------|----------|--------------|
| Orchestrator Control | `zmq.PUB` | bind `0.0.0.0:5550` | default | 0 | — | — |
| Orchestrator Status | `zmq.PULL` | bind `0.0.0.0:5551` | default | — | — | — |
| Node Control | `zmq.SUB` | connect `:5550` | default | 0 | — | `b"CTRL"` |
| Node Status | `zmq.PUSH` | connect `:5551` | default | — | — | — |
| HardwarePublisherNode Data | `zmq.PUB` | bind (default) | default | **0** | — | — |
| ManusSubscriber Data | `zmq.SUB` | connect | **0** (unlimited) | — | -1 (reset after use) | configurable |
| RecorderNode Data (each) | `zmq.SUB` | connect | default | — | — | `b""` (all) |
| HomieCommandReceiver | `zmq.SUB` | **bind** `:8765` | default | — | — | `b"HOMIE_CMD"` |
| HomieControlNode Keyboard | `zmq.SUB` | connect `:5556` | default | — | **0** (non-blocking) | `b"KEYBOARD_STATE"` |
