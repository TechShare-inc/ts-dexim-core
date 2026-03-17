"""dexim.core.ipc — Lock-free inter-process communication primitives.

Provides SeqLock-based shared-memory buffers for real-time robot control
across OS processes.  The design guarantees that a crashed or stalled writer
process never deadlocks the reader:

- No OS-level mutex is held during reads or writes.
- A permanently-odd ``seq_lock`` (writer crashed mid-write) is detected by
  the reader's retry limit, which then falls back to the last known-good
  value.

Typical usage
-------------
Brain process (writer side)::

    from dexim.core.ipc import (
        create_target_shm, TargetBufferWriter,
        create_state_shm, StateBufferReader,
    )
    target_shm = create_target_shm("my_target")
    writer = TargetBufferWriter(target_shm, num_joints=6)
    writer.write(joint_angles, timestamp)

Hardware Core process (reader + writer side)::

    from dexim.core.ipc import (
        open_target_shm, TargetBufferReader,
        open_state_shm, StateBufferWriter,
    )
    target_shm = open_target_shm("my_target")
    reader = TargetBufferReader(target_shm, num_joints=6)
    angles, ts = reader.read()
"""

from dexim.core.ipc.buffers import (
    JointStateBuffer,
    JointTargetBuffer,
    StateBufferReader,
    StateBufferWriter,
    TargetBufferReader,
    TargetBufferWriter,
    create_state_shm,
    create_target_shm,
    make_shm_name,
    open_state_shm,
    open_target_shm,
)

__all__ = [
    # Ctypes structures
    "JointTargetBuffer",
    "JointStateBuffer",
    # Typed wrappers
    "TargetBufferWriter",
    "TargetBufferReader",
    "StateBufferWriter",
    "StateBufferReader",
    # SHM factory helpers
    "create_target_shm",
    "open_target_shm",
    "create_state_shm",
    "open_state_shm",
    "make_shm_name",
]
