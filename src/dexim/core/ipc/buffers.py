"""Shared-memory buffer structures and typed read/write wrappers.

Two buffer layouts are provided:

``JointTargetBuffer``
    Written by the *Brain & Gateway* process (retargeting output),
    read by the *Hardware Core* process (RS-485 command source).

``JointStateBuffer``
    Written by the *Hardware Core* process (RS-485 telemetry),
    read by the *Brain & Gateway* and *Visualizer* processes.

Both structures embed a ``seq_lock`` counter at byte offset 0 (see
``dexim.core.ipc.seqlock`` for the protocol).

Typed wrappers (``TargetBufferWriter``, ``TargetBufferReader``, etc.)
encapsulate the SeqLock protocol and provide a high-level ``read()`` /
``write()`` API over the raw shared-memory blocks.
"""

from __future__ import annotations

import ctypes
import time
import uuid
from multiprocessing.shared_memory import SharedMemory

import numpy as np
from dexim.core.ipc.seqlock import (
    seqlock_begin_write,
    seqlock_end_write,
    seqlock_read_seq,
)

# Maximum number of joints supported by the buffer layouts.
# Setting this larger than any supported robot wastes a few hundred bytes of
# shared memory but avoids ever needing to change the layout.
MAX_JOINTS: int = 32


# ---------------------------------------------------------------------------
# ctypes buffer structures
# ---------------------------------------------------------------------------


class JointTargetBuffer(ctypes.Structure):
    """Shared-memory layout for target joint angles.

    Fields:
        seq_lock: SeqLock counter (even = stable, odd = write in progress).
        num_joints: Number of active joints written into ``joint_angles``.
        _pad: Alignment padding so ``timestamp`` starts on an 8-byte boundary.
        timestamp: Wall-clock time (seconds) when the targets were written.
        joint_angles: Target positions in radians, indices [0..num_joints).
    """

    _pack_ = 1
    _fields_ = [
        ("seq_lock", ctypes.c_uint32),
        ("num_joints", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 3),
        ("timestamp", ctypes.c_double),
        ("joint_angles", ctypes.c_float * MAX_JOINTS),
    ]


class JointStateBuffer(ctypes.Structure):
    """Shared-memory layout for actual joint telemetry.

    Fields:
        seq_lock: SeqLock counter.
        num_joints: Number of active joints.
        _pad: Alignment padding.
        timestamp: Wall-clock time of the last hardware read.
        joint_angles: Measured positions in radians.
        joint_velocities: Measured velocities in rad/s (zeros if unavailable).
        joint_torques: Measured torques in Nm (zeros if unavailable).
    """

    _pack_ = 1
    _fields_ = [
        ("seq_lock", ctypes.c_uint32),
        ("num_joints", ctypes.c_uint8),
        ("_pad", ctypes.c_uint8 * 3),
        ("timestamp", ctypes.c_double),
        ("joint_angles", ctypes.c_float * MAX_JOINTS),
        ("joint_velocities", ctypes.c_float * MAX_JOINTS),
        ("joint_torques", ctypes.c_float * MAX_JOINTS),
    ]


# ---------------------------------------------------------------------------
# Shared-memory factory helpers
# ---------------------------------------------------------------------------


def make_shm_name(prefix: str, kind: str) -> str:
    """Generate a collision-resistant shared-memory block name.

    Args:
        prefix: Logical prefix, e.g. ``"inspire_left"``.
        kind: Buffer role, e.g. ``"target"`` or ``"state"``.

    Returns:
        A name safe to pass to :class:`~multiprocessing.shared_memory.SharedMemory`.
    """
    uid = uuid.uuid4().hex[:8]
    return f"dexim_{prefix}_{kind}_{uid}"


def _create_shm(name: str, size: int) -> SharedMemory:
    shm = SharedMemory(name=name, create=True, size=size)
    # Zero-initialise so the seq_lock starts at 0 (even = stable).
    assert isinstance(shm.buf, memoryview)
    shm.buf[:size] = bytes(size)
    return shm


def create_target_shm(name: str) -> SharedMemory:
    """Create (and zero-initialise) a new target shared-memory block.

    Args:
        name: Unique name for the shared-memory segment.

    Returns:
        Open :class:`~multiprocessing.shared_memory.SharedMemory` block.
    """
    return _create_shm(name, ctypes.sizeof(JointTargetBuffer))


def open_target_shm(name: str) -> SharedMemory:
    """Attach to an existing target shared-memory block by name.

    Args:
        name: Name passed to :func:`create_target_shm` by the owning process.

    Returns:
        Open :class:`~multiprocessing.shared_memory.SharedMemory` block.
    """
    return SharedMemory(name=name, create=False, size=ctypes.sizeof(JointTargetBuffer))


def create_state_shm(name: str) -> SharedMemory:
    """Create (and zero-initialise) a new state shared-memory block.

    Args:
        name: Unique name for the shared-memory segment.

    Returns:
        Open :class:`~multiprocessing.shared_memory.SharedMemory` block.
    """
    return _create_shm(name, ctypes.sizeof(JointStateBuffer))


def open_state_shm(name: str) -> SharedMemory:
    """Attach to an existing state shared-memory block by name.

    Args:
        name: Name passed to :func:`create_state_shm` by the owning process.

    Returns:
        Open :class:`~multiprocessing.shared_memory.SharedMemory` block.
    """
    return SharedMemory(name=name, create=False, size=ctypes.sizeof(JointStateBuffer))


# ---------------------------------------------------------------------------
# Typed writer / reader wrappers
# ---------------------------------------------------------------------------


class TargetBufferWriter:
    """Writes target joint angles into a :class:`JointTargetBuffer` via SeqLock.

    Intended for use by the *Brain & Gateway* process.

    Args:
        shm: Shared-memory block created by :func:`create_target_shm`.
        num_joints: Number of active joints (must be ≤ ``MAX_JOINTS``).
    """

    def __init__(self, shm: SharedMemory, num_joints: int) -> None:
        if num_joints > MAX_JOINTS:
            raise ValueError(f"num_joints={num_joints} exceeds MAX_JOINTS={MAX_JOINTS}")
        self._shm = shm
        self._buf = JointTargetBuffer.from_buffer(shm.buf)  # type: ignore[arg-type]
        self._nq = num_joints
        # Initialise metadata (seq_lock was zeroed by the factory function).
        self._buf.num_joints = num_joints

    def write(self, joint_angles: np.ndarray, timestamp: float | None = None) -> None:
        """Write new joint targets into the shared-memory buffer.

        Uses the SeqLock write protocol so readers never observe a partial
        update.

        Args:
            joint_angles: Target positions in radians, shape ``(num_joints,)``.
            timestamp: Optional timestamp; defaults to ``time.time()``.
        """
        ts = timestamp if timestamp is not None else time.time()
        seqlock_begin_write(self._shm)  # → odd
        self._buf.timestamp = ts
        for i in range(self._nq):
            self._buf.joint_angles[i] = float(joint_angles[i])
        seqlock_end_write(self._shm)  # → even

    def close(self) -> None:
        """Release the ctypes buffer reference so the underlying SHM can be closed."""
        try:
            del self._buf
        except AttributeError:
            pass


class TargetBufferReader:
    """Reads target joint angles from a :class:`JointTargetBuffer` via SeqLock.

    Intended for use by the *Hardware Core* process.

    Args:
        shm: Shared-memory block opened by :func:`open_target_shm`.
        num_joints: Number of active joints to read.
        max_retries: Maximum spin iterations before returning last-good data.
    """

    def __init__(
        self,
        shm: SharedMemory,
        num_joints: int,
        max_retries: int = 20,
    ) -> None:
        self._shm = shm
        self._buf = JointTargetBuffer.from_buffer(shm.buf)  # type: ignore[arg-type]
        self._nq = num_joints
        self._max_retries = max_retries
        self._last_angles: np.ndarray = np.zeros(num_joints, dtype=np.float32)
        self._last_timestamp: float = 0.0

    def read(self) -> tuple[np.ndarray, float]:
        """Read latest joint targets with SeqLock consistency guarantee.

        If the writer is actively writing (seq odd) or a torn read is detected,
        the method retries up to ``max_retries`` times.  On exhaustion it
        returns the last successfully-read values.

        Returns:
            Tuple of ``(joint_angles, timestamp)`` where joint_angles has
            shape ``(num_joints,)`` and dtype ``float32``.
        """
        for _ in range(self._max_retries):
            seq1 = seqlock_read_seq(self._shm)
            if seq1 % 2 != 0:
                # Writer is active — spin without sleeping to minimise latency.
                continue
            angles = np.array(self._buf.joint_angles[: self._nq], dtype=np.float32)
            timestamp = self._buf.timestamp
            seq2 = seqlock_read_seq(self._shm)
            if seq1 == seq2:
                # Consistent read.
                self._last_angles = angles
                self._last_timestamp = timestamp
                return angles, timestamp
            # Torn read — retry.
        # Fallback: return last known-good values.
        return self._last_angles.copy(), self._last_timestamp

    def close(self) -> None:
        """Release the ctypes buffer reference so the underlying SHM can be closed."""
        try:
            del self._buf
        except AttributeError:
            pass


class StateBufferWriter:
    """Writes joint telemetry into a :class:`JointStateBuffer` via SeqLock.

    Intended for use by the *Hardware Core* process.

    Args:
        shm: Shared-memory block created by :func:`create_state_shm`.
        num_joints: Number of active joints (must be ≤ ``MAX_JOINTS``).
    """

    def __init__(self, shm: SharedMemory, num_joints: int) -> None:
        if num_joints > MAX_JOINTS:
            raise ValueError(f"num_joints={num_joints} exceeds MAX_JOINTS={MAX_JOINTS}")
        self._shm = shm
        self._buf = JointStateBuffer.from_buffer(shm.buf)  # type: ignore[arg-type]
        self._nq = num_joints
        self._buf.num_joints = num_joints

    def write(
        self,
        joint_angles: np.ndarray,
        joint_velocities: np.ndarray | None = None,
        joint_torques: np.ndarray | None = None,
        timestamp: float | None = None,
    ) -> None:
        """Write joint telemetry into the shared-memory buffer.

        Args:
            joint_angles: Measured positions in radians, shape ``(num_joints,)``.
            joint_velocities: Measured velocities in rad/s (zeros if ``None``).
            joint_torques: Measured torques in Nm (zeros if ``None``).
            timestamp: Optional timestamp; defaults to ``time.time()``.
        """
        ts = timestamp if timestamp is not None else time.time()
        vels = joint_velocities if joint_velocities is not None else np.zeros(self._nq)
        taus = joint_torques if joint_torques is not None else np.zeros(self._nq)

        seqlock_begin_write(self._shm)  # → odd
        self._buf.timestamp = ts
        for i in range(self._nq):
            self._buf.joint_angles[i] = float(joint_angles[i])
            self._buf.joint_velocities[i] = float(vels[i])
            self._buf.joint_torques[i] = float(taus[i])
        seqlock_end_write(self._shm)  # → even

    def close(self) -> None:
        """Release the ctypes buffer reference so the underlying SHM can be closed."""
        try:
            del self._buf
        except AttributeError:
            pass


class StateBufferReader:
    """Reads joint telemetry from a :class:`JointStateBuffer` via SeqLock.

    Intended for use by the *Brain & Gateway* and *Visualizer* processes.

    Args:
        shm: Shared-memory block opened by :func:`open_state_shm`.
        num_joints: Number of active joints to read.
        max_retries: Maximum spin iterations before returning last-good data.
    """

    def __init__(
        self,
        shm: SharedMemory,
        num_joints: int,
        max_retries: int = 20,
    ) -> None:
        self._shm = shm
        self._buf = JointStateBuffer.from_buffer(shm.buf)  # type: ignore[arg-type]
        self._nq = num_joints
        self._max_retries = max_retries
        self._last_angles: np.ndarray = np.zeros(num_joints, dtype=np.float32)
        self._last_velocities: np.ndarray = np.zeros(num_joints, dtype=np.float32)
        self._last_torques: np.ndarray = np.zeros(num_joints, dtype=np.float32)
        self._last_timestamp: float = 0.0

    def read(self) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        """Read latest joint telemetry with SeqLock consistency guarantee.

        Returns:
            Tuple of ``(angles, velocities, torques, timestamp)``.  All arrays
            have shape ``(num_joints,)`` and dtype ``float32``.
        """
        for _ in range(self._max_retries):
            seq1 = seqlock_read_seq(self._shm)
            if seq1 % 2 != 0:
                continue
            angles = np.array(self._buf.joint_angles[: self._nq], dtype=np.float32)
            vels = np.array(self._buf.joint_velocities[: self._nq], dtype=np.float32)
            taus = np.array(self._buf.joint_torques[: self._nq], dtype=np.float32)
            timestamp = self._buf.timestamp
            seq2 = seqlock_read_seq(self._shm)
            if seq1 == seq2:
                self._last_angles = angles
                self._last_velocities = vels
                self._last_torques = taus
                self._last_timestamp = timestamp
                return angles, vels, taus, timestamp
        return (
            self._last_angles.copy(),
            self._last_velocities.copy(),
            self._last_torques.copy(),
            self._last_timestamp,
        )

    def close(self) -> None:
        """Release the ctypes buffer reference so the underlying SHM can be closed."""
        try:
            del self._buf
        except AttributeError:
            pass
