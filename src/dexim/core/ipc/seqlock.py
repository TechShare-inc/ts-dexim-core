"""SeqLock -- sequence-lock primitive for lock-free shared-memory IPC.

A SeqLock uses an integer counter embedded in the shared-memory buffer to
coordinate a single writer and multiple readers without any OS mutex:

- **Even** counter  -> data is stable; safe to read.
- **Odd** counter   -> write in progress; reader spins or retries.

Write protocol (caller must call begin/end or use the context manager):
    1. Increment ``seq_lock`` to an **odd** value (signals write in progress).
    2. Write the data into the buffer.
    3. Increment ``seq_lock`` to an **even** value (signals write complete).

Read protocol:
    1. Read ``seq1`` -- retry if **odd** (write in progress).
    2. Copy the data fields into local variables.
    3. Read ``seq2``.
    4. If ``seq1 != seq2`` a write occurred during the read; discard and retry.
    5. If ``seq1 == seq2`` (both even) the read is consistent.

Crash safety:
    If the writer process crashes mid-write, the counter stays **odd**
    permanently.  The reader detects this via its ``max_retries`` limit and
    returns the last known-good value instead of blocking forever.

Platform note:
    On x86/x64 (TSO memory model) aligned 32-bit reads and writes are
    naturally atomic -- no explicit memory-barrier instructions are needed for
    this SeqLock implementation in Python/ctypes.  On ARM targets, callers
    should verify that their platform provides equivalent guarantees.
"""

from __future__ import annotations

import ctypes
from multiprocessing.shared_memory import SharedMemory

# The sequence counter is always the *first* field of every buffer structure.
# These helpers access it by byte offset so that the `seqlock` module has no
# dependency on the concrete buffer structures defined in `buffers.py`.
_SEQ_OFFSET: int = 0
_SeqType = ctypes.c_uint32


def _get_seq(buf: memoryview) -> int:
    """Read the sequence counter from a shared-memory memoryview (offset 0)."""
    return _SeqType.from_buffer(buf, _SEQ_OFFSET).value  # type: ignore[return-value]


def _set_seq(buf: memoryview, value: int) -> None:
    """Write the sequence counter into a shared-memory memoryview (offset 0)."""
    _SeqType.from_buffer(buf, _SEQ_OFFSET).value = value  # type: ignore[assignment]


# ---------------------------------------------------------------------------
# Public helpers used by TargetBufferWriter / StateBufferWriter
# ---------------------------------------------------------------------------


def seqlock_begin_write(shm: SharedMemory) -> None:
    """Increment the SeqLock counter to **odd** (write started).

    Args:
        shm: Shared memory block whose first 4 bytes hold the seq counter.
    """
    assert isinstance(shm.buf, memoryview)
    _set_seq(shm.buf, _get_seq(shm.buf) + 1)


def seqlock_end_write(shm: SharedMemory) -> None:
    """Increment the SeqLock counter to **even** (write complete).

    Args:
        shm: Shared memory block whose first 4 bytes hold the seq counter.
    """
    assert isinstance(shm.buf, memoryview)
    _set_seq(shm.buf, _get_seq(shm.buf) + 1)


def seqlock_read_seq(shm: SharedMemory) -> int:
    """Return the current sequence counter value.

    Args:
        shm: Shared memory block whose first 4 bytes hold the seq counter.

    Returns:
        Current sequence counter value (even = stable, odd = writing).
    """
    assert isinstance(shm.buf, memoryview)
    return _get_seq(shm.buf)
