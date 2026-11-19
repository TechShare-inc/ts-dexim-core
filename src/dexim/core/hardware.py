"""One isolated final I/O owner; revocation never waits behind native calls."""

from __future__ import annotations

import asyncio
import math
import multiprocessing as mp
import queue
import time
import uuid
from dataclasses import dataclass
from typing import Literal, overload

from dexim.core.execution import position_vector
from dexim.core.robot_interface import JointCommand


@dataclass(frozen=True)
class Feedback:
    q: tuple[float, ...]
    sequence: int
    acquired_at: float


@dataclass(frozen=True)
class Submission:
    instance_id: str
    epoch_id: str
    command_id: str
    sequence: int
    input_at: float
    requested_q: tuple[float, ...]
    q: tuple[float, ...]
    vendor_target: tuple[float, ...]
    dispatched_at: float
    completed_at: float
    outcome: str
    vendor_result: str
    input_sequence: int

    def __post_init__(self):
        for key in ("instance_id", "epoch_id", "command_id"):
            if not isinstance(getattr(self, key), str) or not getattr(self, key):
                raise ValueError("Submission identity required")
        if (
            type(self.sequence) is not int
            or self.sequence < 1
            or type(self.input_sequence) is not int
            or self.input_sequence < 1
        ):
            raise ValueError("Submission sequence required")
        if self.outcome not in {"accepted", "failed", "unknown"} or not isinstance(
            self.vendor_result, str
        ):
            raise ValueError("Invalid submission outcome")
        if (
            not all(
                math.isfinite(v)
                for v in (self.input_at, self.dispatched_at, self.completed_at)
            )
            or not 0 <= self.input_at <= self.dispatched_at <= self.completed_at
        ):
            raise ValueError("Invalid submission timing")
        for key in ("requested_q", "q", "vendor_target"):
            values = position_vector(getattr(self, key), len(self.q))
            object.__setattr__(self, key, tuple(float(v) for v in values))


@dataclass(frozen=True)
class WriteRequest:
    instance_id: str
    generation: int
    epoch_id: str
    sequence: int
    input_at: float
    deadline: float
    requested_q: tuple[float, ...]
    input_sequence: int = 1

    def __post_init__(self):
        if (
            not self.instance_id
            or not self.epoch_id
            or type(self.generation) is not int
            or type(self.sequence) is not int
            or self.sequence < 1
            or not math.isfinite(self.input_at)
            or not math.isfinite(self.deadline)
            or not 0 <= self.input_at < self.deadline
        ):
            raise ValueError("Invalid final-owner write identity/timing")


@dataclass(frozen=True)
class SafetyStatus:
    confirmed: bool
    detail: str

    def __post_init__(self):
        if (
            type(self.confirmed) is not bool
            or not isinstance(self.detail, str)
            or not self.detail
        ):
            raise ValueError("Explicit typed safety evidence required")


def _io_owner(factory, descriptor, instance, authority, jobs, events):
    """Only this process constructs the driver and enters the vendor API."""
    interface = None
    event_sequence = 0
    feedback = None
    last_command = 0
    active_epoch = None

    def emit(job_id, kind, value):
        nonlocal event_sequence
        event_sequence += 1
        events.put((event_sequence, job_id, kind, value))

    try:
        interface = factory()
        interface.connect()
        emit("connect", "connected", True)
        while True:
            job_id, operation, data = jobs.get()
            try:
                if operation == "close":
                    interface.disconnect()
                    emit(job_id, "closed", True)
                    return
                if operation == "safe":
                    status = interface.safe_stop(data)
                    emit(job_id, "safe", status)
                    continue
                if operation == "activate":
                    generation, active_epoch = data
                    with authority.get_lock():
                        if authority[0] != generation or not authority[1]:
                            active_epoch = None
                    continue
                if operation == "write":
                    if not isinstance(data, WriteRequest):
                        raise ValueError("Typed final-owner target required")
                    generation, epoch, sequence = (
                        data.generation,
                        data.epoch_id,
                        data.sequence,
                    )
                    input_at, deadline, requested = (
                        data.input_at,
                        data.deadline,
                        data.requested_q,
                    )
                    q = descriptor.apply_limits(requested)
                    now = time.monotonic()
                    # This lock protects admission only, never vendor I/O or safing.
                    with authority.get_lock():
                        if (
                            data.instance_id != instance
                            or epoch != active_epoch
                            or authority[0] != generation
                            or not authority[1]
                            or now >= authority[2]
                            or now >= deadline
                            or input_at > now
                            or now - input_at >= authority[3]
                            or sequence <= last_command
                            or feedback is None
                            or now - feedback.acquired_at >= authority[3]
                        ):
                            emit(job_id, "rejected", "FINAL_EXECUTOR_FENCED_OR_STALE")
                            continue
                        authority[4] = 1
                    last_command = sequence
                    dispatched = time.monotonic()
                    outcome, result = "unknown", "native call did not complete"
                    try:
                        interface.write(JointCommand(q=q))
                        result = repr(interface.vendor_result)
                        outcome = interface.submission_outcome
                    except Exception as exc:
                        result = f"{type(exc).__name__}: {exc}"
                    finally:
                        completed = time.monotonic()
                        receipt = Submission(
                            instance,
                            epoch,
                            job_id,
                            sequence,
                            input_at,
                            tuple(requested),
                            tuple(q),
                            tuple(interface.vendor_target),
                            dispatched,
                            completed,
                            outcome,
                            result,
                            data.input_sequence,
                        )
                        # A receipt precedes the following read, regardless of its result.
                        emit(job_id, "receipt", receipt)
                        with authority.get_lock():
                            authority[4] = 0
                    if outcome != "accepted":
                        with authority.get_lock():
                            authority[1] = 0
                        continue
                state = interface.read()
                feedback = Feedback(
                    tuple(position_vector(state.q, len(descriptor.joint_names))),
                    interface.feedback_sequence,
                    interface.acquired_at,
                )
                emit(job_id, "feedback", feedback)
            except Exception as exc:
                with authority.get_lock():
                    authority[1] = 0
                emit(job_id, "error", f"{type(exc).__name__}: {exc}")
    except Exception as exc:
        emit("connect", "error", f"{type(exc).__name__}: {exc}")
    finally:
        with authority.get_lock():
            authority[1] = 0


class HardwareOwner:
    """Parent side of the revocable mailbox and loss-detecting receipt channel.

    Queue insertion never establishes vendor submission. Instance and generation
    belong to this owner lifetime; a new process cannot adopt an old mailbox.
    """

    def __init__(self, factory, descriptor, *, watchdog=1.0):
        if not math.isfinite(watchdog) or watchdog <= 0:
            raise ValueError("A positive watchdog is required")
        self.descriptor = descriptor
        self.instance_id = uuid.uuid4().hex
        self.watchdog = watchdog
        context = mp.get_context("spawn")
        # generation, active, watchdog deadline, input/feedback age budget, in-flight
        self.authority = context.Array("d", [0, 0, 0, watchdog, 0])
        self.jobs, self.events = context.Queue(), context.Queue()
        self.process = context.Process(
            target=_io_owner,
            args=(
                factory,
                descriptor,
                self.instance_id,
                self.authority,
                self.jobs,
                self.events,
            ),
            daemon=True,
        )
        self.epoch = None
        self.ended = set()
        self.feedback = None
        self.feedback_valid = False
        self.pending_receipts = []
        self._responses = {}
        self._event_sequence = 0
        self._command_sequence = 0
        self.fault = ""
        self.safety = SafetyStatus(
            descriptor.runtime_id == "fake-vendor",
            "ACQUISITION_ONLY_NO_SUBMISSIONS"
            if descriptor.runtime_id == "fake-vendor"
            else "PHYSICAL_SAFING_PROFILE_UNQUALIFIED",
        )
        self.closed = False
        self.closing = False
        self.process.start()

    def poll(self):
        if self.closed:
            return
        while True:
            try:
                sequence, job, kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if sequence != self._event_sequence + 1:
                self.revoke()
                self.fault = "EXECUTOR_EVENT_LOSS"
                raise RuntimeError(self.fault)
            self._event_sequence = sequence
            self._responses[(job, kind)] = value
            if kind == "receipt":
                self.pending_receipts.append(value)
                if value.outcome != "accepted":
                    self.fault = (
                        f"SUBMISSION_{value.outcome.upper()}: {value.vendor_result}"
                    )
                    self.revoke()
            elif kind == "feedback":
                if self.feedback and value.sequence <= self.feedback.sequence:
                    self.fault = "FEEDBACK_SEQUENCE_REGRESSION"
                    self.revoke()
                    raise RuntimeError(self.fault)
                self.feedback = value
                self.feedback_valid = True
            elif kind == "error":
                self.feedback_valid = False
                self.fault = value
                self.revoke()
        if self.epoch is not None and time.monotonic() >= self.authority[2]:
            self.fault = "EXECUTOR_WATCHDOG_EXPIRED"
            self.revoke()
        if (
            not self.closing
            and not self.process.is_alive()
            and self.process.exitcode is not None
        ):
            self.revoke()
            self.fault = self.fault or "EXECUTOR_EXITED_WITH_POSSIBLE_RECEIPT_LOSS"

    def acknowledge_receipt(self, receipt):
        """Transfer one receipt to the consumer's retained/durable stream batch."""
        if not self.pending_receipts or self.pending_receipts[0] != receipt:
            raise ValueError("Receipt handoff must preserve executor order")
        self._responses.pop((receipt.command_id, "receipt"), None)
        self.pending_receipts.pop(0)

    @overload
    async def wait(
        self, job: str, kind: Literal["feedback"], deadline: float
    ) -> Feedback: ...

    @overload
    async def wait(
        self, job: str, kind: Literal["receipt"], deadline: float
    ) -> Submission: ...

    @overload
    async def wait(
        self, job: str, kind: Literal["safe"], deadline: float
    ) -> SafetyStatus: ...

    @overload
    async def wait(
        self, job: str, kind: Literal["connected", "closed"], deadline: float
    ) -> bool: ...

    async def wait(
        self, job: str, kind: str, deadline: float
    ) -> Feedback | Submission | SafetyStatus | bool:
        """Wait for a response to an executor job until the monotonic deadline."""
        while time.monotonic() < deadline:
            self.poll()
            if (job, kind) in self._responses:
                return self._responses.pop((job, kind))
            receipt = self._responses.get((job, "receipt"))
            if (
                kind == "feedback"
                and receipt is not None
                and receipt.outcome != "accepted"
            ):
                raise RuntimeError(
                    f"SUBMISSION_{receipt.outcome.upper()}: {receipt.vendor_result}"
                )
            if (job, "rejected") in self._responses:
                raise RuntimeError(self._responses.pop((job, "rejected")))
            if (job, "error") in self._responses:
                raise RuntimeError(self._responses.pop((job, "error")))
            if not self.process.is_alive():
                raise RuntimeError(self.fault)
            await asyncio.sleep(0.002)
        raise TimeoutError(f"EXECUTOR_{kind.upper()}_UNRESOLVED")

    async def connect(self, deadline):
        return await self.wait("connect", "connected", deadline)

    async def prepare(self, deadline):
        if self.descriptor.runtime_id == "native":
            raise RuntimeError("PHYSICAL_SAFING_PROFILE_UNQUALIFIED")
        self.poll()
        if self.authority[4] or not self.safety.confirmed:
            raise RuntimeError("PREVIOUS_NATIVE_IO_OR_SAFETY_UNRESOLVED")
        result = await self.observe(deadline)
        if time.monotonic() - result.acquired_at >= self.watchdog:
            raise RuntimeError("STALE_FEEDBACK")
        return result

    async def observe(self, deadline):
        job = uuid.uuid4().hex
        self.jobs.put((job, "read", None))
        return await self.wait(job, "feedback", deadline)

    def activate(self, epoch):
        if self.descriptor.runtime_id == "native":
            raise RuntimeError("PHYSICAL_SAFING_PROFILE_UNQUALIFIED")
        if (
            self.epoch is not None
            or epoch in self.ended
            or self.authority[4]
            or not self.safety.confirmed
            or not self.feedback_valid
            or self.feedback is None
            or time.monotonic() - self.feedback.acquired_at >= self.watchdog
        ):
            raise RuntimeError("NOT_PREPARED_OR_UNRESOLVED")
        self.epoch = epoch
        with self.authority.get_lock():
            self.authority[0] += 1
            self.authority[1] = 1
            self.authority[2] = time.monotonic() + self.watchdog
        self.jobs.put((uuid.uuid4().hex, "activate", (int(self.authority[0]), epoch)))
        self.safety = SafetyStatus(False, "PHYSICAL_SAFETY_REQUIRES_CONFIRMATION")

    def keepalive(self, epoch):
        self.poll()
        with self.authority.get_lock():
            if (
                epoch != self.epoch
                or not self.authority[1]
                or time.monotonic() >= self.authority[2]
            ):
                raise RuntimeError("STALE_EPOCH_OR_WATCHDOG")
            self.authority[2] = time.monotonic() + self.watchdog

    def submit(
        self,
        epoch: str,
        command: JointCommand,
        *,
        input_at: float,
        deadline: float,
        input_sequence: int | None = None,
    ) -> str:
        """Queue a command and return its job ID for receipt correlation."""
        if command.mode != "position":
            raise ValueError("Only position mode is qualified")
        q = tuple(position_vector(command.q, len(self.descriptor.joint_names)))
        if not (
            math.isfinite(input_at)
            and math.isfinite(deadline)
            and 0 <= input_at <= time.monotonic() < deadline
        ):
            raise ValueError("Invalid input time/deadline")
        self.poll()
        if epoch != self.epoch or not self.authority[1]:
            raise RuntimeError("STALE_EPOCH")
        self._command_sequence += 1
        job = uuid.uuid4().hex
        request = WriteRequest(
            self.instance_id,
            int(self.authority[0]),
            epoch,
            self._command_sequence,
            input_at,
            min(deadline, input_at + self.watchdog),
            q,
            self._command_sequence if input_sequence is None else input_sequence,
        )
        self.jobs.put((job, "write", request))
        return job

    def revoke(self):
        with self.authority.get_lock():
            self.authority[0] += 1
            self.authority[1] = 0
        if self.epoch is not None:
            self.ended.add(self.epoch)
        self.epoch = None

    async def end(self, reason, deadline):
        self.revoke()
        job = uuid.uuid4().hex
        self.jobs.put((job, "safe", reason))
        try:
            self.safety = await self.wait(job, "safe", deadline)
        except (TimeoutError, RuntimeError) as exc:
            self.safety = SafetyStatus(False, str(exc))
        return self.safety

    async def close(self, deadline):
        self.closing = True
        self.revoke()
        job = uuid.uuid4().hex
        self.jobs.put((job, "close", None))
        try:
            await self.wait(job, "closed", deadline)
        except (TimeoutError, RuntimeError):
            # Teardown never establishes safety; uncertain driver reservations remain.
            self.safety = SafetyStatus(False, "EXECUTOR_TEARDOWN_UNRESOLVED")
            self.process.terminate()
        await asyncio.to_thread(self.process.join, max(0, deadline - time.monotonic()))
        if self.process.is_alive():
            self.process.kill()
            await asyncio.to_thread(self.process.join, 1)
        self.poll()
        self.closed = True
        self.jobs.close()
        self.events.close()
