from __future__ import annotations

from collections.abc import Iterable

import numpy as np
import pytest

from dexim.core.collaborators import motion_controller as motion_module
from dexim.core.collaborators.motion_controller import MotionController
from dexim.core.robot_interface import JointCommand, JointState


def joint_state(q: Iterable[float]) -> JointState:
    positions = np.array(list(q), dtype=float)
    return JointState(
        q=positions,
        qd=np.zeros_like(positions),
        tau=np.zeros_like(positions),
        stamp=0.0,
    )


class FakeInterface:
    def __init__(
        self,
        readings: list[JointState | Exception],
        *,
        connected: bool = True,
        clock: FakeClock | None = None,
    ) -> None:
        self.readings = readings
        self.connected = connected
        self.commands: list[np.ndarray] = []
        self.command_times: list[float] = []
        self.read_count = 0
        self.clock = clock

    def connect(self) -> None:
        self.connected = True

    def disconnect(self) -> None:
        self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def read(self) -> JointState:
        self.read_count += 1
        reading = self.readings.pop(0)
        if isinstance(reading, Exception):
            raise reading
        return reading

    def write(self, cmd: JointCommand) -> None:
        assert cmd.q is not None
        self.commands.append(cmd.q.copy())
        if self.clock is not None:
            self.command_times.append(self.clock.monotonic())

    def time(self) -> float:
        return 0.0

    def estop(self) -> bool:
        return False

    def num_joint_configurations(self) -> int:
        return 1

    def joint_names(self) -> list[str]:
        return ["joint_0"]

    def num_actuated_configurations(self) -> int:
        return 1

    def actuated_joint_names(self) -> list[str]:
        return ["joint_0"]

    def num_full_configurations(self) -> int:
        return 1

    def full_joint_names(self) -> list[str]:
        return ["joint_0"]


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def monotonic(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds


@pytest.fixture(autouse=True)
def fake_clock(monkeypatch: pytest.MonkeyPatch) -> FakeClock:
    clock = FakeClock()
    monkeypatch.setattr(motion_module.time, "monotonic", clock.monotonic)
    monkeypatch.setattr(motion_module.time, "sleep", clock.sleep)
    return clock


def controller(interface: FakeInterface) -> MotionController:
    return MotionController(interface, rate_hz=10.0)


def test_move_to_safe_rejects_disconnected_interface() -> None:
    interface = FakeInterface([], connected=False)

    assert not controller(interface).move_to_safe(np.array([1.0]))
    assert interface.commands == []


def test_move_to_safe_fails_closed_when_initial_read_fails() -> None:
    interface = FakeInterface([RuntimeError("read failed")])

    assert not controller(interface).move_to_safe(np.array([1.0]))
    assert interface.commands == []


def test_move_to_safe_enforces_peak_velocity_and_verifies_feedback(
    fake_clock: FakeClock,
) -> None:
    target = np.array([1.0])
    interface = FakeInterface(
        [joint_state([0.0]), *(joint_state(target) for _ in range(3))],
        clock=fake_clock,
    )
    motion = controller(interface)
    fake_clock.now = 100.0  # Simulate a long-idle controller before this move.

    assert motion.move_to_safe(target, max_velocity_rad_s=0.5)
    positions = np.concatenate(([0.0], [q[0] for q in interface.commands]))
    position_steps = np.diff(positions)
    command_intervals = np.diff([100.0, *interface.command_times])
    moving_intervals = command_intervals[np.abs(position_steps) > 1e-12]
    assert np.max(np.abs(position_steps)) <= 0.5 * motion.dt + 1e-12
    assert np.min(moving_intervals) >= motion.dt - 1e-12
    cached_position = motion._current_joint_positions
    assert cached_position is not None
    assert np.array_equal(cached_position, target)


def test_safe_position_verification_requires_consecutive_samples() -> None:
    target = np.array([1.0])
    interface = FakeInterface(
        [
            joint_state([0.0]),
            joint_state(target),
            joint_state(target),
            joint_state([0.5]),
            joint_state(target),
            joint_state(target),
            joint_state(target),
        ]
    )
    assert controller(interface).move_to_safe(target, max_velocity_rad_s=10.0)
    assert interface.read_count == 7


def test_safe_position_verification_timeout_returns_false(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = np.array([1.0])
    interface = FakeInterface([joint_state([0.0]), joint_state([0.5])])
    monkeypatch.setattr(motion_module, "_SAFE_POSITION_SETTLE_TIMEOUT_SEC", 0.0)

    assert not controller(interface).move_to_safe(target, max_velocity_rad_s=10.0)


@pytest.mark.parametrize("velocity", [0.0, -1.0])
def test_move_to_safe_rejects_non_positive_velocity(velocity: float) -> None:
    interface = FakeInterface([joint_state([0.0])])

    with pytest.raises(ValueError, match="greater than zero"):
        controller(interface).move_to_safe(
            np.array([1.0]),
            max_velocity_rad_s=velocity,
        )
