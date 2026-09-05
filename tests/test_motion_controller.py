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
    ) -> None:
        self.readings = readings
        self.connected = connected
        self.commands: list[np.ndarray] = []
        self.read_count = 0

    def is_connected(self) -> bool:
        return self.connected

    def read(self) -> JointState:
        self.read_count += 1
        reading = self.readings.pop(0)
        if isinstance(reading, Exception):
            raise reading
        return reading

    def write(self, command: JointCommand) -> None:
        assert command.q is not None
        self.commands.append(command.q.copy())


def controller(interface: FakeInterface) -> MotionController:
    result = MotionController(interface, rate_hz=10.0)  # type: ignore[arg-type]
    result.rate_limiter.sleep = lambda: {}  # type: ignore[method-assign]
    return result


def test_move_to_safe_rejects_disconnected_interface() -> None:
    interface = FakeInterface([], connected=False)

    assert not controller(interface).move_to_safe(np.array([1.0]))
    assert interface.commands == []


def test_move_to_safe_fails_closed_when_initial_read_fails() -> None:
    interface = FakeInterface([RuntimeError("read failed")])

    assert not controller(interface).move_to_safe(np.array([1.0]))
    assert interface.commands == []


def test_move_to_safe_enforces_peak_velocity_and_verifies_feedback(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = np.array([1.0])
    interface = FakeInterface(
        [
            joint_state([0.0]),
            joint_state(target),
            joint_state(target),
            joint_state(target),
        ]
    )
    motion = controller(interface)
    monkeypatch.setattr(motion_module.time, "sleep", lambda _seconds: None)

    assert motion.move_to_safe(target, max_velocity_rad_s=0.5)
    positions = np.concatenate(([0.0], [q[0] for q in interface.commands]))
    assert np.max(np.abs(np.diff(positions))) <= 0.5 * motion.dt + 1e-12
    assert np.array_equal(motion._current_joint_positions, target)


def test_safe_position_verification_requires_consecutive_samples(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
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
    monkeypatch.setattr(motion_module.time, "sleep", lambda _seconds: None)

    assert controller(interface).move_to_safe(target, max_velocity_rad_s=10.0)
    assert interface.read_count == 7


def test_safe_position_verification_timeout_returns_false(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    target = np.array([1.0])
    interface = FakeInterface([joint_state([0.0]), joint_state([0.5])])
    monkeypatch.setattr(motion_module, "_SAFE_POSITION_SETTLE_TIMEOUT_SEC", 0.0)
    monkeypatch.setattr(motion_module.time, "sleep", lambda _seconds: None)

    assert not controller(interface).move_to_safe(target, max_velocity_rad_s=10.0)


@pytest.mark.parametrize("velocity", [0.0, -1.0])
def test_move_to_safe_rejects_non_positive_velocity(velocity: float) -> None:
    interface = FakeInterface([joint_state([0.0])])

    with pytest.raises(ValueError, match="greater than zero"):
        controller(interface).move_to_safe(
            np.array([1.0]),
            max_velocity_rad_s=velocity,
        )
