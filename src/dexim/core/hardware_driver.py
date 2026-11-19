"""Strict RobotInterface foundation for supervised, acquisition-only hardware."""

import time

import numpy as np
from dexim.core.robot_interface import JointState

from dexim.core.execution import position_vector
from dexim.core.hardware import SafetyStatus
from dexim.core.ownership import EndpointOwnership


class HardwareDriver:
    def __init__(self, descriptor, physical_id, namespace, *, safety_probe=None):
        self.descriptor = descriptor
        self.ownership = EndpointOwnership(physical_id, namespace=namespace)
        self.safety_probe = safety_probe
        self.connected = False
        self.submitted = False
        self.safety_confirmed = False
        self.acquired_at = 0.0
        self.feedback_sequence = 0
        self.vendor_target = ()
        self.vendor_result = None

    def connect(self):
        self.ownership.acquire()
        # An uncertain partial open retains the reservation until explicit cleanup.
        self._open()
        self.connected = True

    def disconnect(self):
        self._close()
        self.connected = False
        if (
            not self.submitted and self.descriptor.runtime_id == "fake-vendor"
        ) or self.safety_confirmed:
            self.ownership.release()

    def is_connected(self):
        return self.connected

    def read(self):
        if not self.connected:
            raise RuntimeError("Hardware is disconnected")
        q = position_vector(self._read(), len(self.descriptor.joint_names))
        if any(q < np.asarray(self.descriptor.lower) - 1e-10) or any(
            q > np.asarray(self.descriptor.upper) + 1e-10
        ):
            raise ValueError("Feedback outside model limits")
        self.acquired_at = time.monotonic()
        self.feedback_sequence += 1
        # Numeric DTO compatibility only; envelopes declare velocity/effort unavailable.
        return JointState(
            q.copy(), np.zeros(len(q)), np.zeros(len(q)), self.acquired_at
        )

    def write(self, command):
        if not self.connected:
            raise RuntimeError("Hardware is disconnected")
        if command.mode != "position":
            raise ValueError("Only position mode is qualified")
        q = self.descriptor.apply_limits(command.q)
        self.vendor_target = tuple(self._convert(q))
        self.submitted = True
        self.safety_confirmed = False
        self.vendor_result = self._write(self.vendor_target)
        self.submission_outcome = self._outcome(self.vendor_result)

    def safe_stop(self, reason):
        if self.descriptor.runtime_id == "native":
            return SafetyStatus(False, f"PHYSICAL_SAFING_UNQUALIFIED: {reason}")
        if not self.submitted:
            return SafetyStatus(True, "ACQUISITION_ONLY_NO_SUBMISSIONS")
        if self.safety_probe is None:
            return SafetyStatus(False, f"PHYSICAL_SAFING_UNQUALIFIED: {reason}")
        status = self.safety_probe(reason)
        if not isinstance(status, SafetyStatus):
            raise ValueError("Explicit safety evidence is required")
        self.safety_confirmed = status.confirmed
        return status

    def time(self):
        return time.monotonic()

    def estop(self):
        raise NotImplementedError("No qualified physical emergency-stop query")

    def joint_names(self):
        return list(self.descriptor.joint_names)

    actuated_joint_names = joint_names
    full_joint_names = joint_names

    def num_joint_configurations(self):
        return len(self.descriptor.joint_names)

    num_actuated_configurations = num_joint_configurations
    num_full_configurations = num_joint_configurations
