"""dexim.core.collaborators -- shared pipeline collaborators for robot control nodes.

Each collaborator handles one concern in the control pipeline and can be
composed freely by any robot package (arm or hand):

    from dexim.core.collaborators import (
        MotionController,
        DataPlanePublisher,
        PipelineProfiler,
        JointFilter,
        ErgonomicsReceiver,
        SkeletonReceiver,
        TrackerReceiver,
    )

Protocols
---------
ReceiverProtocol
    Minimal structural protocol satisfied by both SkeletonReceiver (hands)
    and TrackerReceiver (arms).

WaitableSubscriberProtocol
    Extended subscriber protocol with active sensor-waiting capability.
"""

from __future__ import annotations

from dexim.core.collaborators.data_publisher import DataPlanePublisher
from dexim.core.collaborators.ergonomics_receiver import ErgonomicsReceiver
from dexim.core.collaborators.joint_filter import JointFilter
from dexim.core.collaborators.motion_controller import MotionController
from dexim.core.collaborators.profiler import PipelineProfiler
from dexim.core.collaborators.protocols import (
    ReceiverProtocol,
    WaitableSubscriberProtocol,
)
from dexim.core.collaborators.skeleton_receiver import SkeletonReceiver
from dexim.core.collaborators.tracker_receiver import TrackerReceiver

__all__ = [
    # Collaborators
    "MotionController",
    "DataPlanePublisher",
    "PipelineProfiler",
    "JointFilter",
    "ErgonomicsReceiver",
    "SkeletonReceiver",
    "TrackerReceiver",
    # Protocols
    "ReceiverProtocol",
    "WaitableSubscriberProtocol",
]
