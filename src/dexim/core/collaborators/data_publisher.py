"""DataPlanePublisher — ZMQ PUB socket for action and observation messages."""

from __future__ import annotations

import time
from typing import Any

import numpy as np
import zmq
from dexim.core.messages import TopicBuilder, pack_data_message
from dexim.core.robot_interface import JointState
from loguru import logger


class DataPlanePublisher:
    """Publishes joint-command actions and joint-state observations over ZMQ PUB.

    The publisher is inactive until :meth:`activate` is called (matching the
    node's ``on_start`` lifecycle hook).

    Args:
        node_id: Node identifier used for topic generation.
        data_endpoint: ZMQ endpoint string for the PUB socket.
        bind: True to bind, False to connect.
    """

    def __init__(
        self,
        node_id: str,
        data_endpoint: str = "tcp://*:5556",
        bind: bool = True,
    ) -> None:
        ctx = zmq.Context.instance()
        pub: zmq.Socket = ctx.socket(zmq.PUB)
        pub.setsockopt(zmq.LINGER, 0)
        pub.setsockopt(
            zmq.SNDHWM, 100
        )  # Drop when no subscriber; prevent unbounded buffering
        if bind:
            pub.bind(data_endpoint)
        else:
            pub.connect(data_endpoint)
        self._pub: zmq.Socket | None = pub
        self._is_active: bool = False

        builder = TopicBuilder()
        self._action_topic: bytes = builder.action.joint_cmd(node_id)
        self._state_topic: bytes = builder.observation.joint_state(node_id)

        logger.info(f"Publisher ready on {data_endpoint}")
        logger.debug(f"  Action topic:  {self._action_topic}")
        logger.debug(f"  State topic:   {self._state_topic}")

    @property
    def is_active(self) -> bool:
        """True when publishing is enabled."""
        return self._is_active

    def activate(self) -> None:
        """Enable publishing (call from node on_start)."""
        self._is_active = True

    def deactivate(self) -> None:
        """Disable publishing (call from node on_pause / on_stop)."""
        self._is_active = False

    def publish_action(
        self,
        joint_cfgs: np.ndarray,
        extra_data: dict[str, Any] | None = None,
    ) -> None:
        """Publish a joint-command action.

        No-op when inactive.

        Args:
            joint_cfgs: Joint positions array.
            extra_data: Optional dict of additional payload fields to merge
                into the published message.  Consumers that only need joint
                positions can ignore unknown keys.  Callers are responsible
                for serialisability of the values.
        """
        if not self._is_active:
            return
        if joint_cfgs.dtype != np.float32:
            joint_cfgs = joint_cfgs.astype(np.float32)
        payload: dict[str, Any] = {"q": joint_cfgs.tolist()}
        if extra_data:
            payload.update(extra_data)
        self._send(self._action_topic, payload)

    def publish_observation(self, state: JointState | None) -> None:
        """Publish a joint-state observation.

        No-op when inactive or when state is None.

        Args:
            state: JointState from interface.read().
        """
        if not self._is_active or state is None:
            return
        try:
            state_dict = {
                "q": state.q.tolist(),
                "qd": state.qd.tolist(),
                "tau": state.tau.tolist(),
                "stamp": state.stamp,
            }
            self._send(self._state_topic, state_dict)
        except Exception as exc:
            logger.debug(f"Failed to publish observation: {exc}")

    def close(self) -> None:
        """Close the ZMQ PUB socket."""
        self._is_active = False
        try:
            if self._pub is not None:
                self._pub.close(linger=0)
        finally:
            self._pub = None

    def _send(self, topic: bytes, data: Any) -> None:
        if self._pub is None:
            return
        try:
            topic_frame, payload_frame = pack_data_message(topic, time.time(), data)
            self._pub.send_multipart([topic_frame, payload_frame], flags=zmq.DONTWAIT)
        except zmq.Again:
            # Non-fatal: no subscriber or HWM reached; message dropped.
            pass
        except Exception as exc:
            logger.debug(f"Failed to send on topic {topic}: {exc}")
