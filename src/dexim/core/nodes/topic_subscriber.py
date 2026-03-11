"""Generic typed ZMQ subscriber for the DexImitate data plane.

``TopicSubscriber[T]`` is a hardware-agnostic replacement for device-specific
subscriber classes (e.g., the former ``ManusSubscriber``).  It wraps a ZMQ SUB
socket, receives two-frame ``[topic, payload]`` messages, unpacks the msgpack
payload, and deserialises the ``data`` field into an instance of a caller-
supplied message type.

Typical usage::

    from dexim.core.messages import HandState, TopicBuilder
    from dexim.core.nodes import TopicSubscriber

    topic = TopicBuilder().observation.hand_state("manus")
    sub = TopicSubscriber(
        address="tcp://localhost:5555",
        topic=topic,
        msg_type=HandState,
    )
    state: HandState | None = sub.read()
    sub.close()

For list-valued topics (e.g., a batch of ``RigidPose`` objects)::

    from dexim.core.messages import RigidPose

    sub = TopicSubscriber(
        address="tcp://localhost:5555",
        topic=TopicBuilder().observation.rigid_pose("manus"),
        msg_type=RigidPose,
    )
    poses: list[RigidPose] = sub.read_batch()
"""

from __future__ import annotations

import time
from typing import Any, Generic, TypeVar

import zmq
from dexim.core.messages import unpack_data_message
from loguru import logger

T = TypeVar("T")


class TopicSubscriber(Generic[T]):
    """ZMQ SUB socket that deserialises messages into a typed dataclass.

    The subscriber connects to a ZMQ PUB endpoint and filters on a single
    topic prefix.  Each received message is unpacked from msgpack and
    deserialised by calling ``msg_type.from_dict(data)``.

    Satisfies the minimal ``SubscriberProtocol`` expected by hand and arm
    teleop nodes (the ``read()`` method).  It does *not* implement the full
    ``DataSubscriber`` landscape protocol — use it when you only need typed
    message consumption.

    Args:
        address: ZMQ endpoint to connect to (e.g. ``"tcp://localhost:5555"``).
        topic: Topic prefix bytes to subscribe to.
        msg_type: Dataclass type that has a ``from_dict(d)`` classmethod.
            For list-valued messages use ``read_batch()`` instead of ``read()``.
        timeout_ms: Receive timeout in milliseconds.  ``read()`` returns
            ``None`` after this duration if no message arrives.
    """

    def __init__(
        self,
        address: str,
        topic: bytes,
        msg_type: type[T],
        timeout_ms: int = 1000,
    ) -> None:
        self._address = address
        self._topic = topic
        self._msg_type = msg_type
        self._timeout_ms = timeout_ms

        self._ctx = zmq.Context.instance()
        self._socket: zmq.Socket = self._ctx.socket(zmq.SUB)
        self._socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
        self._socket.setsockopt_string(
            zmq.SUBSCRIBE, topic.decode("utf-8", errors="replace")
        )
        self._socket.connect(address)

        # Metrics
        self._receive_count: int = 0
        self._error_count: int = 0
        self._last_timestamp: float | None = None
        self._connected_at: float = time.time()

        logger.debug(
            f"TopicSubscriber[{msg_type.__name__}] connected to {address} "
            f"on topic {topic!r}"
        )

    # ------------------------------------------------------------------
    # Core read interface
    # ------------------------------------------------------------------

    def read(self) -> T | None:
        """Receive the next message and deserialise to ``T``.

        Blocks for up to ``timeout_ms`` milliseconds.

        Returns:
            A deserialised ``T`` instance, or ``None`` if the socket timed out,
            the ``data`` field was a list (use ``read_batch()`` instead), or
            deserialisation failed.
        """
        raw = self._recv_raw()
        if raw is None:
            return None

        data = raw.get("data")
        if data is None:
            return None

        # If the publisher sent a list, the caller should use read_batch().
        if isinstance(data, list):
            logger.debug(
                f"TopicSubscriber[{self._msg_type.__name__}].read() received a list; "
                "use read_batch() for list-valued topics"
            )
            if len(data) == 1:
                # Convenience: unwrap single-element lists transparently
                data = data[0]
            else:
                return None

        return self._deserialise(data)

    def read_batch(self) -> list[T]:
        """Receive the next message and deserialise all items to a list of ``T``.

        Use this when the publisher sends a list of objects on a single topic
        (e.g. a list of ``RigidPose`` tracker dicts).

        Blocks for up to ``timeout_ms`` milliseconds.

        Returns:
            A list of deserialised ``T`` instances (may be empty on timeout or error).
        """
        raw = self._recv_raw()
        if raw is None:
            return []

        data = raw.get("data")
        if data is None:
            return []

        items = data if isinstance(data, list) else [data]
        results: list[T] = []
        for item in items:
            obj = self._deserialise(item)
            if obj is not None:
                results.append(obj)
        return results

    def close(self) -> None:
        """Close the ZMQ socket.

        The ZMQ context is shared (``Context.instance()``) and is not
        terminated here.
        """
        try:
            self._socket.close(linger=0)
        except zmq.ZMQError:
            pass
        logger.debug(
            f"TopicSubscriber[{self._msg_type.__name__}] closed "
            f"(received={self._receive_count}, errors={self._error_count})"
        )

    # ------------------------------------------------------------------
    # Metrics
    # ------------------------------------------------------------------

    def get_metrics(self) -> dict[str, Any]:
        """Return basic operational metrics.

        Returns:
            Dict with keys: ``address``, ``topic``, ``msg_type``,
            ``receive_count``, ``error_count``, ``last_timestamp``,
            ``uptime_sec``.
        """
        return {
            "address": self._address,
            "topic": self._topic.decode("utf-8", errors="replace"),
            "msg_type": self._msg_type.__name__,
            "receive_count": self._receive_count,
            "error_count": self._error_count,
            "last_timestamp": self._last_timestamp,
            "uptime_sec": round(time.time() - self._connected_at, 2),
        }

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _recv_raw(self) -> dict[str, Any] | None:
        """Receive one multipart message and unpack it.

        Returns:
            Unpacked dict with keys ``topic``, ``timestamp``, ``data``,
            or ``None`` on timeout or error.
        """
        try:
            frames = self._socket.recv_multipart()
            self._receive_count += 1
            unpacked = unpack_data_message(frames)
            self._last_timestamp = unpacked.get("timestamp")
            return unpacked
        except zmq.Again:
            # Timeout — normal, not an error
            return None
        except zmq.ZMQError as exc:
            logger.warning(
                f"TopicSubscriber[{self._msg_type.__name__}] ZMQ error: {exc}"
            )
            self._error_count += 1
            return None
        except Exception as exc:
            logger.warning(
                f"TopicSubscriber[{self._msg_type.__name__}] unpack error: {exc}"
            )
            self._error_count += 1
            return None

    def _deserialise(self, data: Any) -> T | None:
        """Call ``msg_type.from_dict(data)`` and return the result.

        Returns:
            Deserialised ``T`` instance, or ``None`` on error.
        """
        try:
            return self._msg_type.from_dict(data)  # type: ignore[attr-defined]
        except Exception as exc:
            logger.warning(
                f"TopicSubscriber[{self._msg_type.__name__}] deserialise error: {exc}"
            )
            self._error_count += 1
            return None

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> TopicSubscriber[T]:
        return self

    def __exit__(self, *_: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return (
            f"TopicSubscriber[{self._msg_type.__name__}]("
            f"address={self._address!r}, topic={self._topic!r})"
        )


__all__ = ["TopicSubscriber"]
