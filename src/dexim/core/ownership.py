"""One host-local reservation shared by session and maintenance entry points."""

from __future__ import annotations

import hashlib
import os
import tempfile
import uuid
from pathlib import Path


class EndpointOwnership:
    """Reserve a resolved physical identity; never infer it from a logical alias.

    The parent may delegate its token to one isolated DDS worker. A delegate
    cannot remove the parent's reservation. Crash leftovers require investigation.
    """

    def __init__(
        self,
        physical_robot_id: str,
        *,
        namespace: str,
        delegated_token: str | None = None,
    ):
        if (
            not isinstance(physical_robot_id, str)
            or not physical_robot_id
            or physical_robot_id.strip() != physical_robot_id
        ):
            raise ValueError("A resolved, nonempty physical robot identity is required")
        if namespace not in {"unitree", "nova", "inspire"}:
            raise ValueError("Unknown ownership namespace")
        identity = hashlib.sha256(physical_robot_id.encode()).hexdigest()
        self.path = Path(tempfile.gettempdir()) / f"dexim-{namespace}-{identity}.lock"
        self.token = delegated_token or uuid.uuid4().hex
        self._delegated = delegated_token is not None
        self._fd = None
        self.active = False

    def acquire(self):
        if self.active:
            raise RuntimeError("Ownership is already reserved")
        if self._delegated:
            if self.path.read_text() != self.token:
                raise RuntimeError("Worker ownership token does not match reservation")
        else:
            self._fd = os.open(
                self.path,
                os.O_CREAT | os.O_EXCL | os.O_WRONLY,
                0o600,
            )
            self.active = True
            try:
                if os.write(self._fd, self.token.encode()) != len(self.token):
                    raise OSError("Incomplete ownership token; reservation retained")
            except BaseException:
                os.close(self._fd)
                self._fd = None
                raise
        self.active = True

    def release(self):
        if not self.active:
            return
        if not self._delegated:
            if self.path.read_text() != self.token:
                raise RuntimeError("Ownership changed; reservation cannot be released")
            if self._fd is not None:
                os.close(self._fd)
                self._fd = None
            self.path.unlink()
        self.active = False


def nova_identity(host: str) -> str:
    import ipaddress
    import socket

    try:
        resolved = ipaddress.ip_address(host)
    except ValueError:
        addresses = {
            item[4][0] for item in socket.getaddrinfo(host, None, socket.AF_INET)
        }
        if len(addresses) != 1:
            raise ValueError("Nova host must resolve to one physical IPv4 endpoint")
        resolved = ipaddress.ip_address(addresses.pop())
    if resolved.version != 4:
        raise ValueError("Supervised Nova requires a resolved IPv4 endpoint")
    return f"tcp:{resolved}"


def inspire_identity(port: str) -> str:
    import re

    name = port.upper().removeprefix("\\\\.\\")
    if re.fullmatch(r"COM[1-9][0-9]*", name):
        return "serial:" + name
    return "serial:" + os.path.normcase(str(Path(port).resolve()))
