"""Battle events for the MAVProxy map (``mavproxy_arena.py``): match start and kills.

The orchestrator and the map run in different processes (the map inside
MAVProxy), so events travel as small JSON datagrams over local UDP. Nothing
waits for an answer: without a map the datagrams are simply lost.
"""

from __future__ import annotations

import json
import socket
from typing import Any


class MapEvents:
    def __init__(self, port: int, host: str = "127.0.0.1") -> None:
        self.address = (host, port)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _send(self, event: dict[str, Any]) -> None:
        try:
            self._socket.sendto(json.dumps(event).encode("utf-8"), self.address)
        except OSError:
            pass  # no map listening: the battle goes on without explosions

    def start(self) -> None:
        """A new battle: the map shows every drone again and removes old explosions."""
        self._send({"event": "start"})

    def kill(self, sysid: int, by: int | None, lat: float, lon: float) -> None:
        """Drone ``sysid`` was destroyed (by drone ``by``) at ``lat``, ``lon``."""
        self._send({"event": "kill", "sysid": sysid, "by": by, "lat": lat, "lon": lon})

    def close(self) -> None:
        self._socket.close()
