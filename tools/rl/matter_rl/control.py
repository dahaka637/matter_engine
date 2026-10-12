"""O canal de controle: um socket Unix local com uma linha JSON por comando.

O treinador atende; a tela do Matter Engine e a CLI pedem. Comandos:
status, pausar, retomar, parar, salvar, modo, avaliar, peso, empurroes, gravar.
"""
from __future__ import annotations

import json
import os
import queue
import socket
import socketserver
import threading
from pathlib import Path
from typing import Any, Callable

from . import paths

PROTOCOL = 1


class _Handler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        for raw in self.rfile:
            try:
                request = json.loads(raw.decode("utf-8"))
                command = str(request.get("cmd", ""))
                arguments = request.get("args", {}) or {}
                reply = self.server.dispatch(command, arguments)  # type: ignore[attr-defined]
            except Exception as error:  # noqa: BLE001 - o erro volta para quem pediu
                reply = {"ok": False, "error": str(error)}
            reply.setdefault("protocol", PROTOCOL)
            self.wfile.write((json.dumps(reply, ensure_ascii=False) + "\n").encode("utf-8"))
            self.wfile.flush()


class _Server(socketserver.ThreadingMixIn, socketserver.UnixStreamServer):
    daemon_threads = True
    allow_reuse_address = True


class ControlServer:
    """`status` responde na hora (de uma copia do estado); os outros comandos
    vao para a fila que o laco do treino esvazia entre os passos."""

    def __init__(self, status: Callable[[], dict], path: Path = paths.SOCKET):
        self.path = path
        self.commands: "queue.Queue[tuple[str, dict, queue.Queue]]" = queue.Queue()
        self._status = status
        path.parent.mkdir(parents=True, exist_ok=True)
        os.chmod(path.parent, 0o700)
        if path.exists():
            if ping(path):
                raise RuntimeError("ja existe um treinador atendendo neste PC")
            path.unlink()
        self.server = _Server(str(path), _Handler)
        self.server.dispatch = self._dispatch  # type: ignore[attr-defined]
        os.chmod(path, 0o600)
        self.thread = threading.Thread(target=self.server.serve_forever, name="controle", daemon=True)
        self.thread.start()

    def _dispatch(self, command: str, arguments: dict) -> dict:
        if command in ("status", "ping"):
            return {"ok": True, "status": self._status()}
        answer: "queue.Queue[dict]" = queue.Queue(maxsize=1)
        self.commands.put((command, arguments, answer))
        try:
            return answer.get(timeout=30.0)
        except queue.Empty:
            return {"ok": True, "queued": True}

    def pending(self):
        """Os comandos que chegaram, para o laco atender."""
        while True:
            try:
                yield self.commands.get_nowait()
            except queue.Empty:
                return

    def close(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        try:
            self.path.unlink()
        except FileNotFoundError:
            pass


def request(command: str, arguments: dict | None = None, path: Path = paths.SOCKET,
            timeout: float = 35.0) -> dict | None:
    """Manda um comando ao treinador; None se nao ha treinador atendendo."""
    if not path.exists():
        return None
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
            client.settimeout(timeout)
            client.connect(str(path))
            client.sendall((json.dumps({"cmd": command, "args": arguments or {}}) + "\n").encode("utf-8"))
            data = b""
            while not data.endswith(b"\n"):
                chunk = client.recv(65536)
                if not chunk:
                    break
                data += chunk
        return json.loads(data.decode("utf-8")) if data else None
    except (OSError, json.JSONDecodeError):
        return None


def ping(path: Path = paths.SOCKET) -> bool:
    reply = request("ping", path=path, timeout=2.0)
    return bool(reply and reply.get("ok"))


def reply(answer: "queue.Queue[dict]", payload: dict[str, Any]) -> None:
    try:
        answer.put_nowait(payload)
    except queue.Full:
        pass
