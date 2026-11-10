"""O freio do treino pelas ferramentas (08/11 16h15, pedido dele): enquanto este processo vive, a marca
$XDG_RUNTIME_DIR/freio-treino/<pid> faz o freio-jogo (tools/sistema/freio_jogo.py) limitar a CPU do treino; a marca sai
no fim (e, se o processo morrer sem sair, o freio a ignora). Nao importar no proprio treino (levantar_amp.py).
MATTER_SEM_FREIO=1 desliga."""
from __future__ import annotations

import atexit
import os
import subprocess
import time
from pathlib import Path

MARKS = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "freio-treino"
SERVICE = "matter-rl-final-ajuda.service"


def braked() -> bool:
    out = subprocess.run(["systemctl", "--user", "show", SERVICE, "-p", "CPUQuotaPerSecUSec"],
                         capture_output=True, text=True).stdout
    return "infinity" not in out


def acquire(wait: float = 5.0) -> None:
    """Marca e espera (ate wait s) o limite entrar, para o trabalho pesado comecar ja com o treino freado."""
    if os.environ.get("MATTER_SEM_FREIO"):
        return
    MARKS.mkdir(parents=True, exist_ok=True)
    mark = MARKS / str(os.getpid())
    mark.touch()
    atexit.register(lambda: mark.unlink(missing_ok=True))
    if subprocess.run(["systemctl", "--user", "is-active", "--quiet", SERVICE]).returncode != 0:
        return
    end = time.time() + wait
    while time.time() < end and not braked():
        time.sleep(0.25)
