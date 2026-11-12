#!/usr/bin/env python3
"""O freio do treino com o Matter Engine aberto (05/11, pedido do usuario: "quando eu abrir o Matter Engine e o treino
estiver rodando, ele reduza severamente o treino, para nao superaquecer o computador, mas volte ao normal assim que eu
fechar").

A cada 2 s olha se ha um processo MatterEngine (o aplicativo, de qualquer pasta de build). Aberto: limita a CPU do
servico de treino pelo systemd (CPUQuota, so em tempo de execucao - o treino nao para nem perde nada, so anda mais
devagar). Fechado: tira o limite. Ao iniciar, aplica o estado de agora (limpa um limite que ficou de antes).

(08/11 16h15, ele: "colocar um negocio nas suas ferramentas... para quando tu usa elas, elas temporariamente durante a
execucao diminuir a intensidade dos treinos... esses picos geralmente ocorriam porque voce tentava usar as ferramentas de
analise com o treino em execucao".) O mesmo freio vale enquanto houver uma marca de ferramenta viva em
$XDG_RUNTIME_DIR/freio-treino/<pid> (tools/rl/freio_treino.py nas ferramentas Python; tools/sistema/freio <comando> no
resto). Marca de processo morto e apagada e nao conta - o freio nunca fica preso.

Roda como servico do usuario (freio-jogo.service, sempre ligado).

  python3 tools/sistema/freio_jogo.py            # o freio
  python3 tools/sistema/freio_jogo.py --uma      # mostra o estado e sai, sem mexer
  python3 tools/sistema/freio_jogo.py --teste    # o autoteste da logica (nao mexe no servico)
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

INTERVAL = 2.0
SERVICES = ("matter-rl-final-ajuda.service",)
APP = "MatterEngine"
# O treino com o jogo aberto: 1,5 nucleo (de 12) - ~1/7 do normal com 10 threads.
QUOTA_OPEN = "150%"
LOG = Path.home() / ".local/share/matter-engine/freio-jogo.log"
MARKS = Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) / "freio-treino"


def alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


def tools_active(marks: Path = MARKS) -> bool:
    """Ha uma ferramenta pesada rodando (uma marca com o pid de um processo vivo)? As marcas mortas sao apagadas."""
    active = False
    for m in marks.glob("*") if marks.is_dir() else []:
        if m.name.isdigit() and alive(int(m.name)):
            active = True
        else:
            m.unlink(missing_ok=True)
    return active


def app_open() -> bool:
    """Ha um MatterEngine rodando? (o nome exato do processo: as ferramentas MatterModelInspector e os testes nao contam.)"""
    return subprocess.run(["pgrep", "-x", APP], capture_output=True).returncode == 0


def apply(open_: bool, log) -> None:
    quota = QUOTA_OPEN if open_ else ""
    for s in SERVICES:
        r = subprocess.run(["systemctl", "--user", "set-property", "--runtime", s, f"CPUQuota={quota}"],
                           capture_output=True, text=True)
        log(f"{s}: {'freado (CPUQuota=' + quota + ')' if open_ else 'normal (sem limite)'}"
            + (f" FALHOU: {r.stderr.strip()}" if r.returncode else ""))


def decide(last: bool | None, now: bool) -> bool:
    """Mexe so quando o estado muda (e sempre na primeira leitura)."""
    return last is None or last != now


def log_to_file(msg: str) -> None:
    line = f"{time.strftime('%d/%m %H:%M:%S')} {msg}"
    print(line, flush=True)
    try:
        LOG.parent.mkdir(parents=True, exist_ok=True)
        with LOG.open("a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--uma", action="store_true")
    ap.add_argument("--teste", action="store_true")
    a = ap.parse_args()
    if a.teste:
        assert decide(None, False) and decide(None, True)
        assert decide(False, True) and decide(True, False)
        assert not decide(True, True) and not decide(False, False)
        import tempfile
        with tempfile.TemporaryDirectory() as d:
            marks = Path(d)
            assert not tools_active(marks)
            (marks / str(os.getpid())).touch()
            assert tools_active(marks)
            (marks / str(os.getpid())).unlink()
            (marks / "999999999").touch()
            assert not tools_active(marks) and not (marks / "999999999").exists()
        print("freio ok")
        return
    if a.uma:
        print(f"{APP} aberto: {app_open()}; ferramentas: {tools_active()}")
        for s in SERVICES:
            print(subprocess.run(["systemctl", "--user", "show", s, "-p", "CPUQuotaPerSecUSec"],
                                 capture_output=True, text=True).stdout.strip())
        return
    last = None
    while True:
        game, tools = app_open(), tools_active()
        now = game or tools
        if decide(last, now):
            if now: log_to_file("motivo: " + ("jogo aberto" if game else "ferramenta rodando"))
            apply(now, log_to_file)
            last = now
        time.sleep(INTERVAL)


if __name__ == "__main__":
    sys.exit(main())
