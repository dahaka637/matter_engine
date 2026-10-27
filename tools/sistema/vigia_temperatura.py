#!/usr/bin/env python3
"""O vigia de temperatura (27/10, pedido do usuario depois de um desligamento inesperado).

A cada 3 s le a CPU (o Tctl do Ryzen, k10temp) e a GPU (nvidia-smi); uma leitura
basta. No limite de parar, corta os testes, as avaliacoes, as bancadas e as
compilacoes do projeto e manda os treinos de RL salvarem e pararem (o SIGTERM do
treinador salva o checkpoint "final"); o PC fica ligado. No limite de desligar, faz o
mesmo e, em seguida, DESLIGA o computador. Os treinos se recusam a
abrir sem este vigia vivo (o batimento em $XDG_RUNTIME_DIR/vigia-temperatura.json)
e param, salvando, se ele sumir.

Roda como servico do usuario (vigia-temperatura.service, sempre ligado).

  python3 tools/sistema/vigia_temperatura.py                 # o vigia
  python3 tools/sistema/vigia_temperatura.py --uma           # uma leitura e sai
  python3 tools/sistema/vigia_temperatura.py --teste parar   # o limite de parar simulado
  python3 tools/sistema/vigia_temperatura.py --teste desligar  # o de desligar simulado (nao desliga)
  (os testes so cortam os processos marcados vigia-teste-alvo)
"""
from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from pathlib import Path

INTERVAL = 3.0
# Os limites (graus C, os do usuario em 27/10; uma leitura basta): parar (salva e para
# o trabalho, o PC fica ligado) e desligar (salva, para e desliga o PC). Normal no
# treino: CPU 68-73 (picos ate ~78), GPU 55-66.
CPU_STOP, CPU_OFF = 83.0, 85.0
GPU_STOP, GPU_OFF = 80.0, 83.0
# Sem ler a temperatura por 30 s com trabalho pesado aberto: salva e para (nao desliga).
BLIND_SECONDS = 30.0
# Quanto os treinos tem para salvar antes do corte e do desligamento.
SAVE_SECONDS = 40.0

RUNTIME = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
HEARTBEAT = RUNTIME / "vigia-temperatura.json"
LOG = Path.home() / ".local" / "share" / "matter-engine" / "vigia_temperatura.log"

# O trabalho pesado do projeto (o que o vigia corta na hora; os treinos saem a parte),
# pelo executavel e pelo script de cada processo - nunca pelo texto da linha inteira
# (um terminal com "matter_rl treinar" escrito no meio do comando nao e um treino).
HEAVY_EXE = re.compile(r"^(Matter\w+|ninja|cc1plus|cc1|ld|lto1|lto-wrapper)$")
HEAVY_SCRIPT = re.compile(r"^(avaliar|previa_\w+|hash_locomocao|vazao|rastreador_\w+|prova|checagem\w*|ciclo\w*"
                          r"|troca\w*|folhas\w*)\.(py|sh)$")
TEST_MARK = "vigia-teste-alvo"


def kind(argv: list[str]) -> str:
    """"treino" (o treinador ou o supervisor dele), "pesado" ou ""."""
    exe = os.path.basename(argv[0])
    if exe.startswith("python") and "matter_rl" in argv:
        after = argv[argv.index("matter_rl") + 1:argv.index("matter_rl") + 2]
        if after and after[0] in ("treinar", "servir"):
            return "treino"
    if HEAVY_EXE.match(exe):
        return "pesado"
    if (exe.startswith("python") or exe in ("bash", "sh", "env")) \
            and any(HEAVY_SCRIPT.match(os.path.basename(a)) for a in argv[1:3]):
        return "pesado"
    return ""


def log(text: str) -> None:
    line = f"{time.strftime('%d/%m %H:%M:%S')} {text}"
    print(line, flush=True)
    try:
        LOG.parent.mkdir(parents=True, exist_ok=True)
        with LOG.open("a") as out:
            out.write(line + "\n")
    except OSError:
        pass


def notify(text: str) -> None:
    try:
        subprocess.run(["notify-send", "-u", "critical", "Vigia de temperatura", text], timeout=3,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except (OSError, subprocess.SubprocessError):
        pass


def cpu_temperature() -> float | None:
    best = None
    for hwmon in Path("/sys/class/hwmon").glob("hwmon*"):
        try:
            if (hwmon / "name").read_text().strip() not in ("k10temp", "coretemp", "zenpower"):
                continue
        except OSError:
            continue
        for sensor in hwmon.glob("temp*_input"):
            try:
                value = int(sensor.read_text()) / 1000.0
            except (OSError, ValueError):
                continue
            best = value if best is None else max(best, value)
    return best


def gpu_temperature() -> float | None:
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=temperature.gpu", "--format=csv,noheader,nounits"],
                             capture_output=True, text=True, timeout=2.5)
        values = [float(v) for v in out.stdout.split() if v.strip()]
        return max(values) if values else None
    except (OSError, ValueError, subprocess.SubprocessError):
        return None


def processes() -> list[tuple[int, list[str]]]:
    """Os processos do usuario: (pid, argv)."""
    me, uid, found = os.getpid(), os.getuid(), []
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == me:
            continue
        try:
            if entry.stat().st_uid != uid:
                continue
            argv = [a.decode(errors="replace") for a in (entry / "cmdline").read_bytes().split(b"\0") if a]
        except OSError:
            continue
        if argv:
            found.append((int(entry.name), argv))
    return found


def kill(pids: list[int], sig: int) -> None:
    for pid in pids:
        try:
            os.kill(pid, sig)
        except OSError:
            pass


def alive(pids: list[int]) -> list[int]:
    out = []
    for pid in pids:
        try:
            os.kill(pid, 0)
            out.append(pid)
        except OSError:
            pass
    return out


def signal_work(test: bool) -> tuple[list[int], list[int], list[int]]:
    """Manda o SIGTERM: o pesado sai na hora, os treinos salvam e saem. (treinos, supervisores, pesado)."""
    procs = processes()
    if test:
        procs = [(p, a) for p, a in procs if TEST_MARK in " ".join(a)]
    heavy = [p for p, a in procs if kind(a) == "pesado"]
    kill(heavy, signal.SIGTERM)
    # O supervisor (servir) primeiro: sem ele, o treino que sai nao e reaberto.
    supervisors = [p for p, a in procs if kind(a) == "treino" and "servir" in a]
    workers = [p for p, a in procs if kind(a) == "treino" and "treinar" in a]
    kill(supervisors, signal.SIGTERM)
    kill(workers, signal.SIGTERM)
    if heavy or workers:
        log(f"cortados {len(heavy)} processos pesados; {len(workers)} treino(s) salvando")
    return workers, supervisors, heavy


def stop_work(test: bool) -> bool:
    """Corta o pesado, salva e para os treinos e espera (ate SAVE_SECONDS). True: os treinos sairam salvos."""
    workers, supervisors, heavy = signal_work(test)
    deadline = time.time() + SAVE_SECONDS
    while time.time() < deadline and alive(workers + heavy):
        time.sleep(0.5)
    left = alive(workers + heavy + supervisors)
    if left:
        log(f"{len(left)} processo(s) nao sairam em {SAVE_SECONDS:.0f} s: SIGKILL "
            "(o treino fica com o ultimo checkpoint periodico, de no maximo 10 min)")
        kill(left, signal.SIGKILL)
    return not alive(workers) and not (set(workers) & set(left))


def heavy_running() -> bool:
    return any(kind(a) for _, a in processes())


def beat(cpu: float | None, gpu: float | None, state: str) -> None:
    data = {"time": time.time(), "cpu": cpu, "gpu": gpu, "state": state, "pid": os.getpid(),
            "limits": {"cpuStop": CPU_STOP, "cpuOff": CPU_OFF, "gpuStop": GPU_STOP, "gpuOff": GPU_OFF}}
    tmp = HEARTBEAT.with_suffix(".tmp")
    tmp.write_text(json.dumps(data))
    tmp.replace(HEARTBEAT)


def emergency(reason: str, test: bool, shutdown: list[str]) -> None:
    log(f"EMERGENCIA: {reason}")
    notify(f"{reason}: salvando, parando tudo e desligando")
    saved = stop_work(test)
    log("treinos salvos" if saved else "treinos parados (ver acima)")
    if test:
        log("teste: o desligamento seria agora: " + " ".join(shutdown))
        return
    log("desligando: " + " ".join(shutdown))
    subprocess.run(shutdown)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--uma", action="store_true", help="uma leitura e sai")
    ap.add_argument("--teste", choices=["parar", "desligar"],
                    help="o limite simulado (uma leitura) contra os processos marcados de teste; nunca desliga")
    a = ap.parse_args()
    shutdown = ["systemctl", "poweroff"]
    if a.uma:
        print(json.dumps({"cpu": cpu_temperature(), "gpu": gpu_temperature()}))
        return 0
    log(f"vigia aberto (pid {os.getpid()}): a cada {INTERVAL:.0f} s, uma leitura basta; parar CPU {CPU_STOP:.0f} "
        f"GPU {GPU_STOP:.0f}, desligar CPU {CPU_OFF:.0f} GPU {GPU_OFF:.0f} graus" + (f" [TESTE {a.teste}]" if a.teste else ""))
    blind_since = None
    last_line = 0.0
    # Os que receberam o SIGTERM no limite de parar: o prazo para sair (depois, SIGKILL).
    pending: dict[int, float] = {}
    first = True
    while True:
        started = time.time()
        cpu, gpu = cpu_temperature(), gpu_temperature()
        if a.teste and first:
            cpu = (CPU_OFF if a.teste == "desligar" else CPU_STOP) + 0.5
        first = False
        state = "ok"
        off = [f"{n} a {v:.1f} graus (desliga em {lim:.0f})" for n, v, lim in (("CPU", cpu, CPU_OFF), ("GPU", gpu, GPU_OFF))
               if v is not None and v >= lim]
        stop = [f"{n} a {v:.1f} graus (para em {lim:.0f})" for n, v, lim in (("CPU", cpu, CPU_STOP), ("GPU", gpu, GPU_STOP))
                if v is not None and v >= lim]
        if off:
            beat(cpu, gpu, "desligando")
            emergency(", ".join(off), bool(a.teste), shutdown)
            return 0
        if stop:
            state = "quente: parando o trabalho"
            workers, supervisors, heavy = signal_work(bool(a.teste))
            if workers or heavy:
                log(f"LIMITE DE PARAR: {', '.join(stop)}")
                notify(f"{', '.join(stop)}: treinos salvos e parados (o PC fica ligado)")
            for pid in workers + supervisors + heavy:
                pending.setdefault(pid, started + SAVE_SECONDS)
        for pid, deadline in list(pending.items()):
            if not alive([pid]):
                del pending[pid]
            elif started >= deadline:
                log(f"o processo {pid} nao saiu em {SAVE_SECONDS:.0f} s: SIGKILL")
                kill([pid], signal.SIGKILL)
                del pending[pid]
        if a.teste and not first and not pending:
            log("teste: o trabalho marcado parou e o vigia segue (o PC ficaria ligado)")
            return 0
        if cpu is None or gpu is None:
            blind_since = blind_since or started
            state = "sem leitura"
            if started - blind_since >= BLIND_SECONDS and heavy_running():
                log(f"sem ler a temperatura ha {started - blind_since:.0f} s (CPU {cpu}, GPU {gpu}): "
                    "salvando e parando o trabalho pesado (sem desligar)")
                notify("Sem leitura de temperatura: treinos salvos e parados")
                stop_work(False)
                blind_since = started
        else:
            blind_since = None
        beat(cpu, gpu, state)
        if started - last_line >= 60.0:
            log(f"CPU {cpu} GPU {gpu} ({state})")
            last_line = started
        time.sleep(max(0.0, INTERVAL - (time.time() - started)))


if __name__ == "__main__":
    sys.exit(main())
