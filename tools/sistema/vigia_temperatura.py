#!/usr/bin/env python3
"""O vigia de temperatura (27/10, pedido do usuario depois de um desligamento inesperado).

A cada 3 s le a CPU (o Tctl do Ryzen, k10temp) e a GPU (nvidia-smi). Passando do
limite, corta na hora os testes, as avaliacoes, as bancadas e as compilacoes do
projeto, manda os treinos de RL salvarem e pararem (o SIGTERM do treinador salva o
checkpoint "final") e, em seguida, DESLIGA o computador. Os treinos se recusam a
abrir sem este vigia vivo (o batimento em $XDG_RUNTIME_DIR/vigia-temperatura.json)
e param, salvando, se ele sumir.

Roda como servico do usuario (vigia-temperatura.service, sempre ligado).

  python3 tools/sistema/vigia_temperatura.py                 # o vigia
  python3 tools/sistema/vigia_temperatura.py --uma           # uma leitura e sai
  python3 tools/sistema/vigia_temperatura.py --teste         # a emergencia simulada: corta so
                                                             # processos de teste e nao desliga
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
# Os limites (graus C): passando do de cima em duas leituras seguidas (6 s), ou do
# imediato numa so. Normal no treino: CPU 68-73, GPU 55-66. O Ryzen 5 3600 se protege
# a 95; a GTX 1070 Ti mira 83 e se segura a 96.
CPU_LIMIT, CPU_NOW = 85.0, 90.0
GPU_LIMIT, GPU_NOW = 83.0, 88.0
STRIKES = 2
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


def stop_work(test: bool) -> bool:
    """Corta o pesado, salva e para os treinos. True: os treinos sairam salvos."""
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
    log(f"cortados {len(heavy)} processos pesados; {len(workers)} treino(s) salvando")
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
            "limits": {"cpu": CPU_LIMIT, "cpuNow": CPU_NOW, "gpu": GPU_LIMIT, "gpuNow": GPU_NOW}}
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
    ap.add_argument("--teste", action="store_true",
                    help="a emergencia simulada (CPU acima do limite) contra os processos marcados de teste; nao desliga")
    a = ap.parse_args()
    shutdown = ["systemctl", "poweroff"]
    if a.uma:
        print(json.dumps({"cpu": cpu_temperature(), "gpu": gpu_temperature()}))
        return 0
    log(f"vigia aberto (pid {os.getpid()}): a cada {INTERVAL:.0f} s; CPU {CPU_LIMIT:.0f}/{CPU_NOW:.0f}, "
        f"GPU {GPU_LIMIT:.0f}/{GPU_NOW:.0f} graus" + (" [TESTE]" if a.teste else ""))
    strikes = {"cpu": 0, "gpu": 0}
    blind_since = None
    last_line = 0.0
    while True:
        started = time.time()
        cpu, gpu = cpu_temperature(), gpu_temperature()
        if a.teste:
            cpu = CPU_NOW + 1.0
        reason = None
        for name, value, limit, now in (("cpu", cpu, CPU_LIMIT, CPU_NOW), ("gpu", gpu, GPU_LIMIT, GPU_NOW)):
            strikes[name] = strikes[name] + 1 if value is not None and value >= limit else 0
            if value is not None and (value >= now or strikes[name] >= STRIKES):
                reason = f"{name.upper()} a {value:.1f} graus (limite {limit:.0f}/{now:.0f})"
        state = "ok"
        if reason:
            beat(cpu, gpu, "emergencia")
            emergency(reason, a.teste, shutdown)
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
