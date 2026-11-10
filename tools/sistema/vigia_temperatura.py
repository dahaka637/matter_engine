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
  echo 85.5 > $XDG_RUNTIME_DIR/vigia-simular                # o teste de verdade no servico: entre o parar e o desligar so PARA
                                                            # (e religa ao esfriar); no desligar ou acima, DESLIGA o PC
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
# (05/11 ~11h, ele: "vamos redefinir o nosso monitor termico, para parar os treinos em 85 graus ao inves de 83. E
# desligamento emergencial em 88 graus ao inves de 85".)
CPU_STOP, CPU_OFF = 85.0, 88.0
CPU_BASE_STOP, CPU_BASE_OFF = CPU_STOP, CPU_OFF
GPU_STOP, GPU_OFF = 80.0, 83.0
# (02/11 22h20, ele: "Hoje em especifico vamos aumentar os limites das temperatura, para 85 o fechar o treino e salvar
# e 87 desligamento emergencial" - a cidade muito quente; 03/11 13h10, de novo ate 04/11 12h): os da CPU ate HOT_UNTIL,
# depois voltam sozinhos aos de cima.
CPU_HOT = (85.0, 87.0)
# (04/11 06h30, ele: "uma revisao geral para deixarmos rodando com seguranca de evolucao": os treinos (servicos do systemd
# --user "matter-rl*") que o vigia parou no limite de parar voltam sozinhos quando a CPU fica abaixo de RESUME_CPU por
# RESUME_SECONDS - no maximo RESUME_PER_HOUR vezes por hora; um "Parar" manual nunca volta sozinho.)
RESUME_CPU, RESUME_SECONDS, RESUME_PER_HOUR = 70.0, 60.0, 4
RESUME = Path.home() / ".local/share/matter-engine/rl/retomar_apos_esfriar.json"
HOT_UNTIL = time.mktime((2026, 11, 4, 12, 0, 0, 0, 0, -1))


def cpu_limits(now: float) -> tuple[float, float]:
    return CPU_HOT if now < HOT_UNTIL else (CPU_BASE_STOP, CPU_BASE_OFF)
# Sem ler a temperatura por 30 s com trabalho pesado aberto: salva e para (nao desliga).
BLIND_SECONDS = 30.0
# Quanto os treinos tem para salvar antes do corte e do desligamento.
SAVE_SECONDS = 40.0

RUNTIME = Path(os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"))
HEARTBEAT = RUNTIME / "vigia-temperatura.json"
# O teste de verdade (27/10): uma temperatura neste arquivo vira a leitura da CPU uma
# vez, no proprio servico (o caminho real, ate o desligamento); o arquivo some ao ser lido.
SIMULATE = RUNTIME / "vigia-simular"
LOG = Path.home() / ".local" / "share" / "matter-engine" / "vigia_temperatura.log"

# O trabalho pesado do projeto (o que o vigia corta na hora; os treinos saem a parte),
# pelo executavel e pelo script de cada processo - nunca pelo texto da linha inteira
# (um terminal com "matter_rl treinar" escrito no meio do comando nao e um treino).
HEAVY_EXE = re.compile(r"^(Matter\w+|ninja|cc1plus|cc1|ld|lto1|lto-wrapper)$")
HEAVY_SCRIPT = re.compile(r"^(avaliar|previa_\w+|hash_locomocao|vazao|rastreador_\w+|prova|checagem\w*|ciclo\w*"
                          r"|troca\w*|folhas\w*|levantar_\w+|bench_b21|bench_rl_final)\.(py|sh)$")
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


def started_after(unit: str, when: float | None) -> bool:
    """A unidade foi ligada depois do instante `when` (epoch, s)? (O journal do usuario: o systemd zera os carimbos da
    unidade parada.) Sem o instante (registro antigo): nao. Sem conseguir ler: sim (na duvida, nao religa)."""
    if when is None:
        return False
    try:
        r = subprocess.run(["journalctl", "--user", "-u", unit, "--since", f"@{int(when) + 1}", "-o", "cat", "--no-pager"],
                           capture_output=True, text=True, timeout=15)
        return r.returncode != 0 or any(l.startswith("Started") for l in r.stdout.splitlines())
    except (OSError, subprocess.SubprocessError):
        return True


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


def active_trainings() -> list[str]:
    """Os servicos de treino ligados agora (systemd --user, "matter-rl*")."""
    try:
        out = subprocess.run(["systemctl", "--user", "list-units", "--type=service", "--state=active", "--plain",
                              "--no-legend", "matter-rl*"], capture_output=True, text=True, timeout=10).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [line.split()[0] for line in out.splitlines() if line.strip()]


def remember_trainings(units: list[str]) -> None:
    """Anota os treinos que o vigia parou (para voltarem quando esfriar)."""
    if not units:
        return
    try:
        old = json.loads(RESUME.read_text()).get("units", []) if RESUME.exists() else []
        RESUME.parent.mkdir(parents=True, exist_ok=True)
        RESUME.write_text(json.dumps({"units": sorted(set(old) | set(units)), "parado": time.time()}))
    except (OSError, ValueError):
        pass


def main() -> int:
    global CPU_STOP, CPU_OFF
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
    cold_since = None
    resumes: list[float] = []
    first = True
    while True:
        started = time.time()
        if cpu_limits(started) != (CPU_STOP, CPU_OFF):
            CPU_STOP, CPU_OFF = cpu_limits(started)
            log(f"limites da CPU agora: parar {CPU_STOP:.0f}, desligar {CPU_OFF:.0f} graus")
        cpu, gpu = cpu_temperature(), gpu_temperature()
        try:
            fake = float(SIMULATE.read_text())
            SIMULATE.unlink()
            log(f"SIMULACAO: a CPU lida como {fake:.1f} graus (a real: {cpu})")
            cpu = fake
        except (OSError, ValueError):
            pass
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
            if not a.teste:
                remember_trainings(active_trainings())
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
        # (O religamento dos treinos parados pelo vigia: frio por RESUME_SECONDS, sem nada pendente.)
        if not a.teste and RESUME.exists() and not pending and cpu is not None and gpu is not None:
            cold_since = cold_since or started if cpu < RESUME_CPU and gpu < GPU_STOP - 10.0 else None
            resumes = [t for t in resumes if started - t < 3600.0]
            if cold_since is not None and started - cold_since >= RESUME_SECONDS and len(resumes) < RESUME_PER_HOUR:
                try:
                    saved = json.loads(RESUME.read_text())
                    units = saved.get("units", [])
                except (OSError, ValueError):
                    saved, units = {}, []
                RESUME.unlink(missing_ok=True)
                # (05/11 19h22: o vigia religou um treino que o usuario tinha pausado depois da parada termica. Nao religa
                # o que alguem ligou depois da parada do vigia - quem ligou e parou de novo pausou de proposito.)
                units = [u for u in units if not started_after(u, saved.get("parado"))]
                for unit in units:
                    subprocess.run(["systemctl", "--user", "start", unit], timeout=30)
                resumes.append(started)
                cold_since = None
                log(f"esfriou (CPU {cpu:.1f}): treinos religados: {', '.join(units) or 'nenhum'}")
                notify(f"CPU a {cpu:.0f} graus: treinos religados")
        beat(cpu, gpu, state)
        if started - last_line >= 60.0:
            log(f"CPU {cpu} GPU {gpu} ({state})")
            last_line = started
        time.sleep(max(0.0, INTERVAL - (time.time() - started)))


if __name__ == "__main__":
    sys.exit(main())
