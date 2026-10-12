"""O que se le de um treino: o status, o relatorio de um periodo, os graficos e
as folhas de quadros (o jogador de verdade, pelo MatterModelInspector)."""
from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path

from . import control, paths, storage


def read_status(run: str) -> dict | None:
    """Do treinador vivo (socket) ou, se ele nao responde, do status.json."""
    reply = control.request("status", timeout=3.0)
    if reply and reply.get("ok") and reply["status"].get("run") == run:
        status = reply["status"]
        status["alive"] = True
        return status
    path = paths.run_dir(run) / "status.json"
    if path.exists():
        status = json.loads(path.read_text())
        status["alive"] = False
        if status.get("state") not in ("parado", "erro"):
            status["state"] = "parado"
        return status
    return None


def hours(seconds: float) -> str:
    seconds = int(seconds or 0)
    return f"{seconds // 3600}h{(seconds % 3600) // 60:02d}"


def format_status(status: dict) -> str:
    last = status.get("last") or {}
    evaluation = status.get("evaluation") or {}
    best = status.get("best") or {}
    lines = [
        f"treino {status['run']} · {status.get('taskName', status.get('task'))} · {status['state']}"
        + ("" if status.get("alive") else " (treinador fechado)"),
        f"modo {status.get('modeLabel', status.get('mode'))} · {status.get('threads')} threads · {status.get('device')}"
        f" · {status.get('environments')} mundos",
        f"rodada {status.get('iteration')} · {status.get('steps', 0) / 1e6:.1f} M decisoes · "
        f"{status.get('sps', 0):.0f} decisoes/s · treinado {hours(status.get('trainSeconds', 0))}",
    ]
    if last:
        ep = last.get("episode_seconds")
        fall = last.get("fall_fraction")
        lines.append(f"ultima rodada: recompensa {last.get('reward', 0):.3f}/passo · episodio "
                     f"{ep:.1f} s" if ep is not None else f"ultima rodada: recompensa {last.get('reward', 0):.3f}")
        if fall is not None:
            lines[-1] += f" · quedas {100 * fall:.0f}% dos fins"
    if evaluation:
        parts = []
        for solver, m in evaluation.get("solvers", {}).items():
            part = (f"{solver}: de pe {100 * m['stand_fraction']:.0f}% ({m['mean_seconds']:.0f} s), "
                    f"oscilacao {100 * m['sway']:.1f} cm")
            if m.get("turn") is not None:
                part += f", giro do tronco {math.degrees(m['turn']):.0f} graus/s"
            parts.append(part)
        lines.append(f"avaliacao (rodada {evaluation.get('iteration')}): nota {evaluation.get('score', 0):.3f} · "
                     + " · ".join(parts))
    if best and best.get("iteration", -1) >= 0:
        lines.append(f"melhor: nota {best['score']:.3f} na rodada {best['iteration']}")
    temperature = status.get("cpuTemperature")
    if temperature:
        lines.append(f"CPU {temperature:.0f} °C · memoria {status.get('memoryMB', 0):.0f} MB")
    if status.get("message"):
        lines.append(status["message"])
    return "\n".join(lines)


def report(run: str, since_seconds: float) -> str:
    """O relatorio de um periodo, em Markdown (o que eu leio na checagem)."""
    db = storage.Database(readonly=True)
    start = time.time() - since_seconds
    rows = db.execute("SELECT * FROM iterations WHERE run=? AND time>=? ORDER BY iteration", (run, start)).fetchall()
    evals = db.execute("""SELECT * FROM evaluations WHERE run=? AND time>=? ORDER BY iteration, solver""",
                       (run, start)).fetchall()
    events = db.execute("SELECT * FROM events WHERE run=? AND time>=? ORDER BY time", (run, start)).fetchall()
    out = [f"# Treino `{run}` — ultimas {since_seconds / 3600:.1f} h", ""]
    if rows:
        first, last = rows[0], rows[-1]
        def mean(key, items):
            values = [r[key] for r in items if r[key] is not None]
            return sum(values) / len(values) if values else float("nan")
        head, tail = rows[: max(1, len(rows) // 5)], rows[-max(1, len(rows) // 5):]
        out += [
            f"- Rodadas {first['iteration']}–{last['iteration']} ({len(rows)}), "
            f"{(last['steps'] - first['steps']) / 1e6:.1f} M decisoes, {mean('sps', rows):.0f} decisoes/s em media.",
            f"- Recompensa por passo: {mean('reward', head):.3f} → {mean('reward', tail):.3f}.",
            f"- Duracao dos episodios: {mean('episode_seconds', head):.1f} s → {mean('episode_seconds', tail):.1f} s; "
            f"quedas nos fins: {100 * mean('fall_fraction', head):.0f}% → {100 * mean('fall_fraction', tail):.0f}%.",
            f"- Entropia {mean('entropy', head):.2f} → {mean('entropy', tail):.2f}; desvio da acao "
            f"{mean('action_std', head):.3f} → {mean('action_std', tail):.3f}; KL {mean('kl', tail):.4f}; "
            f"taxa {last['learning_rate']:.2e}.",
        ]
        components = json.loads(last["components"] or "{}")
        if components:
            out.append("- Parcelas (ultima rodada): " + ", ".join(f"{k} {v:+.3f}" for k, v in components.items()))
        ends: dict[str, int] = {}
        for r in tail:
            for k, v in json.loads(r["ends"] or "{}").items():
                ends[k] = ends.get(k, 0) + v
        if ends:
            out.append("- Fins de episodio (ultimo quinto): " + ", ".join(f"{k} {v}" for k, v in sorted(ends.items())))
        alerts = []
        if mean("entropy", tail) < mean("entropy", head) - 15:
            alerts.append("entropia caindo rapido (exploracao acabando)")
        if mean("kl", tail) > 0.05:
            alerts.append("KL alto (atualizacoes grandes demais)")
        if len(rows) > 50 and abs(mean("reward", tail) - mean("reward", head)) < 0.005:
            alerts.append("recompensa parada (plato)")
        if alerts:
            out.append("- **Alertas:** " + "; ".join(alerts) + ".")
    else:
        out.append("- Nenhuma rodada no periodo.")
    if evals:
        out += ["", "## Avaliacoes", "", "| Rodada | Solver | De pe 60 s | Media (s) | Oscilacao (cm) | Deriva pes (cm) | "
                "Giro do tronco (graus/s) | Mudanca de acao | Nota |", "|---:|---|---:|---:|---:|---:|---:|---:|---:|"]
        for e in evals:
            value = e["turn"] if "turn" in e.keys() else None
            turn = f"{math.degrees(value):.1f}" if value is not None else "—"
            out.append(f"| {e['iteration']} | {e['solver']} | {100 * e['stand_fraction']:.0f}% | {e['mean_seconds']:.1f} | "
                       f"{100 * e['sway']:.1f} | {100 * e['foot_drift']:.1f} | {turn} | {e['action_rate']:.4f} | "
                       f"{e['score']:.3f} |")
    if events:
        out += ["", "## Eventos", ""]
        for e in events:
            out.append(f"- {time.strftime('%d/%m %H:%M', time.localtime(e['time']))} · {e['kind']} · "
                       f"{e['author']}: {e['message']}")
    db.close()
    return "\n".join(out)


def plots(run: str, output: Path) -> list[Path]:
    """PNG das curvas: recompensa, duracao, quedas, avaliacao, PPO, velocidade."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    db = storage.Database(readonly=True)
    rows = db.sampled_iterations(run, 2000)
    evals = db.execute("SELECT * FROM evaluations WHERE run=? ORDER BY iteration", (run,)).fetchall()
    db.close()
    output.mkdir(parents=True, exist_ok=True)
    if not rows:
        return []
    steps = [r["steps"] / 1e6 for r in rows]
    figure, axes = plt.subplots(3, 2, figsize=(13, 10))
    def series(axis, key, title, scale=1.0):
        values = [(s, r[key] * scale) for s, r in zip(steps, rows) if r[key] is not None]
        if values:
            axis.plot([v[0] for v in values], [v[1] for v in values], linewidth=1.0)
        axis.set_title(title)
        axis.set_xlabel("milhoes de decisoes")
        axis.grid(alpha=0.3)
    series(axes[0][0], "reward", "recompensa por passo")
    series(axes[0][1], "episode_seconds", "duracao do episodio (s)")
    series(axes[1][0], "fall_fraction", "fins por queda (%)", 100.0)
    if evals:
        for solver in sorted({e["solver"] for e in evals}):
            points = [(e["steps"] / 1e6, 100 * e["stand_fraction"]) for e in evals if e["solver"] == solver]
            axes[1][1].plot([p[0] for p in points], [p[1] for p in points], marker="o", label=solver)
        axes[1][1].legend()
    axes[1][1].set_title("avaliacao: de pe por 60 s (%)")
    axes[1][1].grid(alpha=0.3)
    series(axes[2][0], "action_std", "desvio da acao")
    series(axes[2][1], "sps", "decisoes por segundo")
    figure.tight_layout()
    main = output / f"{run}-curvas.png"
    figure.savefig(main, dpi=90)
    plt.close(figure)
    return [main]


def sheet(dump: Path, scenario: str | None, output: Path, frames: int = 8, view: str | None = None,
          start: float | None = None, end: float | None = None) -> Path:
    """Folha de quadros de um episodio gravado, com o jogador de verdade
    (start/end: so um trecho, em segundos)."""
    inspector = paths.inspector()
    if inspector is None:
        raise SystemExit("MatterModelInspector ausente: compile a build (tools/build.sh)")
    command = [str(inspector), "quadros", "--dump", str(dump), "--quadros", str(frames), "--colisores", "0",
               "--saida", str(output)]
    if scenario:
        command += ["--cenario", scenario]
    if view:
        command += ["--vista", view]
    if start is not None:
        command += ["--de", str(start)]
    if end is not None:
        command += ["--ate", str(end)]
    subprocess.run(command, check=True, cwd=paths.REPO, stdout=subprocess.DEVNULL)
    return output


def scenarios(dump: Path) -> list[str]:
    names = []
    with open(dump) as handle:
        for line in handle:
            if line.startswith("S "):
                names.append(line[2:].strip())
    return names
