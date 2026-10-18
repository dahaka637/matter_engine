#!/usr/bin/env python3
"""Um A/B curto da locomocao (vault 04_locomocao/referencia/pesquisas/08_plano_consenso.md
§1.1 e §4.2): dois bracos do MESMO checkpoint (controle e mudanca), a mesma semente e as
mesmas rodadas, os pesos dos exercicios congelados, um depois do outro; no fim, as medidas
das ultimas rodadas de cada um (lado a lado) e a bateria determinista do portao em cada um.

A especificacao (JSON):
  {"nome": "q1_piso",
   "checkpoint": "<pasta de um checkpoint>",            # o ponto de partida (protegido)
   "fonte": "loco_p2v",                                 # o treino de onde vem o run.json e a tarefa
   "tarefa": "locomocao_f2t",                           # opcional: a tarefa dos assets em vez da copia
   "rodadas": 600,
   "comum": {"run": {...}, "tarefa": {...}, "exercicios": {"<id>": {...}}},  # o que os dois bracos levam
   "mudanca": {"run": {...}, "tarefa": {...}},          # so o braco B
   "ajuste_b": "recentrar" | "renormalizar" | null,     # um ajuste fora do laco antes do B
   "ajuste_comum": "renormalizar" | null}               # antes dos dois (a mesma partida)
Os dicionarios se fundem por chave (recursivo) no run.json e no tarefa.json do braco.

  .venv/bin/python ab_locomocao.py spec.json [--avaliar 64]
"""
from __future__ import annotations

import argparse
import copy
import json
import shutil
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths, storage  # noqa: E402


def merge(base: dict, extra: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        out[key] = merge(out.get(key, {}), value) if isinstance(value, dict) and isinstance(out.get(key), dict) else value
    return out


def make_arm(spec: dict, name: str, change: bool) -> str:
    run = f"ab_{spec['nome']}_{name}"
    target = paths.run_dir(run)
    if target.exists():
        raise SystemExit(f"o braco {run} ja existe: de outro nome ao A/B (nada e apagado)")
    target.mkdir(parents=True)
    source = paths.run_dir(spec["fonte"])
    checkpoint = Path(spec["checkpoint"])
    iteration = int(json.loads((checkpoint / "manifest.json").read_text())["iteration"])
    shutil.copytree(checkpoint, target / "checkpoints" / f"ck-{iteration:09d}")
    storage.write_json_atomic(target / "latest.json", {"iteration": iteration, "path": f"ck-{iteration:09d}"})
    config = json.loads((source / "run.json").read_text())
    task = json.loads((paths.task_path(spec["tarefa"]) if spec.get("tarefa") else source / "tarefa.json").read_text())
    task.setdefault("drillWeights", {})["auto"] = False  # os bracos nao divergem pela mistura
    parts = [spec.get("comum", {})] + ([spec.get("mudanca", {})] if change else [])
    for part in parts:
        config = merge(config, part.get("run", {}))
        task = merge(task, part.get("tarefa", {}))
        # Campos de um exercicio pelo id (a lista de exercicios nao se funde por chave).
        for drill in task.get("drills", []):
            drill.update(part.get("exercicios", {}).get(drill["id"], {}))
    config["end_reason"] = ""
    storage.write_json_atomic(target / "run.json", config)
    storage.write_json_atomic(target / "tarefa.json", task)
    return run


def matter_rl(*args: str) -> int:
    return subprocess.call([str(HERE / "matter-rl"), *args])


def train(run: str, rounds: int) -> None:
    reference = storage.latest_checkpoint(paths.run_dir(run))
    start = int(storage.load_checkpoint(reference, "cpu").get("counters", {}).get("iteration", reference.iteration))
    code = matter_rl("treinar", "--run", run, "--rodadas", str(start + rounds))
    if code != 0:
        raise SystemExit(f"o braco {run} falhou ({code})")


def measures(run: str, last: int = 100) -> dict:
    db = storage.Database(paths.DATABASE, readonly=True)
    rows = db.execute("SELECT details, fall_fraction, learning_rate, kl, clip_fraction, action_std, sps "
                      "FROM iterations WHERE run=? ORDER BY iteration DESC LIMIT ?", (run, last)).fetchall()
    if not rows:
        return {}
    details = [json.loads(r[0]) for r in rows]
    mean = lambda values: float(sum(values) / len(values)) if values else None  # noqa: E731
    out = {"quedas_treino": mean([r[1] for r in rows if r[1] is not None]), "taxa": mean([r[2] for r in rows]),
           "kl": mean([r[3] for r in rows]), "corte": mean([r[4] for r in rows]), "desvio": mean([r[5] for r in rows]),
           "decisoes_s": mean([r[6] for r in rows])}
    diag = [d.get("diag", {}) for d in details if d.get("diag")]
    if diag:
        groups = diag[0].get("saturation", {}).keys()
        out["saturacao"] = {g: mean([d["saturation"][g] for d in diag if g in d.get("saturation", {})]) for g in groups}
        out["trancos_media"] = mean([d["jerk_mean_action"] for d in diag if d.get("jerk_mean_action") is not None])
        top: dict = {}
        for d in diag:
            for name, value in d.get("kl_top5", []):
                top[name] = top.get(name, 0.0) + value / len(diag)
        out["kl_top"] = sorted(top.items(), key=lambda item: -item[1])[:5]
    drills: dict = {}
    for d in details:
        for drill, s in d.get("drills", {}).items():
            acc = drills.setdefault(drill, {"episodes": 0, "falls": 0.0, "speed": 0.0, "requested": 0.0})
            n = s["episodes"]
            acc["episodes"] += n
            acc["falls"] += s["falls"] * n
            acc["speed"] += s["speed"] * n
            acc["requested"] += s["requested"] * n
    out["exercicios"] = {k: {"quedas": v["falls"] / v["episodes"], "feita": v["speed"] / v["episodes"],
                             "pedida": v["requested"] / v["episodes"]} for k, v in drills.items() if v["episodes"]}
    return out


def gate(run: str, worlds: int) -> str:
    result = subprocess.run([str(HERE / ".venv" / "bin" / "python"), str(HERE / "portao_locomocao.py"), "--treino", run,
                             "--mundos", str(worlds), "--threads", "12"], capture_output=True, text=True)
    return "\n".join(line for line in result.stdout.splitlines() if not line.startswith("[INFO]"))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("spec")
    parser.add_argument("--avaliar", type=int, default=64, help="episodios da bateria por exercicio (0: sem)")
    args = parser.parse_args()
    spec = json.loads(Path(args.spec).read_text())
    report = {"spec": spec, "inicio": time.strftime("%Y-%m-%d %H:%M"), "bracos": {}}
    for name, change in (("a", False), ("b", True)):
        if not change and spec.get("controle_de"):
            # O controle identico de outro A/B (o mesmo checkpoint, a mesma config, as mesmas
            # rodadas): "nome" (o braco a dele) ou "nome:b".
            name_of, _, arm_of = spec["controle_de"].partition(":")
            previous = json.loads((HERE / "ab" / f"{name_of}.json").read_text())
            report["bracos"]["a"] = dict(previous["bracos"][arm_of or "a"], reaproveitado_de=spec["controle_de"])
            continue
        run = make_arm(spec, name, change)
        for adjust in ([spec.get("ajuste_comum")] if spec.get("ajuste_comum") else []) + \
                ([spec["ajuste_b"]] if change and spec.get("ajuste_b") else []):
            iteration = storage.latest_checkpoint(paths.run_dir(run)).iteration
            if matter_rl(adjust, "--run", run, "--rodada", str(iteration)) != 0:
                raise SystemExit(f"o ajuste {adjust} nao passou na prova de equivalencia ({run})")
        begin = time.time()
        train(run, int(spec["rodadas"]))
        report["bracos"][name] = {"run": run, "minutos": (time.time() - begin) / 60.0, "medidas": measures(run)}
        if args.avaliar > 0:
            report["bracos"][name]["portao"] = gate(run, args.avaliar)
        out = HERE / "ab" / f"{spec['nome']}.json"
        out.parent.mkdir(exist_ok=True)
        out.write_text(json.dumps(report, indent=1, ensure_ascii=False))
    print(json.dumps({k: v["medidas"] for k, v in report["bracos"].items()}, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
