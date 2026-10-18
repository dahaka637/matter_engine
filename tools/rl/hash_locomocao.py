#!/usr/bin/env python3
"""A prova bit a bit do modulo do treino (matter_locomotion_native, o binario
que treina; vault 02_estrutura/estudo_desempenho.md, item pipeline-21): semente,
acoes pseudoaleatorias e autoridade fixas, N passos do lote, e o hash por passo
de tudo o que o modulo devolve (observacoes, critico, critico terminal,
recompensas e componentes, fins e episodios terminados). Roda com 1 e 12
threads (tem de dar o mesmo) e confere contra um ouro do mesmo build.

  .venv/bin/python hash_locomocao.py --gravar ouro.json      # antes da mudanca
  .venv/bin/python hash_locomocao.py --conferir ouro.json    # depois: sai 0 se igual
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths  # noqa: E402
from matter_rl.locomotion_trainer import native  # noqa: E402


def run(module, task: Path, worlds: int, threads: int, steps: int, seed: int) -> list[str]:
    batch = module.LocomotionBatch(str(task), str(paths.ASSETS), environments=worlds, threads=threads, seed=seed)
    batch.set_authority(1.0, 1.0, 1.0, 1.0)
    batch.reset(seed)
    rng = np.random.default_rng(seed)
    digests = []
    for _ in range(steps):
        actions = np.ascontiguousarray(rng.uniform(-0.05, 0.05, (worlds, batch.action_size)).astype(np.float32))
        batch.step(actions)
        h = hashlib.sha256()
        for part in (batch.observations(), batch.critic_observations(), batch.terminal_critic_observations(),
                     batch.rewards(), batch.reward_components(), batch.terminated(), batch.truncated(),
                     batch.finished_episodes()):
            h.update(np.ascontiguousarray(np.asarray(part)).tobytes())
        digests.append(h.hexdigest()[:16])
    return digests


def main() -> int:
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--gravar", help="grava o ouro neste arquivo")
    group.add_argument("--conferir", help="confere contra este ouro")
    parser.add_argument("--tarefa", default="locomocao_f2t")
    parser.add_argument("--mundos", type=int, default=64)
    parser.add_argument("--passos", type=int, default=150)
    parser.add_argument("--semente", type=int, default=20261017)
    parser.add_argument("--threads", default="1,12")
    args = parser.parse_args()

    from matter_rl import system
    system.refuse_tuned_environment("hash")
    module = native()
    task = paths.task_path(args.tarefa)
    result: dict = {"tarefa": args.tarefa, "mundos": args.mundos, "passos": args.passos, "semente": args.semente,
                    "por_threads": {}}
    for threads in [int(t) for t in args.threads.split(",")]:
        begin = time.perf_counter()
        result["por_threads"][str(threads)] = run(module, task, args.mundos, threads, args.passos, args.semente)
        print(f"threads {threads}: {time.perf_counter() - begin:.1f} s, final {result['por_threads'][str(threads)][-1]}",
              flush=True)
    runs = list(result["por_threads"].values())
    same = all(r == runs[0] for r in runs)
    print("1 e N threads: " + ("iguais" if same else "DIFERENTES"))
    if args.gravar:
        Path(args.gravar).write_text(json.dumps(result, indent=1))
        print(f"ouro gravado: {args.gravar}")
        return 0 if same else 1
    gold = json.loads(Path(args.conferir).read_text())
    ok = same
    for threads, digests in result["por_threads"].items():
        ref = gold["por_threads"].get(threads)
        if ref is None:
            continue
        if digests != ref:
            first = next(i for i, (a, b) in enumerate(zip(digests, ref)) if a != b) if len(digests) == len(ref) else 0
            print(f"threads {threads}: DIFERE do ouro a partir do passo {first}")
            ok = False
        else:
            print(f"threads {threads}: igual ao ouro")
    print("HASH: " + ("igual" if ok else "DIFERENTE"))
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
