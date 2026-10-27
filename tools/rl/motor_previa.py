#!/usr/bin/env python3
"""As folhas de quadros do motor (motor-v4): o corpo físico de um treino (a média da política,
sem ruído) ou a ação zero (o IDLE comandado), num exercício, desenhadas pelo MatterModelInspector.

  .venv/bin/python motor_previa.py --treino motor_autonomo --exercicio parado --segundos 8 --saida /tmp/motor
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import torch

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from matter_rl import paths  # noqa: E402
import motor_treino as trainer  # noqa: E402

native = trainer.native


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--treino', default='motor_autonomo')
    parser.add_argument('--dados', type=Path, default=Path(os.environ.get('MATTER_RL_DATA', paths.DATA)))
    parser.add_argument('--zero', action='store_true', help='sem rede: a ação zero (o IDLE comandado)')
    parser.add_argument('--clipe', action='store_true', help='sem rede: a ação é a pose do clipe de referência (o clipe nos motores)')
    parser.add_argument('--exercicio', default='parado', choices=native.drill_ids())
    parser.add_argument('--velocidade', type=float, default=-1., help='trote: velocidade pedida (padrão: o teto do currículo)')
    parser.add_argument('--segundos', type=float, default=8.)
    parser.add_argument('--quadros', type=int, default=16)
    parser.add_argument('--mundos', type=int, default=4, help='os mundos simulados; a folha é de cada um')
    parser.add_argument('--vista', default='direita')
    parser.add_argument('--saida', type=Path, required=True)
    args = parser.parse_args()
    run = args.dados / 'runs' / args.treino
    task = run / 'tarefa.json' if not args.zero and (run / 'tarefa.json').exists() else trainer.ROOT / 'assets/rl/motor/basico.json'
    drill = native.drill_ids().index(args.exercicio)
    speed = args.velocidade
    if speed <= 0 and drill >= 2 and (run / 'curriculo.json').exists():
        speed = json.loads((run / 'curriculo.json').read_text())['limits'][drill - 2]
    batch = native.MotorBatch(str(trainer.ROOT / 'assets'), str(task), worlds=args.mundos, threads=min(4, args.mundos), seed=20261019,
                              drill=drill, evaluation=True, seconds=args.segundos + 1, trot_speed=speed)
    model, who = None, 'ação zero (IDLE comandado)'
    if args.clipe:
        who = 'clipe nos motores (sem rede)'
    elif not args.zero:
        state = torch.load(run / 'ultimo.pt', map_location='cpu', weights_only=False)
        model = trainer.make_model(batch, 'cpu', state['settings']['noise']['scale'])
        model.load_state_dict(state['model'])
        who = f"{args.treino} rodada {state['counters']['rodada']}"
    args.saida.mkdir(parents=True, exist_ok=True)
    dumps = [[f'S mundo{w}'] for w in range(args.mundos)]
    ended = [None] * args.mundos
    names = native.episode_metric_names()
    steps = int(args.segundos * 30)
    for step in range(steps):
        obs = torch.as_tensor(batch.observations())
        with torch.no_grad():
            if args.clipe:
                action = np.asarray(batch.reference_actions(), np.float32) * (1 - batch.clip_drive)  # o motor ja soma clipDrive x clipe
            elif model is None:
                action = np.zeros((args.mundos, batch.action_size), np.float32)
            else:
                action = model.actor(model.obs_norm(obs)).numpy().astype(np.float32)
        batch.step(action)
        for row in batch.episodes():
            w = int(row[0])
            if ended[w] is None:
                values = dict(zip(names, row[2:]))
                ended[w] = (f"{'caiu' if values['caiu'] else 'tempo'} aos {values['segundos']:.1f} s · postura "
                            f"{values['postura']:.2f} · clipe {values['imitacao']:.2f}")
        if step % 2 == 0:
            t = (step + 1) / 30
            for w in range(args.mundos):
                if ended[w] is None:
                    dumps[w] += [f'F {t:.4f} 0', batch.frame_lines(w).rstrip()]
    inspector = paths.inspector()
    for w in range(args.mundos):
        dump = args.saida / f'mundo{w}.dump'
        dump.write_text('\n'.join(dumps[w]) + '\n')
        if inspector is not None:
            subprocess.run([str(inspector), 'quadros', '--dump', str(dump), '--quadros', str(args.quadros), '--colunas', '8',
                            '--largura', '180', '--altura', '260', '--vista', args.vista,
                            '--saida', str(args.saida / f'mundo{w}.png')], check=False, capture_output=True)
        print(f"mundo {w}: {ended[w] or f'de pé até {args.segundos:.0f} s'} · folha {args.saida / f'mundo{w}.png'}")
    print(f'{who} · {args.exercicio}' + (f' a {speed:.2f} m/s' if drill >= 2 else ''))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
