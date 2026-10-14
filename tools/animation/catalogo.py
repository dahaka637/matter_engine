#!/usr/bin/env python3
"""O catalogo dos clipes importados (assets/animations/clips): por tipo de
movimento, com a duracao, se e ciclo, a velocidade, a direcao, de onde veio e
onde e usado (as tarefas do RL e o personagem do jogo).

  python3 tools/animation/catalogo.py   # grava assets/animations/CATALOGO.md
"""

import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[2]
CLIPS = ROOT / "assets" / "animations" / "clips"
OUTPUT = ROOT / "assets" / "animations" / "CATALOGO.md"

# O tipo pelo id do clipe (a primeira regra que bate), na ordem das pastas das fontes.
TYPES = [
    ("Parado", lambda i: i.endswith("_idle")),
    ("Giros parado", lambda i: i in ("turn_left", "turn_right")),
    ("Andar", lambda i: "walk" in i),
    ("Arrancar e frear", lambda i: i in ("idle_to_sprint", "crouched_to_sprint") or "stop" in i),
    ("Curvas e cortes", lambda i: "arc" in i or "turn" in i or i == "change_direction"),
    ("Trote", lambda i: i.startswith(("jog", "trot", "mixamo_running"))),
    ("Corrida e sprint", lambda i: "sprint" in i or "run_backward" in i),
    ("Pulos", lambda i: i.startswith("jump")),
    ("Chão e levantar", lambda i: i.startswith("stand_up") or "getting_up" in i),
    ("Futebol", lambda i: i.startswith("tackle")),
]


def kind(clip_id: str) -> str:
    return next((name for name, rule in TYPES if rule(clip_id)), "Outros")


def usage() -> dict[str, list[str]]:
    """Onde cada arquivo de clipe e citado: as tarefas do RL e o personagem."""
    places = list((ROOT / "assets" / "rl" / "tasks").glob("*.json"))
    places += list((ROOT / "assets" / "characters").glob("*/character.json"))
    used: dict[str, list[str]] = {}
    for place in sorted(places):
        text = place.read_text()
        label = place.stem if place.parent.name == "tasks" else f"personagem ({place.parent.name})"
        for clip in CLIPS.glob("*.matteranim.json"):
            clip_id = clip.name.removesuffix(".matteranim.json")
            if clip.name in text or f'"{clip_id}"' in text:
                used.setdefault(clip_id, []).append(label)
    return used


def source(data: dict) -> str:
    path = str(data.get("sourceAssetPath") or "")
    report = data.get("retargetReport") or {}
    if report.get("origin") == "authored" or path.startswith("tools/animation"):
        return f"autoral (`{path.split(' ')[0]}`)" if path else "autoral"
    return f"`{path.split('#')[0]}`" if path else "—"


def main() -> int:
    used = usage()
    rows: dict[str, list[str]] = {}
    for clip in sorted(CLIPS.glob("*.matteranim.json")):
        data = json.loads(clip.read_text())
        clip_id = clip.name.removesuffix(".matteranim.json")
        speed = data.get("nominalSpeedMetersPerSecond")
        direction = data.get("travelDirectionDegrees")
        row = (f"| `{clip_id}` | {data.get('displayName', '')} | {float(data.get('durationSeconds', 0)):.2f} | "
               f"{'sim' if data.get('loops') else 'não'} | {f'{speed:.1f}' if speed else '—'} | "
               f"{f'{direction:.0f}°' if direction is not None else '—'} | {source(data)} | "
               f"{', '.join(used.get(clip_id, [])) or '—'} |")
        rows.setdefault(kind(clip_id), []).append(row)
    lines = [
        "# Catálogo dos clipes de animação",
        "",
        "Gerado por `tools/animation/catalogo.py` (não editar à mão; rodar de novo depois",
        "de importar um clipe). Os arquivos ficam em `assets/animations/clips/`. As fontes",
        "(os FBX, por tipo) ficam em `assets/animations/source/` e o inventário delas em",
        "`assets/animations/source/INVENTARIO.md`.",
        "",
        "**Ciclo:** o clipe repete (trotes, sprints) ou começa e termina (cortes, pulos,",
        "levantar). **Vel.:** a velocidade dele (m/s). **Dir.:** a direção em que anda,",
        "em relação à frente (0° frente, 90° esquerda, 180° costas). **Usado em:** as",
        "tarefas do RL e o personagem do jogo que citam o arquivo.",
        "",
    ]
    order = [name for name, _ in TYPES] + ["Outros"]
    for name in order:
        if name not in rows:
            continue
        lines += [f"## {name}", "", "| Clipe | Nome | s | Ciclo | Vel. | Dir. | Fonte | Usado em |",
                  "|---|---|---:|---|---:|---:|---|---|", *rows[name], ""]
    OUTPUT.write_text("\n".join(lines))
    print(f"catalogo: {OUTPUT} ({sum(len(r) for r in rows.values())} clipes)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
