#!/usr/bin/env python3
"""Boletim da locomocao fisica (v2 do equilibrio, 05/10 noite).

Roda a bateria do harness (MATTERENGINE_TEST_FILTER=sweepboletim) em varios
niveis de ajuda ao mesmo tempo (um processo por nivel), monta a tabela
cenario x nivel e guarda o resultado em JSON - para comparar uma versao com
outra (--comparar).

Cada cenario roda em variantes (XBOLETIMVARIANTES, padrao 3: os comandos
atrasados k x 43 ms). Celula da tabela: "ok" ou quedas/rodadas (ex.: 2/3),
mais s<n> com saltinhos (voos sem pulo em que o centro de massa sobe 2 cm
alem do voo natural da passada na mesma velocidade, somados), p com tempo pendurado (> 0,25 s sem pe
apoiado), !<n> com quase-quedas (sem queda, mas a pelve alem de 45 graus ou o
peito alem de 50) e v<n> com o corpo lento (abaixo de 80% da velocidade
pedida, em media, so com pedido).

Uso:
  python3 tools/analysis/locomotion_report.py [--niveis 1,0.7,0.5,0.3,0.15,0]
      [--caso texto] [--env CHAVE=VALOR ...] [--salvar arquivo.json]
      [--comparar outro.json] [--binario build-profile/MatterAdaptiveTests]
      [--variantes 6] [--partes N]
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

LINE = re.compile(
    r"^BOLETIM (?P<level>[\d.]+) \| (?P<name>.+?)\s+\| quedas (?P<falls>\d+) \(\s*(?P<first>-?[\d.]+) s\)"
    r" \| saltinhos (?P<hops>\d+) voo max (?P<flight>[\d.]+) s \| pendurado (?P<hang>[\d.]+) s"
    r" \| pelve acima\s+(?P<rise>[\d.]+) cm \| incl\s+(?P<tilt>\d+) peito\s+(?P<chest>\d+)"
    r" \| ajuda\s+(?P<assist>\d+)% plano\s+(?P<planar>\d+)%(?: \| vel\s+(?P<speed>\d+)%)?")


def run_levels(binary: str, levels: list[str], case: str | None, extra: dict[str, str],
               parts: int = 1, variants: int | None = None) -> dict:
    processes: dict[str, list] = {}
    for level in levels:
        for part in range(parts):
            env = dict(os.environ)
            env.update(extra)
            env["MATTERENGINE_TEST_FILTER"] = "sweepboletim"
            env["XBOLETIMNIVEL"] = level
            env["XBOLETIMPARTE"] = f"{part}/{parts}"
            if variants:
                env["XBOLETIMVARIANTES"] = str(variants)
            if case:
                env["XBOLETIMCASO"] = case
            processes.setdefault(level, []).append(subprocess.Popen(
                [binary], env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True))
    results: dict[str, dict] = {}
    for level, group in processes.items():
        output = "\n".join(process.communicate()[0] for process in group)
        rows = {}
        for line in output.splitlines():
            match = LINE.match(line)
            if not match:
                continue
            values = match.groupdict()
            rows[values["name"].strip()] = {
                "falls": int(values["falls"]), "first_fall": float(values["first"]),
                "hops": int(values["hops"]), "flight_peak": float(values["flight"]),
                "hang": float(values["hang"]), "pelvis_rise_cm": float(values["rise"]),
                "tilt": int(values["tilt"]), "chest": int(values["chest"]),
                "assist": int(values["assist"]), "planar": int(values["planar"]),
                "speed": int(values["speed"]) if values.get("speed") else 100,
            }
        results[level] = rows
    return results


# Quase-queda: sem queda, mas a pelve passou de 45 graus ou o peito de 50 (a
# corrida limpa fica em 16-22) - segurado pela ajuda, nao recuperado.
NEAR_TILT, NEAR_CHEST = 45, 50


def near_fall(row: dict) -> bool:
    return row["falls"] == 0 and (row["tilt"] > NEAR_TILT or row["chest"] > NEAR_CHEST)


def base_name(name: str) -> str:
    return name.split(" #")[0]


def grouped(rows: dict) -> dict:
    """Junta as variantes de cada cenario: rodadas, quedas, saltinhos..."""
    groups: dict[str, dict] = {}
    for name, row in rows.items():
        group = groups.setdefault(base_name(name), {"runs": 0, "fallen": 0, "near": 0, "hops": 0, "hang": 0.0,
                                                    "assist": 0.0, "planar": 0.0})
        group["runs"] += 1
        group["fallen"] += 1 if row["falls"] > 0 else 0
        group["near"] += 1 if near_fall(row) else 0
        group["hops"] += row["hops"]
        group["hang"] += row["hang"]
        group["assist"] += row["assist"]
        group["planar"] += row["planar"]
        group["speed"] = group.get("speed", 0) + row.get("speed", 100)
    return groups


def cell(group: dict | None) -> str:
    if group is None:
        return "-"
    text = "ok" if group["fallen"] == 0 else f"{group['fallen']}/{group['runs']}"
    if group["near"] > 0:
        text += f" !{group['near']}"
    if group["hops"] > 0:
        text += f" s{group['hops']}"
    if group["hang"] > 0.0:
        text += " p"
    # Lento: o corpo a menos de 80% da velocidade pedida, em media.
    speed = group.get("speed", 100 * group["runs"]) / max(1, group["runs"])
    if speed < 80:
        text += f" v{speed:.0f}"
    return text


ALLOWED = "(pode cair)"


def totals(rows: dict) -> dict:
    allowed = {k: r for k, r in rows.items() if ALLOWED in k}
    rows = {k: r for k, r in rows.items() if ALLOWED not in k}
    count = len(rows)
    fallen = sum(1 for r in rows.values() if r["falls"] > 0)
    return {
        "runs": count,
        "fallen": f"{fallen}/{count}",
        "fallen_count": fallen,
        "allowed": f"{sum(1 for r in allowed.values() if r['falls'] > 0)}/{len(allowed)}",
        "near": sum(1 for r in rows.values() if near_fall(r)),
        "hops": sum(r["hops"] for r in rows.values()),
        "hang": round(sum(r["hang"] for r in rows.values()), 2),
        "assist": round(sum(r["assist"] for r in rows.values()) / count, 1) if count else 0.0,
        "planar": round(sum(r["planar"] for r in rows.values()) / count, 1) if count else 0.0,
        "speed": round(sum(r.get("speed", 100) for r in rows.values()) / count, 1) if count else 0.0,
    }


def print_table(results: dict, levels: list[str]) -> None:
    names: list[str] = []
    for level in levels:
        for name in results.get(level, {}):
            if base_name(name) not in names:
                names.append(base_name(name))
    groups = {level: grouped(results.get(level, {})) for level in levels}
    width = max([len(n) for n in names] + [10])
    header = "cenario".ljust(width) + " | " + " | ".join(f"{float(l) * 100:>5.0f}%".ljust(13) for l in levels)
    print(header)
    print("-" * len(header))
    for name in names:
        print(name.ljust(width) + " | " + " | ".join(cell(groups[l].get(name)).ljust(13) for l in levels))
    print("-" * len(header))
    for key, label in (("fallen", "rodadas com queda"), ("allowed", "derrubadas (pode cair)"),
                       ("near", "quase-quedas (!)"), ("hops", "saltinhos"),
                       ("hang", "pendurado (s)"),
                       ("assist", "ajuda media (% do peso)"), ("planar", "ajuda no plano (%)"),
                       ("speed", "velocidade (% da pedida)")):
        print(label.ljust(width) + " | " + " | ".join(str(totals(results.get(l, {}))[key]).ljust(13) for l in levels))


def print_comparison(results: dict, baseline: dict, levels: list[str]) -> None:
    print("\nComparacao (agora - referencia):")
    for level in dict.fromkeys(levels):
        if level not in baseline or level not in results:
            continue
        now, before = totals(results[level]), totals(baseline[level])
        print(f"  {float(level) * 100:5.0f}%: quedas {before['fallen']} -> {now['fallen']}, quase "
              f"{before.get('near', '?')} -> {now['near']}, saltinhos "
              f"{before['hops']} -> {now['hops']}, pendurado {before['hang']} -> {now['hang']} s, ajuda "
              f"{before['assist']} -> {now['assist']}%")
        now_groups, old_groups = grouped(results[level]), grouped(baseline[level])
        for name, group in now_groups.items():
            old = old_groups.get(name)
            if old and old["fallen"] != group["fallen"]:
                print(f"      {name}: {old['fallen']}/{old['runs']} -> {group['fallen']}/{group['runs']}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--niveis", default="1,0.7,0.5,0.3,0.15,0")
    parser.add_argument("--caso")
    parser.add_argument("--env", action="append", default=[])
    parser.add_argument("--salvar")
    parser.add_argument("--comparar")
    parser.add_argument("--binario", default="build-profile/MatterAdaptiveTests")
    parser.add_argument("--partes", type=int, default=0,
                        help="processos por nivel (padrao: os nucleos divididos pelos niveis)")
    parser.add_argument("--variantes", type=int)
    args = parser.parse_args()
    levels = [level.strip() for level in args.niveis.split(",") if level.strip()]
    extra = dict(item.split("=", 1) for item in args.env)
    if not Path(args.binario).exists():
        print(f"binario nao encontrado: {args.binario}", file=sys.stderr)
        return 1
    parts = args.partes or max(1, (os.cpu_count() or 4) // len(levels))
    results = run_levels(args.binario, levels, args.caso, extra, parts, args.variantes)
    print_table(results, levels)
    if args.comparar and Path(args.comparar).exists():
        print_comparison(results, json.loads(Path(args.comparar).read_text()), levels)
    if args.salvar:
        Path(args.salvar).write_text(json.dumps(results, indent=1, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
