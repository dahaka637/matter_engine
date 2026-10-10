#!/usr/bin/env python3
"""Graficos do rastro da locomocao fisica (XRASTROCSV do harness).

Cada painel desenha colunas do CSV no tempo; os contatos dos pes (f0c, f1c)
aparecem como faixas embaixo de todos os paineis. Sem matplotlib: PIL.

Uso:
  XRASTROCSV=/tmp/r.csv MATTERENGINE_TEST_FILTER=sweepboletim XBOLETIMCASO="zigue sprint" \\
      build-profile/MatterAdaptiveTests
  python3 tools/analysis/locomotion_trace.py /tmp/r.csv saida.png \\
      --paineis "pz,mz|f0z,f1z|chao|ax,ay,az|incl,peito" [--de 1.5 --ate 3.5]

Colunas: t estado cx cy cvx cvy rvx rvy px py pz pvx pvy pvz mx my mz mvx mvy
mvz capx capy f0x f0y f0z f0c f1x f1y f1z f1c ax ay az incl peito chao
envelope cambaleio quedas. Expressoes simples tambem valem (ex.: "pz-mz",
"hypot(pvx,pvy)").
"""

from __future__ import annotations

import argparse
import csv
import math
import sys

from PIL import Image, ImageDraw

COLORS = [(52, 120, 220), (230, 110, 40), (60, 170, 90), (200, 60, 150), (130, 90, 200), (120, 120, 120)]


def split_series(panel: str) -> list[str]:
    """Separa as series de um painel pelas virgulas fora de parenteses."""
    parts, depth, current = [], 0, ""
    for char in panel:
        if char == "," and depth == 0:
            parts.append(current)
            current = ""
            continue
        depth += 1 if char == "(" else -1 if char == ")" else 0
        current += char
    parts.append(current)
    return [part.strip() for part in parts if part.strip()]


def evaluate(expression: str, row: dict) -> float:
    scope = {key: float(value) for key, value in row.items()}
    scope.update({"hypot": math.hypot, "abs": abs, "atan2": math.atan2, "sqrt": math.sqrt})
    return float(eval(expression, {"__builtins__": {}}, scope))


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv")
    parser.add_argument("png")
    parser.add_argument("--paineis", default="pz,mz|f0z,f1z|chao|ax,ay,az|incl,peito")
    parser.add_argument("--de", type=float, default=-1e9)
    parser.add_argument("--ate", type=float, default=1e9)
    parser.add_argument("--largura", type=int, default=1400)
    args = parser.parse_args()
    with open(args.csv) as handle:
        rows = [row for row in csv.DictReader(handle) if args.de <= float(row["t"]) <= args.ate]
    if not rows:
        print("rastro vazio no intervalo", file=sys.stderr)
        return 1
    panels = [split_series(panel) for panel in args.paineis.split("|")]
    width, panel_height, margin = args.largura, 170, 70
    contact_height = 30
    height = margin + len(panels) * (panel_height + 20) + contact_height + 30
    image = Image.new("RGB", (width, height), (255, 255, 255))
    draw = ImageDraw.Draw(image)
    times = [float(row["t"]) for row in rows]
    t0, t1 = times[0], times[-1]
    plot_left, plot_right = 90, width - 20

    def x_of(t: float) -> float:
        return plot_left + (t - t0) / max(1e-6, t1 - t0) * (plot_right - plot_left)

    top = 20
    for panel in panels:
        series = [[evaluate(expression, row) for row in rows] for expression in panel]
        low = min(min(values) for values in series)
        high = max(max(values) for values in series)
        if high - low < 1e-6:
            high, low = high + 0.5, low - 0.5
        pad = (high - low) * 0.05
        low, high = low - pad, high + pad

        def y_of(value: float) -> float:
            return top + panel_height - (value - low) / (high - low) * panel_height

        draw.rectangle([plot_left, top, plot_right, top + panel_height], outline=(200, 200, 200))
        if low < 0.0 < high:
            draw.line([plot_left, y_of(0.0), plot_right, y_of(0.0)], fill=(220, 220, 220))
        draw.text((5, top), f"{high:.2f}", fill=(90, 90, 90))
        draw.text((5, top + panel_height - 12), f"{low:.2f}", fill=(90, 90, 90))
        for index, (expression, values) in enumerate(zip(panel, series)):
            color = COLORS[index % len(COLORS)]
            points = [(x_of(t), y_of(v)) for t, v in zip(times, values)]
            draw.line(points, fill=color, width=2)
            draw.text((plot_left + 8 + 150 * index, top + 4), expression, fill=color)
        top += panel_height + 20
    # Contatos dos pes e as quedas.
    for side, key in enumerate(("f0c", "f1c")):
        y = top + side * (contact_height // 2)
        for t, row in zip(times, rows):
            if int(float(row[key])):
                draw.line([x_of(t), y, x_of(t), y + contact_height // 2 - 3], fill=COLORS[side])
        draw.text((5, y), "pe E" if side == 0 else "pe D", fill=COLORS[side])
    previous = 0
    for t, row in zip(times, rows):
        falls = int(float(row["quedas"]))
        if falls > previous:
            draw.line([x_of(t), 20, x_of(t), top + contact_height], fill=(220, 30, 30), width=2)
        previous = falls
    # Eixo do tempo.
    step = 0.1 if t1 - t0 <= 2.0 else 0.5
    tick = math.ceil(t0 / step) * step
    while tick <= t1:
        draw.line([x_of(tick), top + contact_height, x_of(tick), top + contact_height + 5], fill=(90, 90, 90))
        draw.text((x_of(tick) - 10, top + contact_height + 8), f"{tick:.1f}", fill=(90, 90, 90))
        tick += step
    image.save(args.png)
    print(args.png)
    return 0


if __name__ == "__main__":
    sys.exit(main())
