#!/usr/bin/env python3
# (Ferramenta da sonda: le um --dump do MatterAthleteProbe.)
# uso: voo.py <dump> [limiar] - o maior tempo com os dois pes no ar (centro do link do pe acima do limiar, m) antes do
# toque (1.0..1.6 s, o normal da corrida) e depois dele (1.6..2.6 s).
import sys
path = sys.argv[1]; lim = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
frames = []; cur = None
for line in open(path):
    p = line.split()
    if not p: continue
    if p[0] == "F": cur = {"t": float(p[1])}; frames.append(cur)
    elif p[0] in ("B", "C") and cur is not None and p[1] in ("LeftFoot", "RightFoot"): cur[p[1]] = float(p[4])
def longest(t0, t1):
    best = run = 0.0; start = None; prev = None
    for f in frames:
        if not (t0 <= f["t"] <= t1) or "LeftFoot" not in f: continue
        air = f["LeftFoot"] > lim and f["RightFoot"] > lim
        if air:
            if start is None: start = f["t"]
            run = f["t"] - start
            best = max(best, run)
        else: start = None
    return best
print(f"voo max antes {longest(1.0, 1.6):.3f} s | depois {longest(1.6, 2.6):.3f} s")
