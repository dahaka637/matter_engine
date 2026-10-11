#!/usr/bin/env python3
# (Ferramenta da sonda: le um --dump do MatterAthleteProbe.)
# uso: pes.py <dump> <de> <ate> [passo] - os dois pes: altura do centro do link e posicao a frente da pelve (no rumo dela), lado a lado.
import sys, math
path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
step = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
frames = []; cur = None
for line in open(path):
    p = line.split()
    if not p: continue
    if p[0] == "F": cur = {"t": float(p[1])}; frames.append(cur)
    elif p[0] in ("B", "C") and cur is not None: cur[p[1]] = tuple(float(x) for x in p[2:5])
frames = [f for f in frames if t0 <= f["t"] <= t1]
last = -1e9; prev = None
for f in frames:
    if f["t"] - last < step - 1e-6: continue
    last = f["t"]
    pel = f["Pelvis"]
    if prev is not None:
        d = (pel[0]-prev[0], pel[1]-prev[1]); n = math.hypot(*d) or 1.0; fwd = (d[0]/n, d[1]/n)
    else: fwd = (1.0, 0.0)
    prev = pel
    out = []
    for k in ("LeftFoot", "RightFoot"):
        x = f[k]; rel = (x[0]-pel[0], x[1]-pel[1])
        out.append(f"{k[:-4]:5s} z {x[2]:.3f} frente {rel[0]*fwd[0]+rel[1]*fwd[1]:+.2f}")
    print(f"t {f['t']:.3f} pelve z {pel[2]:.3f} | " + " | ".join(out))
