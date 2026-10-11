#!/usr/bin/env python3
# (Ferramenta da sonda: le um --dump do MatterAthleteProbe.)
# uso: pe.py <dump> <de> <ate> [link...] - velocidade/aceleracao dos links no
# referencial da pelve no chao (frente = rumo da pelve) e no mundo.
import sys, math

path, t0, t1 = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
links = sys.argv[4:] or ["LeftFoot", "RightFoot"]
frames = []
cur = None
for line in open(path):
    p = line.split()
    if not p:
        continue
    if p[0] == "F":
        cur = {"t": float(p[1])}
        frames.append(cur)
    elif p[0] in ("B", "C") and cur is not None:
        cur[p[1]] = (float(p[2]), float(p[3]), float(p[4]))
frames = [f for f in frames if t0 - 0.05 <= f["t"] <= t1 + 0.05]

def sub(a, b): return (a[0] - b[0], a[1] - b[1], a[2] - b[2])
def mul(a, k): return (a[0] * k, a[1] * k, a[2] * k)
def norm(a): return math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)

for name in links:
    print(f"== {name}")
    pos = [f[name] for f in frames]
    ts = [f["t"] for f in frames]
    vel = [None] + [mul(sub(pos[i], pos[i - 1]), 1.0 / (ts[i] - ts[i - 1])) for i in range(1, len(pos))]
    acc = [None, None] + [mul(sub(vel[i], vel[i - 1]), 1.0 / (ts[i] - ts[i - 1])) for i in range(2, len(pos))]
    jerk = [None] * 3 + [mul(sub(acc[i], acc[i - 1]), 1.0 / (ts[i] - ts[i - 1])) for i in range(3, len(pos))]
    peakA = peakJ = 0.0
    for i in range(3, len(pos)):
        if not (t0 <= ts[i] <= t1):
            continue
        a, j = norm(acc[i]), norm(jerk[i])
        peakA, peakJ = max(peakA, a), max(peakJ, j)
        pel = frames[i]["Pelvis"]
        rel = sub(pos[i], (pel[0], pel[1], 0.0))
        print(f"t {ts[i]:.3f} pe {pos[i][0]:+.3f} {pos[i][1]:+.3f} {pos[i][2]:.3f} | rel pelve {rel[0]:+.3f} {rel[1]:+.3f}"
              f" | v {norm(vel[i]):5.2f} (vz {vel[i][2]:+.2f}) a {a:6.1f} j {j:7.0f}")
    print(f"pico aceleracao {peakA:.1f} m/s2, pico tranco {peakJ:.0f} m/s3")
