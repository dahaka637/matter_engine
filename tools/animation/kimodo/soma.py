"""SOMA77 (Kimodo): nomes, ossos, centro de massa e conversao para z-up."""
import re, numpy as np
_bvh = open(__import__('os').environ.get('KIMODO', '/home/dahaka/Projetos/kimodo') + '/kimodo/assets/skeletons/somaskel77/somaskel77_standard_tpose.bvh').read()
NAMES = re.findall(r'^\s*(?:ROOT|JOINT)\s+(\S+)', _bvh, re.M)[1:]
IX = {n: i for i, n in enumerate(NAMES)}
BONES = []
for a, b, r in [('Hips','Spine1',0.11),('Spine1','Spine2',0.12),('Spine2','Chest',0.13),('Chest','Neck1',0.07),('Neck1','Neck2',0.05),('Neck2','Head',0.05),('Head','HeadEnd',0.09)]:
    BONES.append((a, b, r))
for s in ('Left', 'Right'):
    BONES += [(f'{s}Shoulder', f'{s}Arm', 0.05), (f'{s}Arm', f'{s}ForeArm', 0.05), (f'{s}ForeArm', f'{s}Hand', 0.04),
              (f'{s}Hand', f'{s}HandMiddle2', 0.035), (f'{s}Leg', f'{s}Shin', 0.07), (f'{s}Shin', f'{s}Foot', 0.05),
              (f'{s}Foot', f'{s}ToeBase', 0.045), (f'{s}ToeBase', f'{s}ToeEnd', 0.035), ('Hips', f'{s}Leg', 0.08)]
# segmentos (de Leva, aprox.): (a, b, massa)
SEGMENTS = [('Hips','Spine1',0.14),('Spine1','Chest',0.16),('Chest','Neck1',0.20),('Neck1','HeadEnd',0.081)]
for s in ('Left','Right'):
    SEGMENTS += [(f'{s}Arm',f'{s}ForeArm',0.028),(f'{s}ForeArm',f'{s}Hand',0.016),(f'{s}Hand',f'{s}HandMiddle2',0.006),
                 (f'{s}Leg',f'{s}Shin',0.10),(f'{s}Shin',f'{s}Foot',0.0465),(f'{s}Foot',f'{s}ToeEnd',0.0145)]
def zup(p):
    """Kimodo (x, y para cima, z) -> nosso (x, y, z para cima)."""
    p = np.asarray(p)
    return np.stack([p[..., 0], -p[..., 2], p[..., 1]], axis=-1)
def com(P):
    """P: (77,3) y-up -> centro de massa (3,)."""
    tot = sum(m for _, _, m in SEGMENTS)
    return sum(m * 0.5 * (P[IX[a]] + P[IX[b]]) for a, b, m in SEGMENTS) / tot
