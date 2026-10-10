"""npz do Kimodo -> dump do render_pose.py (capsulas entre as juntas). uso: k2dump.py motion.npz saida.txt [passo]"""
import sys, math, numpy as np
from soma import IX, BONES, zup
J = zup(np.load(sys.argv[1])['posed_joints'])
step = int(sys.argv[3]) if len(sys.argv) > 3 else 3
floor = 0.0
def quat_x_to(d):
    d = d / (np.linalg.norm(d) + 1e-12); x = np.array([1.0, 0, 0])
    c = np.cross(x, d); w = 1.0 + float(np.dot(x, d))
    if w < 1e-6: return np.array([0, 0, 1.0, 0])
    q = np.array([c[0], c[1], c[2], w]); return q / np.linalg.norm(q)
with open(sys.argv[2], 'w') as f:
    f.write('S kimodo\n')
    for k in range(0, len(J), step):
        f.write(f"F {k/30:.4f} 0 0\n")
        P = J[k]
        for a, b, r in BONES:
            pa, pb = P[IX[a]], P[IX[b]]
            c = 0.5 * (pa + pb); d = pb - pa; L = np.linalg.norm(d)
            q = quat_x_to(d)
            name = ('Left' if 'Left' in a + b else 'Right' if 'Right' in a + b else '') + a
            f.write(f"C {name} {c[0]:.4f} {c[1]:.4f} {c[2]-floor:.4f} {q[0]:.5f} {q[1]:.5f} {q[2]:.5f} {q[3]:.5f} {max(0.0, L/2):.4f} {r:.4f} {r:.4f}\n")
