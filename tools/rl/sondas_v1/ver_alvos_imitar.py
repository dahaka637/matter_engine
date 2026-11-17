import sys, numpy as np, torch
sys.path.insert(0, '.')
from inspecao import comum
amb = comum.Ambiente('cerebro_unico', sys.argv[1])
for key in ('lev_levantar_costas', 'lev_levantar_brucos'):
    b = amb.batch(key, 4)
    b.set_get_up_imitation(1, 1.5)
    model = amb.model(b); b.reset(55); dt = float(b.policy_dt)
    nomes = list(b.action_names()); iq = nomes.index('LeftThigh.swing1') if 'LeftThigh.swing1' in nomes else 0
    linhas = []
    for k in range(int(1.8 / dt)):
        obs = torch.as_tensor(b.observations())
        with torch.no_grad():
            act = model.env_action(model.distribution(model.norm(obs)).mean, obs).numpy().astype(np.float32)
        alvo = np.asarray(b.locomotion_targets()).reshape(4, -1)
        if k % 6 == 0:
            linhas.append(f"t {k*dt:.2f} alvos!=0 em {int((np.abs(alvo).sum(1) > 0).sum())}/4 mundos | coxa esq alvo {alvo[0, iq]:+.2f} rede {act[0, iq]:+.2f} | fase {float(obs[0, -1]):.0f}")
        b.step(act)
    print(f"== {key}"); print('\n'.join(linhas))
