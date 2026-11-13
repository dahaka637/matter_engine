"""One policy file, independent locomotion/recovery heads; recovery normalization frozen, locomotion its own (03/11)."""
import torch
from torch import nn
from rastreador_treino import RunningNorm

# (03/11: a normalizacao da locomocao nasce com a media e o desvio da recuperacao mas com este peso de contagem - umas
# poucas rodadas de locomocao bastam para ela ir para as velocidades do trote.)
LOCOMOTION_NORM_COUNT = 1.0e4


class SkillHead(nn.Module):
    def __init__(self, observations, outputs, factory):
        super().__init__()
        self.recovery = factory(observations, outputs)
        self.locomotion = factory(observations, outputs)
        self.outputs = outputs

    def forward(self, x):
        shape = x.shape[:-1]
        x = x.reshape(-1, x.shape[-1])
        moving = x[:, -1] > .5
        out = x.new_zeros((len(x), self.outputs))
        for mask, head in ((moving, self.locomotion), (~moving, self.recovery)):
            if mask.any():
                out[mask] = head(x[mask])
        return out.reshape(*shape, self.outputs)


class SplitNorm(nn.Module):
    """A normalizacao de cada cabeca pela ultima entrada (a habilidade, que passa crua): a da recuperacao como veio
    (congelada - mudar as estatisticas dela muda a rede do levantar) e a da locomocao, atualizada so com as linhas dela
    (03/11: com a da recuperacao, as velocidades do trote chegavam a ~2,7 desvios e passavam do corte em ate 16%)."""

    def __init__(self, observations):
        super().__init__()
        self.recovery = RunningNorm(observations)
        self.locomotion = RunningNorm(observations)

    def forward(self, x):
        moving = x[..., -1:] > .5
        out = torch.where(moving, self.locomotion(x), self.recovery(x))
        return torch.cat([out[..., :-1], x[..., -1:]], -1)

    def update(self, x, recovery=False, locomotion=True):
        """(11/11 14h) Por padrao so a da locomocao, como sempre; recovery=True: a da recuperacao tambem aprende (a cabeca do
        levantar recomecada do zero - --recuperacao-norma); locomotion=False: a da locomocao parada (--congelar-locomocao)."""
        rows = x.reshape(-1, x.shape[-1])
        moving = rows[:, -1] > .5
        if locomotion and int(moving.sum()) > 1:
            self.locomotion.update(rows[moving])
        if recovery and int((~moving).sum()) > 1:
            self.recovery.update(rows[~moving])


def split_state(state):
    """Um estado de rede unica (ou ja separada so nas cabecas) nas duas cabecas e nas duas normalizacoes."""
    out = {}
    for name, value in state.items():
        value = value.clone() if torch.is_tensor(value) else value
        if name.startswith(('actor.', 'critic.')) and not name.startswith(('actor.recovery.', 'actor.locomotion.',
                                                                          'critic.recovery.', 'critic.locomotion.')):
            prefix, rest = name.split('.', 1)
            out[f'{prefix}.recovery.{rest}'] = value
            out[f'{prefix}.locomotion.{rest}'] = value.clone()
        elif name in ('norm.mean', 'norm.var', 'norm.count'):
            rest = name.split('.', 1)[1]
            out[f'norm.recovery.{rest}'] = value
            out[f'norm.locomotion.{rest}'] = (torch.tensor(LOCOMOTION_NORM_COUNT, dtype=value.dtype)
                                              if rest == 'count' else value.clone())
        else:
            out[name] = value
    if 'loco_log_std' not in out and 'log_std' in out:
        out['loco_log_std'] = out['log_std'].clone()
    return out
