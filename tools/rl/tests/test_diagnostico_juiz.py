"""D0: contas puras e uma execução nativa pequena, sempre em tmp_path."""
import json

import numpy as np
import pytest
import torch

from matter_rl import paths
from matter_rl.diagnostico_juiz import (axis_group, measure_group, weighted_normalization,
                                       split_groups, d0_criterion, run_diagnostico)


def test_grupos_por_nome():
    assert [axis_group(n) for n in ['LeftThigh.swing1.target', 'RightToe.twist.target',
                                    'LeftForearm.twist.target', 'Chest.swing1.target']] == ['legs', 'toes', 'arms', 'trunk']
    assert [measure_group(n) for n in ['pelvis.velocity.x', 'Chest.twist.position',
                                      'Chest.twist.velocity', 'LeftFoot.height']] == list(('pelve', 'angulos', 'velocidades_eixos', 'partes'))


def test_normalizacao_ponderada():
    x = np.array([[0., 3., 2., 3.], [4., 3., 6., 3.], [999., 999., 999., 999.]])
    mean, std = weighted_normalization(x, [1., 3., 0.])
    np.testing.assert_allclose(mean, [4., 3.])
    np.testing.assert_allclose(std, [2., 0.05])
    from matter_rl.amp import Judge, JudgeSettings
    ds = {'pairs': x.astype(np.float32), 'pair_weight': [1., 3., 0.], 'clips': [], 'hash': 'teste'}
    judge = Judge(ds, JudgeSettings(hidden=[8], replay=8), torch.device('cpu'))
    np.testing.assert_allclose(judge.mean.numpy(), np.tile(mean, 2))
    np.testing.assert_allclose(judge.inv_std.numpy(), 1 / np.tile(std, 2))


def test_d0_sintetico():
    assert d0_criterion(3., 2., .6, 1., .7)['pass']
    assert d0_criterion(3., 2., .6, 1., None)['pass']
    for values in [(3.01, 2., .6, 1., .7), (3., 2., .59, 1., .7),
                   (3., 2., .6, 1., .69), (0., 0., .6, 1., None), (3., 2., 0., 0., None)]:
        assert not d0_criterion(*values)['pass']


def test_validacao_nunca_divide_clipe_ou_mundo():
    ids = np.repeat(np.arange(10), 13)
    train, validation = split_groups(ids)
    assert train.any() and validation.any()
    assert not set(ids[train]) & set(ids[validation])
    assert np.all(train ^ validation)
    for group in np.unique(ids):
        assert train[ids == group].all() or validation[ids == group].all()
    t, v = split_groups(np.zeros(12))
    assert not t.any() and not v.any()


@pytest.fixture()
def scratch(tmp_path, monkeypatch):
    monkeypatch.setattr(paths, 'DATA', tmp_path)
    monkeypatch.setattr(paths, 'RUNS', tmp_path / 'runs')
    monkeypatch.setattr(paths, 'DATABASE', tmp_path / 'rl.db')
    return tmp_path


def test_integracao_pequena(scratch):
    # Mantém o juiz verdadeiro e o orçamento padrão; limita threads da CPU.
    previous = torch.get_num_threads()
    torch.set_num_threads(2)
    try:
        result = run_diagnostico('aquisicao_p3a', {'pequeno': .05}, scratch,
                                worlds=16, seconds=1., judge_rounds=2, with_controls=False)
    finally:
        torch.set_num_threads(previous)
    assert {'condicoes', 'reservados', 'd0', 'acao_clipes', 'controles_mistura'} <= result.keys()
    assert set(result['condicoes']) == {'sem_ruido', 'pequeno'}
    for condition in result['condicoes'].values():
        assert condition['coleta']['pares_distintos'] >= 4096
        assert len(condition['medidas_mais_longe']) == 10
        assert condition['juiz']['saude']
        assert len(condition['juiz']['caminho']) == 6
        for split in condition['classificador'].values():
            if split['accuracy'] is not None:
                assert not set(split['clipes_treino']) & set(split['clipes_validacao'])
                assert not set(split['mundos_treino']) & set(split['mundos_validacao'])
    assert {'distance_ratio', 'judge_ratio', 'reserved_score', 'pass'} == result['d0']['pequeno'].keys()
    saved = json.loads((scratch / 'diagnostico_juiz.json').read_text())
    assert saved == result
    assert 'Diagnóstico' in (scratch / 'diagnostico_juiz.md').read_text()
    task = json.loads((scratch / 'tarefa_diagnostico.json').read_text())
    assert task['initialState']['referenceFraction'] == task['initialState']['movingFraction'] == 0
    assert task['body'].startswith('/') and task['style']['dataset'].startswith('/')
