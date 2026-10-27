"""Contratos do treino motor-v4; não dependem de uma política já aprendida."""
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
import pytest
import torch

path = Path(__file__).resolve().parents[1] / 'motor_treino.py'
spec = importlib.util.spec_from_file_location('motor_treino', path)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
TASK = m.ROOT / 'assets/rl/motor/basico.json'


def batch(**kw):
    kw.setdefault('worlds', 2)
    kw.setdefault('threads', 1)
    return m.native.MotorBatch(str(m.ROOT / 'assets'), str(kw.pop('task', TASK)), **kw)


def recipe():
    return json.loads(TASK.read_text())


def test_gae_terminal_and_timeout_bootstrap():
    values = torch.tensor([[2.], [3.]])
    done = torch.tensor([[0.], [1.]])
    # Timeout recebe o valor terminal uma vez na recompensa, nunca no episódio seguinte.
    rewards = torch.tensor([[1.], [1. + .99 * 7.]])
    adv = m.advantages(rewards, values, done, torch.tensor([999.]))
    assert adv[1].item() == pytest.approx(1 + .99 * 7 - 3)
    assert adv[0].item() == pytest.approx(1 + .99 * 3 - 2 + .99 * .95 * adv[1].item())
    assert m.advantages(torch.zeros(1, 1), torch.ones(1, 1), torch.ones(1, 1), torch.tensor([999.])).item() == -1


def test_native_contract_and_probability_consistency():
    b = batch()
    assert b.action_size == 39 and b.obs_size == 156
    model = m.make_model(b, 'cpu', 1.)
    assert not model.log_std.requires_grad  # o ruído é fixo, não aprendido
    obs = torch.as_tensor(b.observations())
    dist = model.distribution(model.obs_norm(obs)); a = dist.sample()
    old = dist.log_prob(a).sum(-1)
    new = model.distribution(model.obs_norm(obs)).log_prob(a).sum(-1)
    torch.testing.assert_close((new - old).exp(), torch.ones(2))
    with pytest.raises(ValueError):
        b.step(np.full((2, 39), np.nan, np.float32))
    with pytest.raises(ValueError):
        b.step(np.zeros((1, 39), np.float32))
    rows = b.episodes()
    assert rows.shape == (0, 2 + len(m.native.episode_metric_names()))


def test_fixed_noise_is_per_group_in_radians():
    b = batch()
    derivative = np.asarray(b.action_derivatives())
    std = m.fixed_log_std(b, 1.).exp().numpy() * derivative
    for name, value in zip(b.action_names(), std):
        assert value == pytest.approx(m.NOISE_RADIANS[m.noise_group(name)], rel=1e-5)
    np.testing.assert_allclose(m.fixed_log_std(b, 2.).exp().numpy() * derivative, 2 * std, rtol=1e-5)
    assert m.noise_group('LeftShin.twist') == 'pernas' and m.noise_group('RightHand.swing1') == 'bracos'
    assert m.noise_group('UpperChest.twist') == 'tronco'


def test_correlated_noise_keeps_unit_variance():
    torch.manual_seed(3)
    eps = torch.randn(20000, 4)
    for _ in range(30):
        eps = m.next_noise(eps, .9)
    assert eps.std().item() == pytest.approx(1, abs=.03)
    fresh = m.next_noise(eps, 0.)
    assert abs(float((fresh * eps).mean())) < .03  # sem correlação, branco


def test_prior_has_a_free_band_and_pulls_toward_the_clip():
    mapping = (torch.ones(3), torch.zeros(3))
    reference = torch.zeros(1, 3)
    inside = torch.tensor([[.04, -.04, 0.]])
    assert m.prior_loss(inside, reference, mapping, .05).item() == 0
    outside = torch.tensor([[.25, 0., 0.]], requires_grad=True)
    loss = m.prior_loss(outside, reference, mapping, .05)
    assert loss.item() == pytest.approx(((math.tanh(.25) - .05) / m.PRIOR_RADIANS) ** 2 / 3, rel=1e-5)
    loss.backward()
    assert outside.grad[0, 0] > 0  # o gradiente descendo leva a média para a referência


def test_prior_schedule_and_validation():
    r = recipe()
    r['teacher'] = {'priorWeight': .2, 'freeRadians': .05, 'fullUntilDecisions': 100, 'zeroAtDecisions': 300}
    assert m.prior_weight(r, 50) == .2
    assert m.prior_weight(r, 200) == pytest.approx(.1)
    assert m.prior_weight(r, 300) == 0
    r['teacher']['zeroAtDecisions'] = 10
    with pytest.raises(ValueError):
        m.prior_weight(r, 0)


def rows_for(drill, n, fell=0., tracking=.9, posture=.9, target=1., chest=0.):
    names = m.native.episode_metric_names()
    rows = np.zeros((n, 2 + len(names)), np.float32)
    rows[:, 1] = drill
    rows[:, m.COL['caiu']] = fell
    rows[:, m.COL['rastreio']] = tracking
    rows[:, m.COL['postura']] = posture
    rows[:, m.COL['velocidade_alvo']] = target
    rows[:, m.COL['erro_peito']] = chest
    rows[:, m.COL['erro_rumo']] = chest
    return rows


def test_permanent_prior_weights_only_its_group():
    b = batch()
    r = recipe()
    assert not m.permanent_prior(r, b.action_names()).any()  # sem a chave, nada muda
    r['teacher']['permanent'] = {'bracos': .1}
    w = m.permanent_prior(r, b.action_names())
    arms = np.asarray([m.noise_group(n) == 'bracos' for n in b.action_names()])
    assert arms.any() and (w[arms] == np.float32(.1)).all() and (w[~arms] == 0).all()
    r['teacher']['permanent'] = {'cabeca': .1}
    with pytest.raises(ValueError):
        m.permanent_prior(r, b.action_names())


def test_trot_promotion_can_require_progress_and_chest_heading():
    r = recipe()
    r['trot']['curriculum'].update({'minimumProgress': .6, 'maximumChestErrorDegrees': 30})
    c = m.Curriculum(r)
    start = c.limits[0]
    sideways = rows_for(2, 64, chest=math.radians(110))
    sideways[:, m.COL['progresso']] = .9
    assert c.observe(sideways) == ([], False)  # rastreio bom com o peito virado: não sobe
    slow = rows_for(2, 64)
    slow[:, m.COL['progresso']] = .2
    assert c.observe(slow) == ([], False)  # projeção boa com progresso vetorial ruim: não sobe
    good = rows_for(2, 64)
    good[:, m.COL['progresso']] = .9
    promoted, _ = c.observe(good)
    assert promoted == [0] and c.limits[0] > start


def test_curriculum_ignores_episodes_started_in_motion():
    c = m.Curriculum(recipe())
    start = c.limits[0]
    moving = rows_for(2, 64)
    moving[:, m.COL['comeco_em_movimento']] = 1
    assert c.observe(moving) == ([], False) and c.limits[0] == start


def test_trot_starts_in_motion_only_in_training(tmp_path):
    r = recipe()
    r['shares'] = {'parado': 0., 'giro': 0., 'trote': 1.}
    r['trot']['startInMotion'] = 1.
    task = tmp_path / 'tarefa.json'
    task.write_text(json.dumps(r))

    def speeds(**kw):
        b = batch(task=task, worlds=8, threads=2, seed=3, **kw)
        names, obs = b.observation_names(), b.observations()
        col = lambda n: obs[:, names.index(n)]  # noqa: E731
        return np.hypot(col('velocity.x'), col('velocity.y')), 2 * np.hypot(col('command.velocity.x'), col('command.velocity.y'))

    body, command = speeds()
    assert (command > .25).all() and (np.abs(body - command) < .35).all()  # nasce na passada, já no pedido
    body, command = speeds(drill=2, evaluation=True)
    assert (body < .05).all() and (command < .1).all()  # a avaliação nasce parada no IDLE


def test_trot_full_state_start_uses_the_clip_root_and_only_in_training(tmp_path):
    def task(name, shares, motion, full):
        r = recipe()
        r['shares'] = shares
        r['trot']['startInMotion'] = r['turn']['startInMotion'] = motion
        if full is not None:
            r['trot']['fullStateStart'] = full
        t = tmp_path / name
        t.write_text(json.dumps(r))
        return t

    trot = {'parado': 0., 'giro': 0., 'trote': 1.}
    absent, off, on = (batch(task=task(f'{n}.json', trot, 1., f), worlds=8, threads=2, seed=3)
                       for n, f in (('ausente', None), ('desligado', False), ('ligado', True)))
    names = on.observation_names()
    col = lambda o, n: o[:, names.index(n)]  # noqa: E731
    spin = lambda o: np.abs(o[:, [names.index(f'angular.{a}') for a in 'xyz']]).max()  # noqa: E731
    o_off, o_on = off.observations(), on.observations()
    assert spin(o_off) < 1e-3 and spin(o_on) > .05  # a raiz nasce girando como no clipe
    assert np.abs(o_on[:, :3] - o_off[:, :3]).max(axis=1).min() > 1e-4  # a raiz inclinada do clipe
    assert not np.array_equal(col(o_on, 'velocity.z'), col(o_off, 'velocity.z'))
    zero = np.zeros((8, on.action_size), np.float32)
    for _ in range(20):  # desligado = o comeco antigo, bit a bit
        assert np.array_equal(absent.observations(), off.observations())
        absent.step(zero); off.step(zero)
    rest = batch(task=task('aval.json', trot, 1., True), drill=2, evaluation=True).observations()
    assert (np.hypot(col(rest, 'velocity.x'), col(rest, 'velocity.y')) < .05).all() and spin(rest) < 1e-3
    turn = {'parado': 0., 'giro': 1., 'trote': 0.}
    a, b = (batch(task=task(f'giro{f}.json', turn, 1., f), worlds=4, seed=3) for f in (False, True))
    for _ in range(5):  # o giro no movimento nao muda
        assert np.array_equal(a.observations(), b.observations())
        a.step(zero[:4]); b.step(zero[:4])


def test_clip_drive_with_zero_action_is_the_clip_in_the_motors(tmp_path):
    r = recipe()
    r['clipDrive'] = 1.
    task = tmp_path / 'tarefa.json'
    task.write_text(json.dumps(r))
    old, new = batch(seed=5, drill=2, evaluation=True), batch(task=task, seed=5, drill=2, evaluation=True)
    names = new.observation_names()
    assert names[:-1] == old.observation_names() and names[-1] == 'clip.drive' and new.clip_drive == 1.
    for _ in range(45):
        old.step(np.asarray(old.reference_actions(), np.float32))
        new.step(np.zeros((2, new.action_size), np.float32))
        assert np.array_equal(old.observations(), new.observations()[:, :-1])  # bit a bit o mesmo corpo
    assert (new.observations()[:, -1] == 1.).all()


def test_clip_drive_key_cannot_appear_or_vanish_live_and_refuses_the_teacher(tmp_path):
    r = recipe()
    r['clipDrive'] = 1.
    task = tmp_path / 'tarefa.json'
    task.write_text(json.dumps(r))
    b = batch(task=task)
    r['clipDrive'] = 0.
    task.write_text(json.dumps(r))
    b.set_task(str(task))  # desligar pelo peso vale
    assert b.clip_drive == 0.
    del r['clipDrive']
    task.write_text(json.dumps(r))
    with pytest.raises(Exception):
        b.set_task(str(task))
    r['clipDrive'] = .5
    r['teacher']['priorWeight'] = .1
    with pytest.raises(ValueError):
        m.teacher_settings(r)
    r['teacher']['priorWeight'] = 0.
    m.teacher_settings(r)


def test_ungated_imitation_pays_for_holding_the_clip_before_the_speed(tmp_path):
    def income(ungated):
        r = recipe()
        r['clipDrive'] = 1.
        r['teacher']['priorWeight'] = 0.
        r['imitation']['ungated'] = ungated
        task = tmp_path / f'tarefa_{ungated}.json'
        task.write_text(json.dumps(r))
        b = batch(task=task, worlds=4, threads=2, seed=9, drill=2, evaluation=True, seconds=4, trot_speed=2.)
        names, rows = m.native.episode_metric_names(), []
        for _ in range(120):
            b.step(np.zeros((4, b.action_size), np.float32))
            rows += list(b.episodes())
        rows = np.asarray(rows)
        return rows[:, 2 + names.index('renda_imitacao')].mean(), rows[:, 2 + names.index('renda_vida')].mean()
    (gated, alive_gated), (ungated, alive_ungated) = income(False), income(True)
    assert ungated > 2 * gated + 1e-3  # o clipe em pe paga antes de acertar a velocidade
    assert alive_ungated == pytest.approx(alive_gated)  # a renda de vida continua presa ao progresso


def test_curriculum_raises_trot_limit_only_with_good_episodes_near_the_limit():
    c = m.Curriculum(recipe())
    start = c.limits[0]
    assert c.observe(rows_for(2, 64, target=.3 * start)) == ([], False)  # longe do teto: não conta
    assert c.observe(rows_for(2, 64, fell=1.)) == ([], False)            # caindo: não sobe
    promoted, _ = c.observe(rows_for(2, 64))
    assert promoted == [0] and c.limits[0] == pytest.approx(start + float(recipe()['trot']['curriculum']['step']))
    assert c.limits[1:] == [start] * 7
    restored = m.Curriculum(recipe(), c.state())
    assert restored.limits == c.limits and restored.stage == c.stage


def test_curriculum_moves_the_mix_to_the_final_shares():
    r = recipe()
    c = m.Curriculum(r)
    base = [r['stages']['base']['shares'][k] for k in ('parado', 'giro', 'trote')]
    assert c.shares() == base
    assert c.observe(np.concatenate([rows_for(0, 64, fell=1.), rows_for(1, 64)])) == ([], False)
    assert c.observe(np.concatenate([rows_for(0, 64), rows_for(1, 64, chest=1.)])) == ([], False)  # giro que não gira
    _, staged = c.observe(np.concatenate([rows_for(0, 64), rows_for(1, 64)]))
    assert staged and c.stage == 1 and c.shares() == [r['shares'][k] for k in ('parado', 'giro', 'trote')]


def test_motor_export_matches_native_runtime(tmp_path):
    import matter_rl_native
    b = batch(worlds=8)
    model = m.make_model(b, 'cpu', 1.)
    with torch.no_grad():
        model.actor[-1].bias.fill_(.3)
    policy = tmp_path / 'motor.mrlpolicy'
    m.export_policy(model, policy, {'observationLayout': m.CONTRACT, 'bodyHash': b.identity()},
                    action_limit=float(np.finfo(np.float32).max))
    runtime = matter_rl_native.Policy(str(policy))
    obs = b.observations()
    with torch.no_grad():
        expected = model.actor(model.obs_norm(torch.as_tensor(obs))).numpy()
    np.testing.assert_allclose(runtime.evaluate(obs), expected, atol=2e-6, rtol=2e-5)


def test_invalid_recipe_preserves_live_environment(tmp_path):
    b = batch(worlds=1)
    before = b.observations().copy()
    r = recipe()
    r['weights']['alive'] = -1
    bad = tmp_path / 'bad.json'; m.atomic_json(bad, r)
    with pytest.raises(RuntimeError):
        b.set_task(str(bad))
    np.testing.assert_array_equal(b.observations(), before)
    b.step(np.zeros((1, b.action_size), np.float32))


def test_zero_action_is_alert_idle_and_standing_never_pays_under_a_move_command():
    b = batch(worlds=4, drill=0, evaluation=True)
    np.testing.assert_allclose(b.pose_actions(b.idle_coordinates()), 0, atol=1e-3)
    zero = np.zeros((4, b.action_size), np.float32)
    standing = np.mean([b.step(zero)[0].mean() for _ in range(15)])
    trot = batch(worlds=4, drill=2, evaluation=True, trot_speed=1.)
    still = np.mean([trot.step(zero)[0].mean() for _ in range(30)])
    assert standing > 0 > still


def test_reference_follows_the_clip_in_the_trot_and_stays_near_idle_standing():
    stand = batch(worlds=2, drill=0, evaluation=True)
    assert np.abs(stand.reference_actions()).max() < .5
    trot = batch(worlds=2, drill=2, evaluation=True, trot_speed=3.)
    zero = np.zeros((2, trot.action_size), np.float32)
    seen = []
    for _ in range(40):
        seen.append(trot.reference_actions()[0].copy())
        trot.step(zero)
    seen = np.asarray(seen)
    assert seen.std(0).max() > .1  # a passada do clipe varia no ciclo


def test_trainer_controls_recipe_resume_and_contract(tmp_path):
    import os
    import subprocess
    import tempfile
    import time
    from matter_rl.control import request

    with tempfile.TemporaryDirectory(prefix='motor-control-') as runtime:
        sock = Path(runtime) / 'control.sock'
        env = dict(os.environ, MATTER_RL_SOCKET=str(sock))
        command = [str(Path(m.sys.executable)), str(path), '--dados', str(tmp_path), '--treino', 'check',
                   '--mundos', '4', '--threads', '1', '--passos', '2', '--avaliar-cada', '0',
                   '--avaliar-mundos', '2', '--avaliar-segundos', '.5']
        log = open(tmp_path / 'process.log', 'w')
        process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)

        def wait_for(predicate):
            deadline = time.monotonic() + 90
            while time.monotonic() < deadline:
                if process.poll() is not None:
                    pytest.fail((tmp_path / 'process.log').read_text())
                result = predicate()
                if result:
                    return result
                time.sleep(.05)
            pytest.fail('treinador não respondeu: ' + (tmp_path / 'process.log').read_text())

        def call(cmd, args=None):
            reply = request(cmd, args, path=sock)
            assert reply and reply['ok'], reply
            return reply

        run = tmp_path / 'runs/check'
        try:
            wait_for(lambda: (s := request('status', path=sock)) and s.get('status', {}).get('state') == 'treinando')
            call('pausar')
            first = call('status')['status']['iteration']
            time.sleep(.15)
            assert call('status')['status']['iteration'] == first
            call('modo', {'modo': 'leve'})
            assert call('status')['status']['threads'] == 4
            call('salvar')
            state = torch.load(run / 'ultimo.pt', weights_only=False)
            assert state['manifest']['contract'] == 'motor-v4' and state['warm_start'] and len(state['warm_start']) == 4
            r = json.loads((run / 'tarefa.json').read_text())
            r['imitation']['weight'] = .5
            m.atomic_json(run / 'tarefa.json', r)
            call('retomar')
            wait_for(lambda: (run / 'recipes.jsonl').exists())
            call('pausar')
            assert list(run.glob('antes_receita_*.pt'))
            assert call('status')['status']['imitation_weight'] == .5
            call('avaliar')
            assert (run / 'avaliacoes.jsonl').is_file()
            evaluation = json.loads((run / 'avaliacoes.jsonl').read_text().splitlines()[-1])['metrics']
            assert set(evaluation) == set(m.native.drill_ids()) and 'postura' in evaluation['parado']
            call('parar'); process.wait(timeout=30)
            assert process.returncode == 0
            saved = torch.load(run / 'ultimo.pt', weights_only=False)
            done = subprocess.run(command + ['--preparar'], env=env, stdout=log, stderr=subprocess.STDOUT, timeout=60)
            assert done.returncode == 0, (tmp_path / 'process.log').read_text()
            resumed = torch.load(run / 'ultimo.pt', weights_only=False)
            for key in saved['model']:
                torch.testing.assert_close(resumed['model'][key], saved['model'][key], rtol=0, atol=0)
            assert resumed['counters'] == saved['counters'] and resumed['warm_start'] == saved['warm_start']
            import struct
            with open(run / 'exports/policy_latest.mrlpolicy', 'rb') as policy:
                assert policy.read(8) == b'MRLPOL01'
                header = json.loads(policy.read(struct.unpack('<I', policy.read(4))[0]))
            assert header['observationLayout'] == 'motor-v4'
            np.testing.assert_allclose(header['explorationStd'], resumed['model']['log_std'].exp().cpu().numpy(), rtol=1e-6)
            assert json.loads((run / 'curriculo.json').read_text())['limits'] == resumed['curriculum']['limits']
            # Um checkpoint de outro contrato é recusado (o motor-v4 começa do zero).
            resumed['manifest'] = dict(resumed['manifest'], contract='motor-v3')
            torch.save(resumed, run / 'ultimo.pt')
            refused = subprocess.run(command + ['--preparar'], env=env, capture_output=True, text=True, timeout=60)
            assert refused.returncode != 0 and 'outro contrato' in refused.stderr
        finally:
            if process.poll() is None:
                process.terminate(); process.wait(timeout=30)
            log.close()
