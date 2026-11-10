import json
import struct
import sys
import tempfile
import unittest
from pathlib import Path
import numpy as np
import torch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from levantar_amp import Policy, expand_inputs, export
from skill_policy import split_state


class SkillPolicyTests(unittest.TestCase):
    def setUp(self):
        torch.set_num_threads(1)
        torch.manual_seed(7)
        self.ranges = np.array([[-1., 1.], [-1., 1.]], np.float32)

    def test_migration_preserves_recovery(self):
        old = Policy(3, 2, self.ranges)
        new = Policy(4, 2, self.ranges, split=True)
        new.load_state_dict(split_state(expand_inputs(old.state_dict(), 4)))
        x = torch.randn(6, 3)
        y = torch.cat([x, torch.zeros(6, 1)], -1)
        self.assertTrue(torch.allclose(old.actor(old.norm(x)), new.actor(new.norm(y)), atol=1e-6))
        mean, var = new.norm.recovery.mean.clone(), new.norm.recovery.var.clone()
        loco_mean = new.norm.locomotion.mean.clone()
        y[:, -1] = 1
        new.norm.update(y * 4)
        self.assertTrue(torch.equal(new.norm.recovery.mean, mean))
        self.assertTrue(torch.equal(new.norm.recovery.var, var))
        self.assertFalse(torch.equal(new.norm.locomotion.mean, loco_mean))
        self.assertTrue(torch.equal(new.norm(y)[:, -1], y[:, -1]))

    def test_locomotion_cannot_update_recovery_even_with_adam_momentum(self):
        model = Policy(4, 2, self.ranges, split=True)
        optimizer = torch.optim.Adam(model.actor_parameters(), lr=.01)
        x = torch.randn(8, 4)
        x[:, -1] = 0
        model.actor(model.norm(x)).square().mean().backward()
        optimizer.step()
        before = {k: v.clone() for k, v in model.actor.recovery.state_dict().items()}
        old_std = model.log_std.detach().clone()
        optimizer.zero_grad(set_to_none=True)
        x[:, -1] = 1
        model.distribution(model.norm(x)).log_prob(torch.ones(8, 2)).mean().backward()
        self.assertTrue(all(p.grad is None for p in model.actor.recovery.parameters()))
        optimizer.step()
        self.assertTrue(all(torch.equal(v, model.actor.recovery.state_dict()[k]) for k, v in before.items()))
        self.assertTrue(torch.equal(old_std, model.log_std))

    def test_export_routing_and_resume(self):
        model = Policy(4, 2, self.ranges, split=True)
        model.norm.update(torch.tensor([[1., 2., 3., 1.], [3., 1., 0., 1.], [2., 2., 2., 1.]]))
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'policy.mrlpolicy'
            export(model, path, {})
            raw = path.read_bytes()
            size = struct.unpack('<I', raw[8:12])[0]
            header = json.loads(raw[12:12 + size])
            floats = np.frombuffer(raw[(12 + size + 3) & ~3:], dtype='<f4')
            self.assertEqual(header['skillModeIndex'], 3)
            for mode in [0., 1.]:
                x = torch.tensor([[.2, -.3, .1, mode]])
                expected = model.env_action(model.actor(model.norm(x))).detach().numpy()[0]
                y = x.numpy()[0]
                norm = header['locomotionNormalizer'] if mode else header['normalizer']
                y = np.clip((y - floats[norm['mean']:norm['mean'] + 4]) * floats[norm['invStd']:norm['invStd'] + 4], -5, 5)
                layers = header['locomotionLayers'] if mode else header['layers']
                for i, layer in enumerate(layers):
                    w = floats[layer['weights']:layer['weights'] + layer['inputs'] * layer['outputs']].reshape(layer['outputs'], layer['inputs'])
                    b = floats[layer['bias']:layer['bias'] + layer['outputs']]
                    y = w @ y + b
                    if i < len(layers) - 1: y = np.maximum(y, 0)
                np.testing.assert_allclose(expected, y, atol=1e-6)
            model.residual = .5
            export(model, path, {})
            raw = path.read_bytes()
            size = struct.unpack('<I', raw[8:12])[0]
            header = json.loads(raw[12:12 + size])
            floats = np.frombuffer(raw[(12 + size + 3) & ~3:], dtype='<f4')
            self.assertEqual(header['locomotionResidual'], .5)
            x = torch.tensor([[.2, -.3, .1, 1.]])
            expected = model.env_action(model.actor(model.norm(x)), x).detach().numpy()[0]
            norm = header['locomotionNormalizer']
            y = np.clip((x.numpy()[0] - floats[norm['mean']:norm['mean'] + 4]) * floats[norm['invStd']:norm['invStd'] + 4], -5, 5)
            for i, layer in enumerate(header['locomotionLayers']):
                w = floats[layer['weights']:layer['weights'] + layer['inputs'] * layer['outputs']].reshape(layer['outputs'], layer['inputs'])
                y = w @ y + floats[layer['bias']:layer['bias'] + layer['outputs']]
                if i < len(header['locomotionLayers']) - 1: y = np.maximum(y, 0)
            np.testing.assert_allclose(expected, y, atol=1e-6)
            resumed = Policy(4, 2, self.ranges, split=True)
            resumed.load_state_dict(split_state(expand_inputs(model.state_dict(), 4)))
            self.assertTrue(torch.equal(resumed.actor(x), model.actor(x)))


if __name__ == '__main__':
    unittest.main()
