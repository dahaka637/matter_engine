import copy
import unittest

import numpy as np
import torch

from levantar_amp import Policy, anchor_distance


class AncoraTests(unittest.TestCase):
    def test_zero_no_campeao_e_positiva_perturbado(self):
        torch.manual_seed(3)
        champion = Policy(6, 3, np.array([[-1., 1.]] * 3, np.float32), split=True)
        obs = torch.randn(64, 6)
        obs[:, -1] = 1.0  # as linhas do banco sao da cabeca da locomocao
        champion.norm.update(obs * 2 + 1)
        with torch.no_grad():
            target = champion.actor(champion.norm(obs))
        model = copy.deepcopy(champion)
        self.assertEqual(float(anchor_distance(model, obs, target)), 0.0)
        with torch.no_grad():
            model.actor.recovery[0].weight.add_(1.0)  # a recuperacao fica fora do banco
        self.assertEqual(float(anchor_distance(model, obs, target)), 0.0)
        with torch.no_grad():
            model.actor.locomotion[0].weight.add_(0.01 * torch.randn_like(model.actor.locomotion[0].weight))
        dist = anchor_distance(model, obs, target)
        self.assertGreater(float(dist), 0.0)
        dist.backward()
        self.assertIsNotNone(model.actor.locomotion[0].weight.grad)
        self.assertIsNone(model.actor.recovery[0].weight.grad)


if __name__ == "__main__":
    unittest.main()
