"""Varias geracoes no Kimodo com o modelo carregado uma vez e o texto em cache (o codificador e a Llama 3 8B
na CPU, o gargalo): para cada variacao do plano, um caminho de raiz denso opcional em linha reta (velocidade
constante, rumo travado para a frente) e CFG separado (texto x restricao). Grava npz + BVH (pose T padrao,
para o importador --source-rig soma) por amostra.

Roda dentro do container demo (o repo do Kimodo e o /workspace):
  docker exec -i demo bash -lc "cd /workspace && python - saidas/<pasta>/plano.json" < lote.py
Plano:
  {"modelo": "Kimodo-SOMA-RP-v1.1", "passos": 100, "saida": "saidas/<pasta>",
   "variacoes": [{"nome": ..., "texto": "A person ...", "segundos": 4.0,
                  "velocidade": 8.5 (m/s; null = sem caminho), "cfg": [texto, restricao],
                  "semente": 1, "amostras": 4, "chaves": "arquivo.json" (opcional, restricoes extras),
                  "pontos": [[quadro, x, z], ...] (opcional: pontos esparsos da raiz, rumo para a frente)}]}
"""
import json
import os
import sys

import torch

from kimodo import load_model
from kimodo.constraints import load_constraints_lst
from kimodo.exports.bvh import save_motion_bvh
from kimodo.exports.motion_io import save_kimodo_npz
from kimodo.skeleton import SOMASkeleton30, global_rots_to_local_rots
from kimodo.tools import save_json, seed_everything


class CachedText(torch.nn.Module):
    """O mesmo texto e codificado uma vez so (a API leva minutos por frase)."""

    def __init__(self, inner):
        super().__init__()
        self._inner = [inner]
        self.cache = {}

    def forward(self, texts):
        if isinstance(texts, str):
            texts = [texts]
        missing = [t for t in dict.fromkeys(texts) if t not in self.cache]
        if missing:
            feat, lengths = self._inner[0](missing)
            for i, t in enumerate(missing):
                self.cache[t] = feat[i, :lengths[i]].clone()
        lengths = [self.cache[t].shape[0] for t in texts]
        first = self.cache[texts[0]]
        out = torch.zeros(len(texts), max(lengths), first.shape[-1], dtype=first.dtype, device=first.device)
        for i, t in enumerate(texts):
            out[i, :lengths[i]] = self.cache[t]
        return out, lengths


def straight_path(frames, fps, speed):
    """Raiz em linha reta para +Z (rumo 0 do Kimodo), velocidade constante desde o quadro 0."""
    return {"type": "root2d", "frame_indices": list(range(frames)),
            "smooth_root_2d": [[0.0, speed * k / fps] for k in range(frames)],
            "global_root_heading": [[1.0, 0.0]] * frames}


def main():
    plan = json.load(open(sys.argv[1]))
    device = "cuda:0" if torch.cuda.is_available() else "cpu"
    model, resolved = load_model(plan.get("modelo", "Kimodo-SOMA-RP-v1.1"), device=device,
                                 default_family="Kimodo", return_resolved_name=True)
    model.text_encoder = CachedText(model.text_encoder)
    skeleton = model.skeleton
    skeleton77 = skeleton.somaskel77.to(device) if isinstance(skeleton, SOMASkeleton30) else skeleton
    print(f"modelo {resolved}, {model.fps} qps", flush=True)
    for v in plan["variacoes"]:
        frames = int(round(v.get("segundos", 4.0) * model.fps))
        constraints = []
        if v.get("velocidade"):
            constraints.append(straight_path(frames, model.fps, float(v["velocidade"])))
        if v.get("pontos"):
            points = v["pontos"]
            constraints.append({"type": "root2d", "frame_indices": [int(p[0]) for p in points],
                                "smooth_root_2d": [[float(p[1]), float(p[2])] for p in points],
                                "global_root_heading": [[1.0, 0.0]] * len(points)})
        if v.get("chaves"):
            constraints += json.load(open(v["chaves"]))
        constraint_lst = load_constraints_lst(constraints, skeleton) if constraints else []
        cfg = v.get("cfg", [2.0, 2.0])
        cfg_kwargs = ({"cfg_type": "separated", "cfg_weight": [float(cfg[0]), float(cfg[1])]} if constraints
                      else {"cfg_type": "regular", "cfg_weight": float(cfg[0])})
        seed_everything(int(v.get("semente", 0)))
        out = model([v["texto"]], [frames], constraint_lst=constraint_lst,
                    num_denoising_steps=int(plan.get("passos", 100)), num_samples=int(v.get("amostras", 4)),
                    multi_prompt=True, num_transition_frames=5, post_processing=v.get("pos", True),
                    return_numpy=True, **cfg_kwargs)
        folder = os.path.join(plan["saida"], v["nome"])
        os.makedirs(folder, exist_ok=True)
        n = int(out["posed_joints"].shape[0])
        for i in range(n):
            single = {k: (x[i] if hasattr(x, "shape") and len(x.shape) > 0 and x.shape[0] == n else x)
                      for k, x in out.items()}
            stem = os.path.join(folder, f"{v['nome']}_{i:02d}")
            save_kimodo_npz(stem + ".npz", single)
            joints = torch.from_numpy(out["posed_joints"][i]).to(device)
            rotations = torch.from_numpy(out["global_rot_mats"][i]).to(device)
            save_motion_bvh(stem + ".bvh", global_rots_to_local_rots(rotations, skeleton77),
                            joints[:, skeleton77.root_idx, :], skeleton=skeleton77, fps=model.fps,
                            standard_tpose=True)
        if constraints:
            save_json(os.path.join(folder, "constraints.json"), constraints)
        save_json(os.path.join(folder, "meta.json"), {
            "text": v["texto"], "duration": frames / model.fps, "num_samples": n,
            "seed": int(v.get("semente", 0)), "diffusion_steps": int(plan.get("passos", 100)),
            "cfg": {"enabled": True, "text_weight": float(cfg[0]), "constraint_weight": float(cfg[1])}})
        print(f"{v['nome']}: {n} amostras", flush=True)


if __name__ == "__main__":
    main()
