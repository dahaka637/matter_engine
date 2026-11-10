"""A prova de equivalencia do treino (10/11; vault 04_locomocao/metodologia_rl.md, secao 8: uma limpeza so entra se o treino
faz a mesma conta antes e depois).

Com --prova ARQUIVO o levantar_amp.py roda na CPU, deterministico, e guarda o hash de cada coleta (todos os tensores da
rodada), dos episodios terminados (ordenados pelo mundo) e da rede depois de cada atualizacao. O tempo de parede fica de
fora. Comparar dois arquivos:

  python prova.py comparar A.json B.json      # sai 0 se iguais; senao diz a primeira diferenca
"""
from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import torch


def _bytes(value) -> bytes:
    if torch.is_tensor(value):
        return value.detach().to("cpu").contiguous().numpy().tobytes()
    return np.ascontiguousarray(np.asarray(value)).tobytes()


def digest(items: dict) -> str:
    """O hash de um dicionario de tensores ou arrays, na ordem das chaves."""
    h = hashlib.sha256()
    for key in sorted(items):
        h.update(key.encode())
        h.update(_bytes(items[key]))
    return h.hexdigest()


def state_digest(*modules) -> str:
    h = hashlib.sha256()
    for module in modules:
        if module is None:
            continue
        for key, value in module.state_dict().items():
            h.update(key.encode())
            h.update(_bytes(value))
    return h.hexdigest()


class Prova:
    def __init__(self, path: str, argv: list[str]):
        self.path = Path(path)
        self.argv = list(argv)
        self.records: list[dict] = []

    @staticmethod
    def preparar() -> torch.device:
        """A CPU deterministica: um fio, os algoritmos deterministicos do torch."""
        torch.use_deterministic_algorithms(True)
        torch.set_num_threads(1)
        return torch.device("cpu")

    def coleta(self, update: int, roll: dict) -> None:
        self.records.append({"atualizacao": int(update), "coleta": digest({k: v for k, v in roll.items() if torch.is_tensor(v)})})

    def episodios(self, update: int, episodes) -> None:
        rows = np.asarray(episodes)
        if rows.size:
            rows = rows[np.lexsort(rows.T[::-1])]
        self.records.append({"atualizacao": int(update), "episodios": digest({"linhas": rows}), "n": int(len(rows))})

    def rede(self, update: int, *modules) -> None:
        self.records.append({"atualizacao": int(update), "rede": state_digest(*modules)})

    def gravar(self) -> None:
        self.path.write_text(json.dumps({"argv": self.argv, "registros": self.records}, indent=1))
        print(f"prova: {len(self.records)} registros em {self.path}", flush=True)


def comparar(a: str, b: str) -> int:
    ra = json.loads(Path(a).read_text())["registros"]
    rb = json.loads(Path(b).read_text())["registros"]
    for i, (x, y) in enumerate(zip(ra, rb)):
        if x != y:
            print(f"DIFERENTE no registro {i}: {x} != {y}")
            return 1
    if len(ra) != len(rb):
        print(f"DIFERENTE: {len(ra)} registros contra {len(rb)}")
        return 1
    print(f"IGUAL: {len(ra)} registros")
    return 0


if __name__ == "__main__":
    if len(sys.argv) == 4 and sys.argv[1] == "comparar":
        raise SystemExit(comparar(sys.argv[2], sys.argv[3]))
    print(__doc__)
    raise SystemExit(2)
