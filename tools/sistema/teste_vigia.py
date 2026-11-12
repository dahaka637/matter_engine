#!/usr/bin/env python3
"""O teste do vigia: quem ele corta (o executavel e o script, nunca o texto da linha)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from vigia_temperatura import kind  # noqa: E402

CASOS = {
    ("/r/.venv/bin/python", "-m", "matter_rl", "treinar", "--run", "b4"): "treino",
    ("/r/.venv/bin/python", "-m", "matter_rl", "servir", "--run", "b4"): "treino",
    ("/r/.venv/bin/python", "-m", "matter_rl", "status"): "",
    ("/usr/bin/bash", "-c", "python -m matter_rl treinar; avaliar.py"): "",
    ("MatterStandBench", "--segundos", "60"): "pesado",
    ("/r/build-profile/MatterRLTrainingTests",): "pesado",
    (".venv/bin/python", "/tmp/s/avaliar.py", "b4", "frente"): "pesado",
    (".venv/bin/python", "previa_locomocao.py", "--treino", "b4"): "pesado",
    (".venv/bin/python", "levantar_amp.py", "--treino", "levantar_amp"): "pesado",
    (".venv/bin/python", "/tmp/s/bench_b21.py", "saida.json"): "pesado",
    (".venv/bin/python", "/tmp/s/bench_rl_final.py", "saida.json"): "pesado",
    ("/usr/bin/bash", "-c", "python bench_b21.py saida.json"): "",
    (".venv/bin/python", "bench_comparar.py", "antes", "depois"): "",
    ("/usr/bin/bash", "/tmp/s/checagem_b1.sh", "b4", "4"): "pesado",
    ("/usr/bin/code",): "",
    ("ninja", "-C", "build"): "pesado",
    ("claude",): "",
}
for argv, want in CASOS.items():
    got = kind(list(argv))
    assert got == want, (argv, got, want)
print(f"vigia: {len(CASOS)} casos ok")
