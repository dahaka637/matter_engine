"""Poe amostras escolhidas na interface do Kimodo (Load Example): uma pasta por amostra em
kimodo/assets/demo/examples/kimodo-soma-rp/<nome>/ com motion.npz, meta.json (o texto e a geracao da variacao) e
constraints.json (se a variacao teve). Recarregar a pagina mostra os novos.

uso: exemplos.py nome=saidas/<rodada>/<variacao>/<amostra>.npz [...]
"""
import json
import os
import shutil
import sys

KIMODO = os.environ.get("KIMODO", "/home/dahaka/Projetos/kimodo")
EXAMPLES = os.path.join(KIMODO, "kimodo/assets/demo/examples/kimodo-soma-rp")

for spec in sys.argv[1:]:
    name, source = spec.split("=", 1)
    source = source if os.path.isabs(source) else os.path.join(KIMODO, source)
    folder = os.path.dirname(source)
    meta = json.load(open(os.path.join(folder, "meta.json")))
    meta["num_samples"] = 1
    target = os.path.join(EXAMPLES, name)
    if os.path.exists(target):
        shutil.rmtree(target)
    os.makedirs(target)
    shutil.copy2(source, os.path.join(target, "motion.npz"))
    if os.path.exists(os.path.join(folder, "constraints.json")):
        shutil.copy2(os.path.join(folder, "constraints.json"), os.path.join(target, "constraints.json"))
    json.dump(meta, open(os.path.join(target, "meta.json"), "w"), indent=2, ensure_ascii=False)
    print(f"{name}: {source}")
