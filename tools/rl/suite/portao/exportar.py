"""Exporta um ponto do treino como o proprio treino exporta (levantar_amp.export, mesma rede pela inspecao): exportar.py TREINO PONTO SAIDA"""
import sys
from pathlib import Path
sys.path.insert(0, "/home/dahaka/Projetos/matter_engine_wip/tools/rl")
from inspecao import comum
treino, ck, out = sys.argv[1], sys.argv[2], Path(sys.argv[3])
amb = comum.Ambiente(treino, ck)
b = amb.L.get_up_batch(amb.S.ASSETS, environments=1, threads=1, seed=61, evaluation=True, evaluation_kind=0, **amb.S.TRACK)
model = amb.model(b)
st, c = amb.state, amb.state["counters"]
amb.L.export(model, out, {"run": treino, "iteration": c["atualizacao"], "steps": c["amostras"], "observationNames": st["settings"]["observacao"]})
print(out, c["atualizacao"])
