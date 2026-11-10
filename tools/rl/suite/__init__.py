"""A suite fixa de regressao do RL (10/11; vault 04_locomocao/metodologia_rl.md, secoes 3-5): a mesma bateria em todo
candidato (checkpoint + receita), nas mesmas sementes, que decide se uma mudanca melhorou o alvo sem piorar o resto.

  cenarios   - os cenarios e as sementes versionados (SUITE_V1);
  captura    - roda o candidato nos cenarios e grava as poses do corpo e do clipe, contatos, pedido, telemetria (npz);
  medidas/   - um teste por queixa dele, em numpy puro sobre as capturas (nunca a nota do treino);
  estatistica- IC95 por bootstrap nos mundos (Wilson nas taxas), IQM, pior mundo, margens e pisos;
  relatorio  - resultados.json e tabela.md (os candidatos lado a lado, o delta contra o primeiro, a marca);
  figuras    - o parado visto de cima e a tira da freada.

Uso (de dentro de tools/rl): .venv/bin/python -m suite --help
"""
