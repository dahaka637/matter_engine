"""Ferramentas de inspecao dos treinos de RL (05/11, ele: "crie para voce ferramentas melhores... para conferencia da
movimentacao, de como esta o treino, os dados dele... um fantasma do movimento... ferramentas que apontam os problemas
automaticamente").

Modulos:
  comum     - caminhos, o treino (run.json, checkpoint), o lote de cada exercicio, a politica, a guarda termica e o servico;
  captura   - grava o movimento (corpo, referencia do clipe, contatos, medidas, acoes) em .npz e a gravacao do Inspector;
  fantasma  - o fantasma do movimento (as poses sobrepostas no tempo, em varias vistas, com os rastros e o clipe);
  marcha    - a analise da marcha (passos, angulos, inclinacao, simetria, deslizes) contra o clipe e o espelho;
  auditor   - aponta os problemas automaticamente, com a gravidade e a evidencia;
  painel    - os dados do treino (log.jsonl): a descida da forca, o que segura cada exercicio, os pares espelhados, a saude;
  inspecao  - o orquestrador: para o treino (regra termica), captura, desenha, analisa, audita, religa e escreve o relatorio.

Uso: tools/rl/.venv/bin/python -m inspecao.<modulo> --help (de dentro de tools/rl).
"""
