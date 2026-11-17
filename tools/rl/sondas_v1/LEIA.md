# sondas_v1 — as réguas da fase final da v1 do RL (13–14/11)

Rodar de dentro de `tools/rl`, com `MATTER_TRACKING_NATIVE=<build>/python` e o python do `.venv`. O diário de cada medida está no vault, em `04_locomocao/plano_v1_final_13_11.md`.

| script | o que mede | uso |
|---|---|---|
| `sonda_quedas_jogo.py` | quedas do jogo sem fogo (comando fases 1–4, estrela fixo/livre/girando), separando a queda real (pelve < 0,55 m) do fim pelas regras (virado, fora do rumo); `SO=caso1,caso2`, `SEMENTE=`, `SAIDA=` (as trilhas) | `CK RECEITA [N] [SEGUNDOS]` |
| `custo_ingame.py` | o custo por jogador e decisão (passo nativo, observação, rede) com 23 mundos numa thread; `MATTER_RL_SOLVER="pos,vel"` troca as iterações do solver (build n99+) | `CK RECEITA` |
| `braco_parada.py` | o braço estabanado na freada: o maior desvio da mão contra o clipe entre a ordem de parar e +2 s, nas capturas da suíte | `PASTA_CAPTURAS [...]` |
| `ver_alvos_imitar.py` | os alvos da imitação do levantar (`--levantar-imitar`): diferentes de zero só nos exercícios da máscara | `CK` |
| `imitar_confere.py` | a nota de imitação (`--levantar-imitar-nota`) na mesma trajetória com e sem | `CK RECEITA` |
| `acompanha_g.py` | os braços do levantar de costas contra o principal (semelhança, levantou, tempo, imitação) | — |
| `acompanha_s.py` | o especialista do de costas contra o principal (semelhança, levantou, `longe` = fim cedo, tempo, KL) | — |
| `lancar_especialista.sh` | lança o especialista do de costas do checkpoint atual do principal (85% levantar de costas limpo, nota de semelhança, fim cedo; build n106) | `NOTA LIMIAR SEGUNDOS [THREADS]` |
| `folha_costas.sh` | o levantar de costas de um checkpoint em quadros de lado (para pôr ao lado das poses do clipe do `MatterModelInspector poses`) | `TREINO CHECKPOINT NOME` |
