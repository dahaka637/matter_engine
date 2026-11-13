# Sondas e ferramentas da fase de fogo do levantar (12/11)

Copiadas da pasta temporária da sessão. Rodar de `tools/rl` com `MATTER_TRACKING_NATIVE=<build>/python` e o python do venv;
o caminho de `inspecao.comum` vem do `sys.path.insert(0, '.')`. Documentação: vault `04_locomocao/como_fizemos_o_levantar_sob_fogo_12_11.md`.

- `sonda_fogo3.py` — a calibração v4 (o nível em que o sucesso cruza o alvo, numa grade).
- `sonda_recupera.py`, `sonda_seguidos.py`, `sonda_modo_treino2.py` — a caça ao "recuperou 0% no treino" (avaliação × modo de treino, ruído, configurar).
- `sonda_codigos_media.py` — os códigos do "foi ao chão" e a janela de firmeza, sem ruído.
- `sonda_tentar12.py`, `sonda_reativo.py`, `sonda_lado.py` — a nova tentativa (episódio de 12 s, como o treino).
- `sonda_contra.py` — o sucesso por rumo da força com o limite contra a corrida.
- `sonda_n56.py`, `sonda_paredes.py` — o teto do contínuo pelo atrito e as paredes.
- `sonda_fase2b.py` — as adversidades da fase 2, por adversidade.
- `sonda_pose.py` — o começo em pose variada (lote em modo de treino).
- `sonda_troca.py` — a troca calma × por tempo (a firmeza depois de levantar).
- `sonda_juiz.py` — o juiz da recuperação do checkpoint no limpo × fogo.
- `resumo_fogo.py`, `analise_tendencias.py` — os resumos do log; `vigia_escada.sh` — o vigia (Monitor); `lancar.sh` — o lançador do serviço.
