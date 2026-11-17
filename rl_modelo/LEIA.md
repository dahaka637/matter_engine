# A rede de RL atual (cérebro único r130240)

É a rede que estava no laboratório e no jogo em 17/11/2026, quando o projeto foi arquivado. Ela foi treinada até 15/11 às 15h e é a melhor que ele aprovou. Ainda está torta; a história e os problemas conhecidos estão no vault (repositório `matter_engine_ia_brain`, comece pelo `LEIA_PRIMEIRO.md`).

| Arquivo (em `dados/runs/`) | O que é |
|---|---|
| `cerebro_unico/exports/policy_latest.mrlpolicy` | a rede que o jogo carrega (laboratório e partida) |
| `cerebro_unico/run.json`, `tarefa.json` | a configuração do treino, que o jogo também lê |
| `cerebro_unico/ultimo.pt` | o checkpoint completo da r130240, de onde o treino continua (é o `final_15h_r130240.pt` da época) |
| `cerebro_unico/receita_principal_etapa14.json` | a receita da r130240: os argumentos do `levantar_amp.py` |
| `cerebro_unico/ancora_r115231.pt`, `idle_alerta/estavel_dois_cerebros_r109450.pt` | os dois professores que a receita usa |
| `../niveis_iniciais.json` | os níveis iniciais da prova de fogo do levantar, que a receita também usa |

## Pôr a rede no jogo

```bash
rl_modelo/restaurar.sh
```

O script copia `dados/runs/` para `~/.local/share/matter-engine/rl/runs/` (ou para `$MATTER_RL_DATA/runs/`) e não sobrescreve nada que já exista. Depois disso, o laboratório e o jogo carregam a rede sozinhos. A sonda `MatterRlBrainProbe` deve mostrar "rede: cerebro_unico r130240".

## Continuar o treino

- O código do treino está no ramo `rl-base-wip`, que era o worktree `~/Projetos/matter_engine_wip`. O jogo está no `main`, e os dois ramos divergem.
- O lançador é `tools/rl/suite/portao/lancar.sh UNIDADE RECEITA.json TREINO BUILD THREADS ATUALIZACOES`. Ele precisa do módulo nativo compilado do `rl-base-wip` (`BUILD/python`) e do ambiente Python de `tools/rl/setup.sh`.
- A receita tem caminhos absolutos da máquina de 2026 (`/home/dahaka/...`), em `--levantar-fogo-niveis`, `--cerebro-unico-professor` e `--levantar-imitar-professor-ancora`. Se a pasta mudar, ajuste esses três caminhos.
- Ele permitiu no máximo cerca de 8 threads de simulação, com nice 15 e o vigia de temperatura ligado.
