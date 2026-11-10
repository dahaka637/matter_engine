# inspecao: as ferramentas de inspeção dos treinos de RL

O guia completo (cada comando, a regra térmica, como ler as saídas e a rotina da checagem) está no vault:
`/home/dahaka/Área de trabalho/IA_Brain/04_locomocao/ferramentas_de_inspecao_rl.md`. O `--help` de cada módulo traz o
detalhe de cada opção.

Sempre de dentro de `tools/rl`, com `.venv/bin/python -m inspecao.<módulo>`. O módulo nativo vem de `build-rl-n1/python`.

| módulo | para quê | treino |
|---|---|---|
| `inspecao` | a checagem inteira: painel → para o treino → captura e folhas → religa → marcha, fantasma, antes×agora, auditor → `build-profile/tmp/inspecao/<data>/index.md` | para e religa sozinho; rodar em segundo plano, sem `timeout` |
| `painel` | os dados do treino pelo log (força, o que segura cada exercício, pares espelhados, saúde) | pode estar ligado |
| `vigia_treino` | alertas automáticos do log (`--uma-vez` ou em laço) | pode estar ligado |
| `captura` | grava o movimento em `.npz` (`--modos atual 0.3 clipe`) | **parado** |
| `fantasma` | as figuras: `--movimento`, `--equilibrio`, `--pegadas`, `--parado`, `--espelho`, `--padrao` | pode estar ligado (`--malha`: parado) |
| `marcha` | os números da marcha contra o clipe (`--contra`) e o espelho (`--espelho`) | pode estar ligado |
| `auditor` | aponta os defeitos com gravidade, evidência e figura | pode estar ligado |
| `simetria` | a assimetria da própria rede (ação no estado espelhado) | **parado** |

Também: `../espelho.py` (o espelho esquerda↔direita das entradas e das ações; `--teste`) e, no treino,
`levantar_amp.py --simetria W` (a perda de simetria; com 0 só registra `assimetria` no log).

Autotestes: `python -m inspecao.<módulo> --teste` (captura e simetria não têm).
