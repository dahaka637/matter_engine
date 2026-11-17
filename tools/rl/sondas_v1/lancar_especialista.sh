#!/bin/bash
# (14/11 v1) O especialista do levantar de costas: do checkpoint atual do principal, quase so levantar de costas limpo,
# a nota de semelhanca com o clipe no ritmo e o fim cedo quando fica longe dele (DeepMimic). build-base-n106.
# lancar_especialista.sh NOTA LIMIAR SEGUNDOS [THREADS]
R=~/.local/share/matter-engine/rl/runs; P=/home/dahaka/Projetos/matter_engine/tools/rl/.venv/bin/python
S=$R/v1_costas_especialista; mkdir -p $S
if [ ! -f $S/ultimo.pt ]; then
  cp -a $R/cerebro_unico/{run.json,status.json,tarefa.json,teto_estabilidade.json,teto_firmeza.json,teto_semelhanca.json,teto_velocidade.json} $S/ 2>/dev/null
  cp -a $R/cerebro_unico/exports $S/ && cp -a $R/cerebro_unico/ultimo.pt $S/ultimo.pt && cp -a $R/cerebro_unico/ultimo.pt $S/inicio_principal.pt
  tail -n 400 $R/cerebro_unico/log.jsonl > $S/log.jsonl
fi
$P - "$S/receita_s.json" "$1" "$2" "$3" <<'PY'
import json, sys
out, nota, limiar, seg = sys.argv[1:]
d = json.load(open('/home/dahaka/Projetos/matter_engine_wip/tools/rl/receitas/cerebro_unico_quedas.json')); a = d['argumentos']
def put(flag, *vals):
    i = a.index(flag); a[i + 1:i + 1 + len(vals)] = list(vals)
put('--levantar-unreal', '0.85'); put('--comeco-de-pe', '0.05'); put('--locomocao', '0.08'); put('--giro', '0.02')
put('--levantar-unreal-prioridade', '0'); put('--levantar-fogo', '1.0', '0.0', '0.0', '0.0')
put('--taxa-adaptativa', '0.06'); put('--kl-max', '0.25')
a += ['--levantar-unreal-pesos', '1', '0', '0', '0', '--levantar-imitar', '1', '--levantar-imitar-nota', nota,
      '--levantar-imitar-longe', limiar, seg]
d['descricao'] = f'14/11 v1 especialista do de costas: nota {nota}, longe {limiar} por {seg} s'
json.dump(d, open(out, 'w'), ensure_ascii=False, indent=1)
PY
bash /tmp/claude-1000/-home-dahaka--rea-de-trabalho-IA-Brain/8fe86884-eddc-41e6-ada9-9fcf6d9b523b/scratchpad/lancar.sh rl-v1-costas-s $S/receita_s.json v1_costas_especialista build-base-n106 ${4:-4} 0
for i in $(seq 1 50); do sleep 6; grep -aq "^atualizacao=\|Traceback" $S/treino.out 2>/dev/null && break; done
grep -a -E "Traceback|Error|^atualizacao=" $S/treino.out | tail -3 | cut -c1-200; date +%H:%M
