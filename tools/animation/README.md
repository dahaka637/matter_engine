# Autoria de animações

As animações do personagem são escritas aqui, em código: cada arquivo em
`clips/` é uma função do tempo que devolve uma pose em linguagem anatômica.
A metodologia, os gates e o motivo de cada regra estão em
`docs/ANIMATION_AUTHORING.md`.

```sh
# gera, valida e grava em assets/animations/clips (so o que passar)
python3 tools/animation/build_clips.py
# um clipe, com folha de previa em PNG
python3 tools/animation/build_clips.py natural_idle --preview-dir /tmp/previas
# o que cada eixo de cada junta faz com o corpo
python3 tools/animation/axis_atlas.py
```

O strafe (`clips/strafe.py`) é a exceção: parte da referência Jog Strafe
retargeteada (`assets/animations/source/retargeted/`) e só corrige o que o
corpo exige. A postura dos braços correndo é uma só em todas as passadas
(`RUNNING_ARM_*` em `gait.py`), e a mão relaxada vem dos ossos de dedo da
skin, não dos clipes.

Para um clipe novo, crie `clips/<id>.py` com uma função
`definition(rig) -> ClipDefinition`, ou `definitions(rig)` para uma família.
Use `natural_idle.py` como modelo de pose e `jog.py` como modelo de
passada (`gait.py`). O nome do arquivo tem de ser igual ao id.

Para se inspirar numa referência (FBX/GLB) sem copiá-la, meça-a e compare
com o nosso clipe, número a número:

```sh
blender --background --factory-startup --python-exit-code 1 \
  --python tools/animation/reference_capture.py -- "Slow Run.fbx" \
  --output /tmp/slow_run.capture.json
python3 tools/animation/gait_metrics.py /tmp/slow_run.capture.json \
  assets/animations/clips/jog.matteranim.json
```

Depois de gravar, rode os testes de personagem, porque o corpo simulado tem
de reproduzir a pose:

```sh
cd build-profile
MATTERENGINE_TEST_FILTER=character ./MatterEngineTests
# só o olhar x movimento (andar, recuar, sprint, varredura de direção)
MATTERENGINE_TEST_FILTER=look ./MatterEngineTests
```
