Fonte: Mixamo (Adobe), baixados pelo usuario em 13/10/2026 para as viradas
correndo do RL (o tiro com volta, etapa 4a do plano de imitacao).

- RunningTurn180.fbx  ("Running Turn 180"): a meia-volta repentina. Veio
  no lugar (In Place): gira 180 graus sem andar (0,04 m em 0,67 s).
- RunningToTurn.fbx   ("Running To Turn"): corre, freia, vira (174 graus) e
  termina parado; anda 4,1 m em 2,27 s.
- TurnToRunning.fbx   ("Turn To Running"): parado, vira e sai correndo;
  anda 3,3 m em 1,70 s.

Licenca: termos padrao do Mixamo/Adobe (nao CC0), como os outros desta pasta.
Clipes gerados (assets/animations/clips/running_turn_180, running_to_turn,
turn_to_running) com:

  env PYTHONHOME=/usr blender --background --factory-startup \
    --python-use-system-env --python-exit-code 1 \
    --python tools/import_humanoid_animation.py -- <fbx> --source-rig mixamo \
    --profile assets/characters/football_player/FootballPlayerV1.ragdoll.json \
    --root-motion preserve --dynamic-projection --output <clipe> --id <id> --name <nome>

O --dynamic-projection foi preciso (como no Sprint): sem ele o joelho de um
quadro passa do teto de 8 graus de erro de membro (14,8 a 17,7 graus).
