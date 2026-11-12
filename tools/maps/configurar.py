#!/usr/bin/env python3
"""Instala o perfil Soccer Fall no TrenchBroom sem alterar os outros jogos."""
import json
from pathlib import Path
import shutil
import struct
import sys
import zlib

from compilar import ROOT, ASSETS, MATERIALS, TEXTURE_SIZE


def png(name, color):
    raw = bytearray()
    for y in range(TEXTURE_SIZE):
        raw.append(0)
        for x in range(TEXTURE_SIZE):
            noise = ((x * 73856093 ^ y * 19349663) % 13) - 6
            seam = x % 32 == 0 or y % 32 == 0
            if name == 'brick': seam = y % 32 < 2 or (x + (y // 32 % 2) * 32) % 64 < 2
            shade = -24 if seam else noise
            raw.extend(max(0, min(255, c + shade)) for c in color)
    def chunk(tag, data):
        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data))
    return (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', TEXTURE_SIZE, TEXTURE_SIZE, 8, 2, 0, 0, 0))
            + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))


def install(userdata=None):
    userdata = userdata or Path.home() / '.TrenchBroom'
    game = userdata / 'games/SoccerFall'
    if game.is_symlink(): raise ValueError(f'Perfil existente e um link: {game}')
    preferences = userdata / 'Preferences.json'
    prefs = json.loads(preferences.read_text()) if preferences.exists() else {}
    game.mkdir(parents=True, exist_ok=True)
    # Uma copia inicial preserva as preferencias anteriores a esta integracao.
    if preferences.exists() and not preferences.with_suffix('.json.antes_soccer_fall').exists():
        shutil.copy2(preferences, preferences.with_suffix('.json.antes_soccer_fall'))
    for name in ('GameConfig.cfg', 'SoccerFall.fgd'):
        shutil.copy2(Path(__file__).parent / 'trenchbroom' / name, game / name)
    shutil.copy2(ASSETS / 'src/integracao.map', game / 'initial.map')
    destination = ASSETS / 'textures/matter'
    destination.mkdir(parents=True, exist_ok=True)
    for name, color in MATERIALS.items(): (destination / (name + '.png')).write_bytes(png(name, color))
    from galpao_materiais import generate
    generate()
    prefs['Games/Soccer Fall/Path'] = str(ASSETS)
    temporary = preferences.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(prefs, indent=4, ensure_ascii=False) + '\n')
    temporary.replace(preferences)
    output = ASSETS / 'compiled'
    output.mkdir(parents=True, exist_ok=True)
    profiles = {'version': 1, 'profiles': [{
        'name': 'Matter - Compilar GLB', 'workdir': str(ROOT), 'tasks': [
            {'type': 'export', 'enabled': True, 'stripTbProperties': False,
             'target': '${WORK_DIR_PATH}/assets/maps/compiled/${MAP_BASE_NAME}.map'},
            {'type': 'tool', 'enabled': True, 'treatNonZeroResultCodeAsError': True,
             'tool': sys.executable,
             'parameters': '"${WORK_DIR_PATH}/tools/maps/compilar.py" "${WORK_DIR_PATH}/assets/maps/compiled/${MAP_BASE_NAME}.map"'},
            {'type': 'tool', 'enabled': True, 'treatNonZeroResultCodeAsError': True,
             'tool': '/bin/bash',
             'parameters': '"${WORK_DIR_PATH}/tools/maps/verificar_engine.sh" "${WORK_DIR_PATH}/assets/maps/compiled/${MAP_BASE_NAME}.glb"'},
        ]}]}
    profile = game / 'CompilationProfiles.cfg'
    if profile.exists():
        existing = json.loads(profile.read_text())
        profiles['profiles'] = [p for p in existing['profiles'] if p['name'] != 'Matter - Compilar GLB'] + profiles['profiles']
    profile.write_text(json.dumps(profiles, indent=4) + '\n')
    launcher = Path.home() / '.local/share/applications/soccer-fall-mapas.desktop'
    if userdata == Path.home() / '.TrenchBroom':
        editor = shutil.which('trenchbroom') or str(Path.home() / '.local/bin/trenchbroom')
        if not Path(editor).exists(): raise ValueError('TrenchBroom nao encontrado')
        start_map=ASSETS / 'src/galpao_01.map'
        if not start_map.exists(): start_map=ASSETS / 'src/integracao.map'
        launcher.parent.mkdir(parents=True, exist_ok=True)
        launcher.write_text('[Desktop Entry]\nType=Application\nName=Soccer Fall - Mapas\n'
                            'Comment=TrenchBroom configurado para a Matter Engine\n'
                            f'Exec="{editor}" "{start_map}"\n'
                            'Icon=trenchbroom\nTerminal=false\nCategories=Development;Graphics;\n')
        launcher.chmod(0o755)
    return game


if __name__ == '__main__':
    try: print(f'Perfil instalado: {install()}')
    except (OSError, ValueError) as error: sys.exit(str(error))
