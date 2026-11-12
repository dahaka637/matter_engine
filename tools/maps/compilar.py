#!/usr/bin/env python3
"""Compila brushes Valve 220 em GLB (metros, Y-up) para o GltfLoader da engine."""
import argparse
import itertools
import json
import math
from pathlib import Path
import re
import struct
import tempfile

from galpao_materiais import SURFACES as INDOOR_SURFACES

ROOT = Path(__file__).resolve().parents[2]
ASSETS = ROOT / 'assets/maps'
UNITS = 32.0
TEXTURE_SIZE = 128
MATERIALS = {
    'concrete': (150, 153, 157), 'brick': (151, 79, 58),
    'wood': (144, 105, 63), 'steel': (123, 137, 148),
    'grass': (72, 112, 48), 'rubber': (53, 56, 62),
}
SURFACES = {f'matter/{name}': dict(physical=name, metallic=.8 if name == 'steel' else 0,
                                 roughness=.5 if name == 'steel' else .9)
            for name in MATERIALS} | INDOOR_SURFACES


def add(a, b): return tuple(x + y for x, y in zip(a, b))
def sub(a, b): return tuple(x - y for x, y in zip(a, b))
def mul(a, s): return tuple(x * s for x in a)
def dot(a, b): return sum(x * y for x, y in zip(a, b))
def cross(a, b):
    return (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
def unit(a):
    length = math.sqrt(dot(a, a))
    if length < 1e-9: raise ValueError('Plano degenerado no brush')
    return mul(a, 1 / length)
def yup(a): return (a[0], a[2], -a[1])


TOKEN = re.compile(r'\s+|//[^\n]*|"(?:\\.|[^"\\])*"|[{}()\[\]]|[^\s{}()\[\]"]+')


def parse(text):
    tokens = [m.group() for m in TOKEN.finditer(text)
              if not m.group().isspace() and not m.group().startswith('//')]
    index = 0

    def take(expected=None):
        nonlocal index
        if index == len(tokens): raise ValueError('Fim inesperado do .map')
        value = tokens[index]
        index += 1
        if expected is not None and value != expected:
            raise ValueError(f'Esperado {expected!r}, recebido {value!r}')
        return value

    def number():
        n = float(take())
        if not math.isfinite(n) or abs(n) > 1e7:
            raise ValueError('Coordenada/atributo nao finito ou fora de escala')
        return n

    def vector(left, right, count):
        take(left)
        result = tuple(number() for _ in range(count))
        take(right)
        return result

    entities = []
    while index < len(tokens):
        take('{')
        props, brushes = {}, []
        while tokens[index] != '}':
            if tokens[index] == '{':
                take('{')
                faces = []
                while tokens[index] != '}':
                    points = [vector('(', ')', 3) for _ in range(3)]
                    material = take().strip('"')
                    u, v = vector('[', ']', 4), vector('[', ']', 4)
                    number()  # Valve 220 grava a rotacao nos eixos UV.
                    scales = (number(), number())
                    if min(abs(s) for s in scales) < 1e-6:
                        raise ValueError('Escala UV zero')
                    normal = unit(cross(sub(points[2], points[0]), sub(points[1], points[0])))
                    faces.append((normal, dot(normal, points[0]), material, u, v, scales))
                take('}')
                if not 4 <= len(faces) <= 64:
                    raise ValueError('Brush deve ter de 4 a 64 planos')
                brushes.append(faces)
            else:
                key, value = take(), take()
                if not key.startswith('"') or not value.startswith('"'):
                    raise ValueError('Propriedades devem estar entre aspas')
                key, value = json.loads(key), json.loads(value)
                if key in props: raise ValueError(f'Propriedade duplicada: {key}')
                props[key] = value
        take('}')
        entities.append((props, brushes))
    return entities


def polygons(faces):
    vertices = []
    for a, b, c in itertools.combinations(faces, 3):
        bc = cross(b[0], c[0])
        determinant = dot(a[0], bc)
        if abs(determinant) < 1e-8: continue
        p = mul(add(add(mul(bc, a[1]), mul(cross(c[0], a[0]), b[1])),
                    mul(cross(a[0], b[0]), c[1])), 1 / determinant)
        if all(dot(f[0], p) <= f[1] + 1e-4 for f in faces):
            if not any(dot(sub(p, q), sub(p, q)) < 1e-8 for q in vertices):
                vertices.append(p)
    if len(vertices) < 4: raise ValueError('Brush vazio/aberto ou planos invertidos')
    result, edges = [], {}
    for face in faces:
        n, d = face[:2]
        ids = [i for i, p in enumerate(vertices) if abs(dot(n, p) - d) < 1e-4]
        if len(ids) < 3: raise ValueError('Plano redundante ou brush aberto')
        center = mul(tuple(sum(vertices[i][j] for i in ids) for j in range(3)), 1 / len(ids))
        u = unit(sub(vertices[ids[0]], center))
        v = cross(n, u)
        ids.sort(key=lambda i: math.atan2(dot(sub(vertices[i], center), v), dot(sub(vertices[i], center), u)))
        for a, b in zip(ids, ids[1:] + ids[:1]):
            key = tuple(sorted((a, b)))
            edges[key] = edges.get(key, 0) + 1
        result.append((face, [vertices[i] for i in ids]))
    if any(count != 2 for count in edges.values()):
        raise ValueError('Brush nao fecha um volume convexo')
    return result


def finite_values(text, count):
    values = tuple(float(n) for n in text.split())
    if len(values) != count or not all(math.isfinite(n) and abs(n) < 1e7 for n in values):
        raise ValueError(f'Vetor invalido: {text}')
    return values


def compile_map(source):
    entities = parse(source.read_text())
    if not entities or entities[0][0].get('classname') != 'worldspawn':
        raise ValueError('O primeiro objeto deve ser worldspawn')
    if sum(p.get('classname') == 'worldspawn' for p, _ in entities) != 1:
        raise ValueError('Deve existir um unico worldspawn')
    groups, markers, lights, textures, brush_count = {}, [], [], {}, 0

    def texture_info(material):
        data_maps = {s.get(k) for s in SURFACES.values() for k in ('normal', 'mr')}
        if material not in SURFACES and material not in data_maps: raise ValueError(f'Material desconhecido: {material}')
        if material not in textures:
            data = (ASSETS / 'textures' / (material + '.png')).read_bytes()
            if len(data) < 24 or data[:8] != b'\x89PNG\r\n\x1a\n': raise ValueError(f'PNG invalido: {material}')
            width, height = struct.unpack_from('>II', data, 16)
            if not 1 <= width <= 4096 or not 1 <= height <= 4096:
                raise ValueError(f'Dimensoes de textura invalidas: {material}')
            textures[material] = data, width, height
        return textures[material]
    for props, brushes in entities:
        classname = props.get('classname')
        if classname in ('worldspawn', 'func_detail') or (classname == 'func_group' and props.get('_tb_type') in ('_tb_group', '_tb_layer')):
            for brush in brushes:
                brush_count += 1
                for face, points in polygons(brush):
                    n, _, material, u, v, scales = face
                    if material not in SURFACES: raise ValueError(f'Material desconhecido: {material}')
                    _, width, height = texture_info(material)
                    positions, normals, uvs, indices = groups.setdefault(material, ([], [], [], []))
                    first = len(positions)
                    for p in points:
                        positions.append(yup(mul(p, 1 / UNITS)))
                        normals.append(yup(n))
                        uvs.append(((dot(p, u[:3]) / scales[0] + u[3]) / width,
                                    (dot(p, v[:3]) / scales[1] + v[3]) / height))
                    for i in range(1, len(points) - 1): indices.extend((first, first+i, first+i+1))
        elif classname in ('info_player_start', 'matter_field', 'light'):
            if brushes: raise ValueError(f'{classname} nao aceita brushes')
            position = yup(mul(finite_values(props.get('origin', '0 0 0'), 3), 1 / UNITS))
            angle = finite_values(props.get('angle', '0'), 1)[0]
            # -90 graus alinha o +Y frontal da engine com o +X do editor.
            yaw = math.radians(angle - 90) / 2
            node = {'name': classname, 'translation': position,
                    'rotation': [0, math.sin(yaw), 0, math.cos(yaw)],
                    'extras': {'entity_type': classname, **{k: v for k, v in props.items() if not k.startswith('_tb_')}}}
            if classname == 'matter_field':
                for key, default in [('length_m', '105'), ('width_m', '68')]:
                    value = finite_values(props.get(key, default), 1)[0]
                    if not 1 <= value <= 200: raise ValueError(f'{key} fora de 1..200 metros')
                    node['extras'][key] = value
            elif classname == 'light':
                color = finite_values(props.get('_color', '1 1 1'), 3)
                intensity = finite_values(props.get('intensity', '550'), 1)[0]
                light_range = finite_values(props.get('range', '20'), 1)[0]
                if not all(0 <= c <= 1 for c in color) or not 0 < intensity <= 100000 or not 0 < light_range <= 200:
                    raise ValueError('Cor, intensidade ou alcance de luz invalido')
                node['extensions'] = {'KHR_lights_punctual': {'light': len(lights)}}
                kind = props.get('type', 'point')
                if kind not in ('point', 'spot'): raise ValueError('Tipo de luz deve ser point ou spot')
                light = {'type': kind, 'color': color, 'intensity': intensity, 'range': light_range}
                if kind == 'spot':
                    inner = finite_values(props.get('inner_cone', '80'), 1)[0]
                    outer = finite_values(props.get('outer_cone', '110'), 1)[0]
                    pitch = finite_values(props.get('pitch', '-90'), 1)[0]
                    if not 0 <= inner < outer <= 178 or not -90 <= pitch <= 90:
                        raise ValueError('Cone/pitch de spot invalido')
                    light['spot'] = {'innerConeAngle': math.radians(inner / 2), 'outerConeAngle': math.radians(outer / 2)}
                    tilt = math.radians(pitch) / 2
                    cy,sy,cp,sp = math.cos(yaw),math.sin(yaw),math.cos(tilt),math.sin(tilt)
                    node['rotation'] = [cy*sp,sy*cp,-sy*sp,cy*cp]
                lights.append(light)
            markers.append(node)
        else:
            raise ValueError(f'Entidade ainda nao suportada: {classname}')
    if not groups: raise ValueError('Mapa sem geometria')
    if not any(n['name'] == 'info_player_start' for n in markers):
        raise ValueError('Mapa precisa de info_player_start')
    document = {'asset': {'version': '2.0', 'generator': 'Matter map compiler v1'},
                'scene': 0, 'scenes': [{'nodes': []}], 'nodes': [], 'meshes': [],
                'materials': [], 'textures': [], 'images': [],
                'samplers': [{'magFilter': 9729, 'minFilter': 9987, 'wrapS': 10497, 'wrapT': 10497}],
                'accessors': [], 'bufferViews': []}
    binary = bytearray()

    def view(data, target=None):
        binary.extend(b'\0' * (-len(binary) % 4))
        info = {'buffer': 0, 'byteOffset': len(binary), 'byteLength': len(data)}
        if target: info['target'] = target
        document['bufferViews'].append(info)
        binary.extend(data)
        return len(document['bufferViews']) - 1

    def accessor(values, size, integer=False):
        rows = [(v,) for v in values] if size == 1 else values
        flattened = [v for row in rows for v in row]
        index = view(struct.pack('<' + ('I' if integer else 'f') * len(flattened), *flattened), 34963 if integer else 34962)
        info = {'bufferView': index, 'componentType': 5125 if integer else 5126,
                'count': len(rows), 'type': {1: 'SCALAR', 2: 'VEC2', 3: 'VEC3'}[size]}
        if size == 3:
            info['min'] = [min(r[i] for r in rows) for i in range(size)]
            info['max'] = [max(r[i] for r in rows) for i in range(size)]
        document['accessors'].append(info)
        return len(document['accessors']) - 1

    # shortcut: preserva faces internas entre brushes, fazer CSG/culling se mapas reais justificarem o custo.
    for material, (positions, normals, uvs, indices) in sorted(groups.items()):
        name = material.split('/')[1]
        surface = SURFACES[material]
        image_index = len(document['images'])
        document['images'].append({'bufferView': view(texture_info(material)[0]), 'mimeType': 'image/png'})
        document['textures'].append({'source': image_index, 'sampler': 0})
        output_material = {'name': name, 'pbrMetallicRoughness': {
            'baseColorTexture': {'index': image_index}, 'metallicFactor': surface['metallic'],
            'roughnessFactor': surface['roughness']},
            'extras': {'physical_material': surface['physical'], 'collision_mode': 'static_mesh'}}
        for key, field in [('normal', 'normalTexture'), ('mr', 'metallicRoughnessTexture')]:
            if key in surface:
                image_data, _, _ = texture_info(surface[key])
                index = len(document['images'])
                document['images'].append({'bufferView': view(image_data), 'mimeType': 'image/png'})
                document['textures'].append({'sampler': 0, 'source': index})
                target = output_material if key == 'normal' else output_material['pbrMetallicRoughness']
                target[field] = {'index': index}
        if 'parallax' in surface:
            output_material['extras']['parallax_depth'] = surface['parallax']
        if 'emissive' in surface:
            output_material['emissiveFactor'] = surface['emissive']
            output_material['emissiveTexture'] = {'index': image_index}
            output_material['extensions'] = {'KHR_materials_emissive_strength': {'emissiveStrength': surface['strength']}}
            document.setdefault('extensionsUsed', []).append('KHR_materials_emissive_strength')
        document['materials'].append(output_material)
        primitive = {'attributes': {'POSITION': accessor(positions, 3), 'NORMAL': accessor(normals, 3),
                                    'TEXCOORD_0': accessor(uvs, 2)},
                     'indices': accessor(indices, 1, True), 'material': len(document['materials']) - 1, 'mode': 4}
        document['meshes'].append({'primitives': [primitive]})
        document['nodes'].append({'name': material, 'mesh': len(document['meshes']) - 1,
                                  'rotation': [0, 0, 0, 1]})
    document['nodes'].extend(markers)
    if lights:
        document.setdefault('extensionsUsed', []).append('KHR_lights_punctual')
        document['extensions'] = {'KHR_lights_punctual': {'lights': lights}}
    if 'extensionsUsed' in document:
        document['extensionsUsed'] = sorted(set(document['extensionsUsed']))
    document['scenes'][0]['nodes'] = list(range(len(document['nodes'])))
    binary.extend(b'\0' * (-len(binary) % 4))
    document['buffers'] = [{'byteLength': len(binary)}]
    data = json.dumps(document, separators=(',', ':'), allow_nan=False).encode()
    data += b' ' * (-len(data) % 4)
    glb = (struct.pack('<4sII', b'glTF', 2, 28+len(data)+len(binary))
           + struct.pack('<I4s', len(data), b'JSON') + data
           + struct.pack('<I4s', len(binary), b'BIN\0') + binary)
    report = {'brushes': brush_count, 'triangles': sum(len(g[3])//3 for g in groups.values()),
              'materials': len(groups), 'entities': len(markers), 'bytes': len(glb)}
    return glb, report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapa', type=Path)
    parser.add_argument('--saida', type=Path)
    args = parser.parse_args()
    output = args.saida or ASSETS / 'compiled' / (args.mapa.stem + '.glb')
    try:
        if output.resolve() == args.mapa.resolve(): raise ValueError('Saida nao pode sobrescrever o .map')
        glb, report = compile_map(args.mapa)
        output.parent.mkdir(parents=True, exist_ok=True)
        with tempfile.NamedTemporaryFile(dir=output.parent, delete=False) as tmp:
            tmp.write(glb)
            temporary = Path(tmp.name)
        temporary.replace(output)
        print(json.dumps({'output': str(output), **report}, ensure_ascii=False))
    except (ValueError, OSError, IndexError) as error:
        parser.exit(1, f'Mapa rejeitado: {error}\n')


if __name__ == '__main__': main()
