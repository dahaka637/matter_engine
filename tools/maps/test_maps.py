"""python3 -m unittest discover -s tools/maps -v"""
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from compilar import ASSETS, ROOT, compile_map, parse, polygons, dot, cross, sub
from configurar import install
from novo import box, brush, template
from auditar import audit, shape, contact, coplanar_area


def decode(glb):
    magic, version, length = struct.unpack_from('<4sII', glb)
    assert (magic, version, length) == (b'glTF', 2, len(glb))
    size, tag = struct.unpack_from('<I4s', glb, 12)
    assert tag == b'JSON'
    doc = json.loads(glb[20:20+size])
    count, tag = struct.unpack_from('<I4s', glb, 20+size)
    assert tag == b'BIN\0' and count == doc['buffers'][0]['byteLength']
    return doc, glb[28+size:]


class MapsTest(unittest.TestCase):
    def compile_text(self, text):
        with tempfile.TemporaryDirectory(prefix='matter map ') as directory:
            source = Path(directory) / 'teste.map'
            source.write_text(text)
            return compile_map(source)

    def test_scale_axes_normals_and_uv(self):
        glb, report = self.compile_text(template('base'))
        doc, binary = decode(glb)
        self.assertEqual((report['brushes'], report['triangles']), (1, 12))
        position = doc['accessors'][doc['meshes'][0]['primitives'][0]['attributes']['POSITION']]
        self.assertEqual(position['min'], [-8, -0.5, -8])
        self.assertEqual(position['max'], [8, 0, 8])
        for view in doc['bufferViews']:
            self.assertEqual(view['byteOffset'] % 4, 0)
            self.assertLessEqual(view['byteOffset']+view['byteLength'], len(binary))
        self.assertTrue(all(i['mimeType'] == 'image/png' for i in doc['images']))
        self.assertEqual(doc['materials'][0]['extras']['physical_material'], 'concrete')
        spawn = doc['nodes'][-1]
        self.assertEqual(spawn['translation'], [0, 0.25, 0])
        self.assertAlmostEqual(spawn['rotation'][1], -math.sqrt(0.5))
        for brush_faces in parse(template('base'))[0][1]:
            for face, points in polygons(brush_faces):
                self.assertGreater(dot(cross(sub(points[1], points[0]), sub(points[2], points[0])), face[0]), 0)

    def test_ramp_and_rotated_brush(self):
        # Prisma triangular de 2x2m e 1m de altura: topo inclinado, cinco faces.
        planes = [((-1, 0, 0), (0, 0, 0)), ((0, -1, 0), (0, 0, 0)),
                  ((0, 1, 0), (0, 64, 0)), ((0, 0, -1), (0, 0, 0)),
                  ((0.5, 0, 1), (0, 0, 32))]
        ramp = brush(planes)
        faces = parse('{"classname" "worldspawn"\n' + ramp + '}')[0][1][0]
        self.assertEqual([len(p) for _, p in polygons(faces)].count(3), 2)
        # Rotacao e translacao arbitrarias nao mudam topologia.
        a = math.pi / 5
        def rotate(v): return (v[0]*math.cos(a)-v[1]*math.sin(a), v[0]*math.sin(a)+v[1]*math.cos(a), v[2])
        rotated = [(rotate(n), tuple(x+y for x,y in zip(rotate(p), (100, -200, 50)))) for n,p in planes]
        faces = parse('{"classname" "worldspawn"\n' + brush(rotated) + '}')[0][1][0]
        self.assertEqual(len(polygons(faces)), 5)

    def test_beveled_box(self):
        text='{"classname" "worldspawn"'+box((0,0,0),(64,64,64),bevel=2)+'}'
        faces=parse(text)[0][1][0]
        polys=polygons(faces)
        self.assertEqual(len(polys),18)
        points={tuple(round(c,6) for c in p) for _,poly in polys for p in poly}
        self.assertNotIn((0,0,0),points)
        self.assertEqual([min(p[i] for p in points) for i in range(3)],[0,0,0])
        self.assertEqual([max(p[i] for p in points) for i in range(3)],[64,64,64])
        for amount in (-1,32,65):
            with self.assertRaisesRegex(ValueError,'Chanfro'):
                box((0,0,0),(64,64,64),bevel=amount)

    def test_entities_groups_and_unknowns(self):
        text = template('entity') + '{"classname" "matter_field" "origin" "320 640 0" "angle" "90" "length_m" "105" "width_m" "68"}\n'
        glb, report = self.compile_text(text)
        field = decode(glb)[0]['nodes'][-1]
        self.assertEqual(field['translation'], [10, 0, -20])
        self.assertEqual(field['extras']['length_m'], 105)
        self.assertEqual(report['entities'], 2)
        grouped = template('group') + '{"classname" "func_group" "_tb_type" "_tb_group"\n' + box((320,0,0), (352,32,32)) + '}\n'
        self.assertEqual(self.compile_text(grouped)[1]['brushes'], 2)
        with self.assertRaisesRegex(ValueError, 'suportada'):
            self.compile_text(template('bad') + '{"classname" "portal"}')

    def test_pbr_maps_do_not_shift_material_indices(self):
        text = template('pbr').replace('matter/concrete','indoor/gramado_simples')
        text += '{"classname" "func_detail"' + box((320,0,0),(352,32,32),'matter/steel') + '}\n'
        doc,_ = decode(self.compile_text(text)[0])
        grass=next(m for m in doc['materials'] if m['name']=='gramado_simples')
        self.assertIn('normalTexture',grass)
        self.assertIn('metallicRoughnessTexture',grass['pbrMetallicRoughness'])
        self.assertEqual(grass['extras']['parallax_depth'],.004)
        self.assertEqual(len(doc['images']),len(doc['materials'])+2)
        for mesh in doc['meshes']:
            self.assertLess(mesh['primitives'][0]['material'],len(doc['materials']))

    def test_spot_direction_cones_and_validation(self):
        light='{"classname" "light" "type" "spot" "pitch" "-90" "inner_cone" "90" "outer_cone" "120"}'
        doc,_=decode(self.compile_text(template('spot')+light)[0])
        spot=doc['extensions']['KHR_lights_punctual']['lights'][0]
        self.assertEqual(spot['type'],'spot')
        self.assertAlmostEqual(spot['spot']['outerConeAngle'],math.pi/3)
        x,y,z,w=doc['nodes'][-1]['rotation']
        # Quaternion rotates glTF -Z down to -Y (engine -Z).
        self.assertAlmostEqual(-2*(x*z+y*w),0,places=6)
        self.assertAlmostEqual(2*(x*w-y*z),-1,places=6)
        self.assertAlmostEqual(2*(x*x+y*y)-1,0,places=6)
        with self.assertRaisesRegex(ValueError,'Cone'):
            self.compile_text(template('bad_spot')+light.replace('"120"','"180"'))

    def test_bad_input(self):
        text = template('base')
        bad = [text[:-3], text.replace('matter/concrete', 'matter/typo'),
               text.replace('0 0.25 0.25', '0 0 0.25'),
               text.replace('-256', 'nan', 1),
               text.replace('"info_player_start"', '"worldspawn"'),
               text.split('{\n"classname" "info_player_start"')[0],
               text + '{"classname" "matter_field" "width_m" "-1"}',
               text.replace('"message" "base"', '"message" "base" "message" "duplicado"')]
        for item in bad:
            with self.subTest(item=item[-80:]):
                with self.assertRaises((ValueError, IndexError)): self.compile_text(item)
        faces = parse(text)[0][1][0]
        with self.assertRaises(ValueError): polygons(faces[:-1])
        with self.assertRaises(ValueError): polygons(faces + faces[:1])
        with patch.object(Path,'read_bytes',return_value=b'\x89PNG\r\n\x1a\n'):
            with self.assertRaisesRegex(ValueError,'PNG invalido'):
                self.compile_text(text)

    def test_preserve_output_on_error(self):
        with tempfile.TemporaryDirectory(prefix='matter map ') as directory:
            source, output = Path(directory)/'bad.map', Path(directory)/'old.glb'
            source.write_text('{invalid')
            output.write_bytes(b'previous version')
            result = subprocess.run([sys.executable, str(ROOT/'tools/maps/compilar.py'), str(source), '--saida', str(output)], capture_output=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertEqual(output.read_bytes(), b'previous version')

    def test_indoor_material_and_lights(self):
        from galpao_materiais import generate
        generate()
        text = template('lamp').replace('matter/concrete','indoor/luz_led')
        text += '{"classname" "light" "origin" "0 0 224" "_color" "1 .9 .8" "intensity" "650" "range" "24"}'
        doc,binary = decode(self.compile_text(text)[0])
        material = doc['materials'][0]
        self.assertEqual(material['extras']['physical_material'],'steel')
        self.assertEqual(material['extensions']['KHR_materials_emissive_strength']['emissiveStrength'],5)
        self.assertEqual(material['emissiveTexture'],material['pbrMetallicRoughness']['baseColorTexture'])
        light=doc['extensions']['KHR_lights_punctual']['lights'][0]
        self.assertEqual(light['intensity'],650)
        self.assertEqual(doc['nodes'][-1]['translation'],[0,7,0])
        uv_index=doc['meshes'][0]['primitives'][0]['attributes']['TEXCOORD_0']
        accessor=doc['accessors'][uv_index]
        offset=doc['bufferViews'][accessor['bufferView']]['byteOffset']
        uvs=struct.unpack_from('<'+'f'*(accessor['count']*2),binary,offset)
        self.assertEqual(max(uvs),2)  # PNG de 512px com a escala 0,25 do template.
        from galpao_materiais import SURFACES
        for name,spec in SURFACES.items():
            data=(ASSETS/'textures'/f'{name}.png').read_bytes()
            self.assertEqual(struct.unpack_from('>II',data,16),spec.get('size',(512,512)))
        for old,new in [('"650"','"-1"'),('"1 .9 .8"','"2 1 1"'),('"24"','"0"')]:
            with self.assertRaisesRegex(ValueError,'luz invalido'):
                self.compile_text(text.replace(old,new))

    def test_indoor_keeps_the_playing_volume_clear(self):
        from galpao_01 import build
        text,counts=build()
        self.assertGreater(sum(counts.values()),2000)
        field=[p for p,_ in parse(text) if p['classname']=='matter_field']
        self.assertEqual((field[0]['length_m'],field[0]['width_m']),('40','20'))
        self.assertNotIn('indoor/rede_verde',text,'A rede verde antiga deve ser removida')
        doors,geometry,cabinets,fence,firecases,columns=[],[],[],[],[],[]
        for props,brushes in parse(text):
            for faces in brushes:
                points=[p for _,poly in polygons(faces) for p in poly]
                lo=[min(p[i] for p in points)/32 for i in range(3)]
                hi=[max(p[i] for p in points)/32 for i in range(3)]
                geometry.append((lo,hi))
                overlaps=lo[0]<20 and hi[0]>-20 and lo[1]<10 and hi[1]>-10
                if overlaps and hi[2]>.02:
                    self.assertGreater(lo[2],7.2,'Geometria invadiu o volume livre da quadra')
                if faces[0][2]=='indoor/porta_areia': doors.append((lo,hi))
                if any(f[2]=='indoor/madeira_bancos' for f in faces):
                    self.assertLess(hi[0]-lo[0],3,'Ripa sem emendas ou comprimento de fabricacao plausivel')
                if any(f[2]=='indoor/quadro_eletrico' for f in faces): cabinets.append((lo,hi))
                if props.get('_tb_name')=='09 Cerca e portao': fence.append((lo,hi))
                if faces[0][2]=='indoor/vermelho':firecases.append((lo,hi))
                if faces[0][2]=='indoor/estrutura_clara' and lo[2]<.3 and hi[2]>6:columns.append((lo,hi))
        doors.sort()
        for first,second in zip(doors[::2],doors[1::2]):
            left,right=first[0][0],second[1][0]
            front=max(first[1][1],second[1][1])
            for lo,hi in geometry:
                blocks=(lo[0]<right and hi[0]>left and lo[1]<front+.8 and hi[1]>front+.08
                        and lo[2]<2.5 and hi[2]>.04)
                self.assertFalse(blocks,'Geometria bloqueia a frente de uma porta')
        self.assertEqual(len(cabinets),1)
        self.assertTrue(firecases)
        for a,b in firecases:
            for lo,hi in columns:
                self.assertFalse(all(a[i]<hi[i] and b[i]>lo[i] for i in range(3)),
                                 'Armario de incendio atravessa um pilar')
        for a,b in cabinets:
            for lo,hi in fence:
                self.assertFalse(all(a[i]<hi[i] and b[i]>lo[i] for i in range(3)),
                                 'Cerca atravessa o quadro eletrico')
        self.assertTrue(fence,'Falta o fechamento metalico junto ao campo')
        self.assertAlmostEqual(max(hi[2] for lo,hi in fence),5.48,places=4)
        for lo,hi in fence:
            self.assertTrue(hi[0]<=-20 or lo[0]>=20 or hi[1]<=-10 or lo[1]>=10,
                            'Cerca invadiu a area jogavel ou dos gols')
        self.assertTrue(any(lo[0]<-14.2<hi[0] and lo[1]<11.5<hi[1] and lo[2]<1.1<hi[2]
                            for lo,hi in fence),'A folha do portao deve estar fechada no vao')
        # A folha fechada nao invade a circulacao do lado dos bancos.
        access_lo,access_hi=(-14.98,11.65,.05),(-13.45,12.90,2.20)
        for lo,hi in geometry:
            self.assertFalse(all(lo[i]<access_hi[i] and hi[i]>access_lo[i] for i in range(3)),
                             'Portao ou instalacao invade a circulacao externa')

    def test_indoor_signage_has_one_front_and_fitted_uvs(self):
        from galpao_01 import build
        from galpao_materiais import SURFACES
        labels={'indoor/saida':(0,1,0),'indoor/caixa_incendio':(0,1,0),
                'indoor/quadro_eletrico':(0,1,0),'indoor/placa_setor':(0,-1,0)}
        count=0
        for _,brushes in parse(build()[0]):
            for faces in brushes:
                tagged=[f for f in faces if f[2] in labels]
                if not tagged:continue
                self.assertEqual(len(tagged),1,'Arte nao deve repetir nas laterais do objeto')
                count+=1
                for f,poly in polygons(faces):
                    if f[2] not in labels:continue
                    for a,b in zip(f[0],labels[f[2]]):self.assertAlmostEqual(a,b,places=5)
                    width,height=SURFACES[f[2]]['size'] if 'size' in SURFACES[f[2]] else (512,512)
                    u=[(dot(f[3][:3],p)/f[5][0]+f[3][3])/width for p in poly]
                    v=[(dot(f[4][:3],p)/f[5][1]+f[4][3])/height for p in poly]
                    for values in (u,v):
                        self.assertAlmostEqual(min(values),0,places=5)
                        self.assertAlmostEqual(max(values),1,places=5)
        self.assertEqual(count,7)

    def test_assembly_audit(self):
        floor=box((0,0,-32),(160,160,0))
        supported=box((32,32,0),(64,64,32))
        floating=box((96,96,1),(128,128,33))
        source='{"classname" "worldspawn"'+floor+supported+floating+'}'
        report=audit(source)
        self.assertEqual([[i['brush'] for i in g] for g in report['detached']],[[2]])
        report=audit('{"classname" "worldspawn"'+floor+supported+supported+'}')
        self.assertEqual(len(report['duplicates']),1)
        # Barras paralelas inclinadas: as caixas envolventes se cruzam, os solidos nao.
        u,v=(math.sqrt(.5),math.sqrt(.5),0),(-math.sqrt(.5),math.sqrt(.5),0)
        def diagonal(shift):
            center=tuple(c*shift for c in v)
            planes=[]
            for axis,extent in [(u,64),(v,1.6),((0,0,1),1.6)]:
                for sign in (-1,1):
                    planes.append((tuple(c*sign for c in axis),tuple(c+q*extent*sign for c,q in zip(center,axis))))
            faces=parse('{"classname" "worldspawn"'+brush(planes)+'}')[0][1][0]
            return shape(faces,'diagonal')
        a,b=diagonal(0),diagonal(8)
        self.assertTrue(all(a['lo'][i]<b['hi'][i] and b['lo'][i]<a['hi'][i] for i in range(3)))
        self.assertFalse(contact(a,b))
        self.assertTrue(contact(a,diagonal(3.2)))
        def cube(lo,hi):
            return shape(parse('{"classname" "worldspawn"'+box(lo,hi)+'}')[0][1][0],'cubo')
        a=cube((0,0,0),(32,32,32))
        self.assertAlmostEqual(coplanar_area(a,cube((16,0,0),(48,32,32))),2)
        self.assertEqual(coplanar_area(a,cube((32,0,0),(64,32,32))),0)

    def test_indoor_assemblies_are_supported(self):
        from galpao_01 import build
        report=audit(build()[0])
        self.assertEqual(report['detached'],[])
        self.assertEqual(report['duplicates'],[])
        for pair in report['coplanar_candidates']:
            self.assertFalse(any(pair[s]['layer_brush'].startswith('09 ') for s in ('a','b')),
                             'Faces coincidentes na cerca ou no portao: '+str(pair))
            a,b=pair['a'],pair['b']
            if b['material']=='indoor/protecao_verde': a,b=b,a
            if a['material']=='indoor/protecao_verde' and b['layer_brush'].startswith('07 '):
                self.fail('Instalacao coplanar com a protecao verde: '+str(pair))

    @patch('galpao_materiais.generate')
    def test_install_preserves_preferences_and_other_profiles(self, _generate):
        with tempfile.TemporaryDirectory() as directory:
            data = Path(directory)
            (data/'Preferences.json').write_text('{"unrelated": true}')
            game = install(data)
            profile = game/'CompilationProfiles.cfg'
            config = json.loads(profile.read_text())
            config['profiles'].append({'name': 'Meu perfil', 'tasks': [], 'workdir': '.'})
            profile.write_text(json.dumps(config))
            install(data)
            self.assertTrue(json.loads((data/'Preferences.json').read_text())['unrelated'])
            self.assertEqual(len(json.loads(profile.read_text())['profiles']), 2)
            self.assertEqual(json.loads((data/'Preferences.json.antes_soccer_fall').read_text()), {'unrelated': True})


if __name__ == '__main__': unittest.main()
