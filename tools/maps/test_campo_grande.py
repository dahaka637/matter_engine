"""Conferência técnica do MAP salvo; não substitui revisão visual das fotografias."""
import json
import re
import unittest

from compilar import ASSETS, UNITS, parse, polygons


class CampoEtapaTests(unittest.TestCase):
    def test_source_scope_topology_dimensions_and_playing_clearance(self):
        entities=parse((ASSETS/'src/campo_grande_01.map').read_text())
        metadata=json.loads((ASSETS/'autoria/campo_grande_01/implantacao.json').read_text())
        self.assertFalse(metadata['canopy'])
        self.assertEqual(metadata['scope'],['campo','arquibancada','bloco_2_andares_5x20','cercas'])
        layers=[p['_tb_name'] for p,_ in entities if '_tb_name' in p]
        self.assertEqual(len(layers),7)
        for forbidden in ('rio','bairro','pavilhao','estrada','calcada','mobiliario','refletores'):
            self.assertFalse(any(re.search(r'\b'+forbidden+r'\b',name.lower()) for name in layers),forbidden)
        field=next(p for p,_ in entities if p.get('classname')=='matter_field')
        self.assertEqual((float(field['length_m']),float(field['width_m'])),(105,68))
        grass_bounds=None
        for props,solids in entities:
            for index,solid in enumerate(solids):
                try: faces=polygons(solid)
                except ValueError as error:self.fail(f"{props.get('_tb_name')} / {index+1}: {error}")
                vertices=[p for _,poly in faces for p in poly]
                lo=[min(v[k] for v in vertices)/UNITS for k in range(3)]
                hi=[max(v[k] for v in vertices)/UNITS for k in range(3)]
                for face,_ in faces:
                    self.assertTrue((ASSETS/'textures'/(face[2]+'.png')).is_file(),face[2])
                if solid[0][2]=='beirario/campo_verde':grass_bounds=(lo,hi)
                if hi[2]>.05 and not props.get('_tb_name','').startswith('01 '):
                    self.assertFalse(lo[0]<57.5-1e-5 and hi[0]>-57.5+1e-5 and
                                     lo[1]<38-1e-5 and hi[1]>-38+1e-5,
                                     (props.get('_tb_name'),index+1,lo,hi))
        lo,hi=grass_bounds
        self.assertAlmostEqual(hi[0]-lo[0],115)
        self.assertAlmostEqual(hi[1]-lo[1],76)
        building=next(s for p,s in entities if p.get('_tb_name','').startswith('04 '))
        slab=polygons(building[0])
        points=[p for _,poly in slab for p in poly]
        self.assertAlmostEqual((max(p[0] for p in points)-min(p[0] for p in points))/UNITS,20)
        self.assertAlmostEqual((max(p[1] for p in points)-min(p[1] for p in points))/UNITS,5)


if __name__=='__main__':unittest.main()
