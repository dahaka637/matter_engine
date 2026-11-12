#!/usr/bin/env python3
"""Audita contatos, brushes duplicados e faces coplanares de um MAP estatico."""
import argparse
from collections import defaultdict
from functools import lru_cache
import json
from pathlib import Path

from compilar import parse, polygons, dot, cross, sub, mul, unit, UNITS

EPS = 1e-4  # metros; tolerancia de contato, nao uma folga de montagem.


def directions(vectors):
    result = set()
    for v in vectors:
        if dot(v,v) < 1e-12: continue
        n = unit(v)
        if next(c for c in n if abs(c) > 1e-8) < 0: n = mul(n,-1)
        result.add(tuple(round(c,8) for c in n))
    return tuple(sorted(result))


@lru_cache(maxsize=512)
def separating_axes(normals_a, edges_a, normals_b, edges_b):
    return directions(list(normals_a)+list(normals_b)+[cross(a,b) for a in edges_a for b in edges_b])


def shape(faces, label):
    surfaces = [(f[0],f[1]/UNITS,[mul(p,1/UNITS) for p in poly]) for f,poly in polygons(faces)]
    vertices = tuple(sorted({tuple(round(c,8) for c in p) for _,_,poly in surfaces for p in poly}))
    edges = [sub(b,a) for _,_,poly in surfaces for a,b in zip(poly,poly[1:]+poly[:1])]
    return dict(label=label, material=faces[0][2], surfaces=surfaces, vertices=vertices,
                normals=directions(f[0] for f in faces), edges=directions(edges), projections={},
                lo=tuple(min(p[i] for p in vertices) for i in range(3)),
                hi=tuple(max(p[i] for p in vertices) for i in range(3)))


def contact(a,b):
    # SAT completo: so AABBs daria falsos contatos em tirantes inclinados.
    for axis in separating_axes(a['normals'],a['edges'],b['normals'],b['edges']):
        intervals=[]
        for item in (a,b):
            if axis not in item['projections']:
                values=[dot(p,axis) for p in item['vertices']]
                item['projections'][axis]=(min(values),max(values))
            intervals.append(item['projections'][axis])
        x,y=intervals
        if x[1]<y[0]-EPS or y[1]<x[0]-EPS: return False
    return True


def overlap_area(poly, clip):
    def side(a,b,p): return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])
    def area(points):
        return sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(points,points[1:]+points[:1]))/2
    if area(clip)<0: clip=clip[::-1]
    for a,b in zip(clip,clip[1:]+clip[:1]):
        output=[]
        for p,q in zip(poly,poly[1:]+poly[:1]):
            sp,sq=side(a,b,p),side(a,b,q)
            if sp>=0: output.append(p)
            if (sp<0)!=(sq<0):
                t=sp/(sp-sq)
                output.append(tuple(p[i]+t*(q[i]-p[i]) for i in range(2)))
        poly=output
        if not poly: return 0
    return abs(area(poly))


def coplanar_area(a,b):
    area=0
    for n,d,poly in a['surfaces']:
        for m,e,other in b['surfaces']:
            if dot(n,m)<1-1e-9 or abs(d-e)>1e-5: continue
            drop=max(range(3),key=lambda i:abs(n[i]))
            project=lambda points:[tuple(p[i] for i in range(3) if i!=drop) for p in points]
            area+=overlap_area(project(poly),project(other))/abs(n[drop])
    return area


def audit(text):
    shapes=[]
    for props,brushes in parse(text):
        for index,faces in enumerate(brushes,1):
            shapes.append(shape(faces,f"{props.get('_tb_name',props.get('classname'))} / {index}"))
    if not shapes: raise ValueError('Mapa sem brushes estaticos')
    parents=list(range(len(shapes)))
    def root(i):
        while parents[i]!=i:
            parents[i]=parents[parents[i]]
            i=parents[i]
        return i
    seen,duplicates={},[]
    for i,item in enumerate(shapes):
        key=tuple(tuple(round(c,6) for c in p) for p in item['vertices'])
        if key in seen: duplicates.append((seen[key],i))
        seen[key]=i
    active=[]
    coplanars=[]
    for i in sorted(range(len(shapes)),key=lambda i:shapes[i]['lo'][0]):
        a=shapes[i]
        active=[j for j in active if shapes[j]['hi'][0]>=a['lo'][0]-EPS]
        for j in active:
            b=shapes[j]
            if any(a['hi'][k]<b['lo'][k]-EPS or b['hi'][k]<a['lo'][k]-EPS for k in (1,2)): continue
            if root(i)!=root(j) and contact(a,b): parents[root(i)]=root(j)
            area=coplanar_area(a,b)
            # shortcut: fios cruzados abaixo de 2 cm2 sao omitidos; revisar se engrossarem.
            if area>.0002: coplanars.append((j,i,round(area,6)))
        active.append(i)
    groups=defaultdict(list)
    for i in range(len(shapes)): groups[root(i)].append(i)
    def describe(i):
        item=shapes[i]
        return dict(brush=i,layer_brush=item['label'],material=item['material'],
                    center=[round((a+b)/2,4) for a,b in zip(item['lo'],item['hi'])])
    return dict(brushes=len(shapes),
                # O primeiro brush e a fundacao do mapa; os demais precisam alcança-la por contatos.
                detached=[[describe(i) for i in group] for r,group in groups.items() if r!=root(0)],
                duplicates=[[describe(a),describe(b)] for a,b in duplicates],
                coplanar_candidates=[dict(a=describe(a),b=describe(b),area_m2=area) for a,b,area in coplanars])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mapa',type=Path)
    args=parser.parse_args()
    result=audit(args.mapa.read_text())
    print(json.dumps(result,ensure_ascii=False,indent=2))
    # Coplanares requerem conferir visibilidade: ha encontros internos intencionais.
    raise SystemExit(bool(result['detached'] or result['duplicates']))
