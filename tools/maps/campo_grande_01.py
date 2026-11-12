#!/usr/bin/env python3
"""Etapa 1: campo, arquibancada descoberta/bloco 5x20 e alambrado. Sem engine."""
import argparse
from collections import defaultdict
from datetime import datetime
import json
import math
import shutil

from compilar import ASSETS, UNITS, add, sub, mul, cross, dot, unit
from novo import brush
from campo_materiais import SURFACES, generate

NAME = 'campo_grande_01'
FIELD = (105, 68)
GRASS = (-57.5, -38, 57.5, 38)
FENCE = (-57.6, -38.1, 57.6, 38.1)
STAND = (-28, -47.8, 17, -39.8)
SUPPORT = (17, -44.8, 37, -39.8)


def build():
    layers = defaultdict(list)

    def solid(layer, planes, material, top=None):
        # +X fica à esquerda na vista do campo: espelhar para o bloco ficar à direita, como na foto.
        planes = [((-n[0],n[1],n[2]),(-p[0],p[1],p[2])) for n,p in planes]
        lines = brush([(n, mul(p, UNITS)) for n, p in planes], 'beirario/'+material).splitlines()
        for i, (n, _) in enumerate(planes, 1):
            name = top if top and n[2] > .99 else material
            scale = SURFACES[name][4] * UNITS / 512
            lines[i] = lines[i].replace('beirario/'+material, 'beirario/'+name).replace(
                ' 0 0.25 0.25', f' 0 {scale:.9g} {scale:.9g}')
        layers[layer].append('\n'.join(lines))

    def block(layer, lo, hi, material, top=None):
        if layer.startswith('04 ') and lo[2]==0 and hi[2]>.10:
            lo=(*lo[:2],.10)
        assert all(b > a for a, b in zip(lo, hi)), (lo, hi)
        planes = []
        for k in range(3):
            for s, bound in ((-1, lo[k]), (1, hi[k])):
                n, p = [0,0,0], [0,0,0]
                n[k], p[k] = s, bound
                planes.append((tuple(n), tuple(p)))
        solid(layer, planes, material, top)

    def section(layer, profile, y0, y1, material):
        # Perfil convexo X/Z, anti-horário; extrusão em Y para lajes, empenas e vigas.
        planes = [((0,-1,0),(0,y0,0)), ((0,1,0),(0,y1,0))]
        for a,b in zip(profile, profile[1:]+profile[:1]):
            n = unit((b[1]-a[1], 0, a[0]-b[0]))
            planes.append((n,(a[0],0,a[1])))
        solid(layer, planes, material)

    def beam(layer, a, b, width, depth, material):
        axis = unit(sub(b,a))
        u = unit(cross((0,0,1) if abs(axis[2]) < .95 else (0,1,0),axis))
        v = cross(axis,u)
        center = mul(add(a,b),.5)
        planes = []
        for n, extent in ((axis,math.dist(a,b)/2), (u,width/2), (v,depth/2)):
            for s in (-1,1): planes.append((mul(n,s),add(center,mul(n,s*extent))))
        solid(layer,planes,material)

    ground='01 Campo e bases'
    block(ground,(-57.7,-38.2,-.30),(57.7,38.2,-.18),'concreto')
    block(ground,(-57.5,-38,-.18),(57.5,38,0),'campo_verde')
    # Apenas o embasamento e acesso da arquibancada; sem terreno/ruas/passeios externos.
    block(ground,(-28.25,-48.05,-.30),(17,-38.2,0),'concreto')
    block(ground,(17,-45.05,-.30),(37.25,-38.2,0),'concreto')
    block(ground,(-28.25,-39.8,0),(37.25,-38.2,.045),'concreto')

    terrace='02 Arquibancada - degraus e escadas'
    frame='03 Arquibancada - estrutura posterior'
    x0,yback,x1,yfront=STAND
    rows, tread, rise = 10,.75,.34
    aisles=[(-14.1,-12.9),(.9,2.1)]
    sectors=[(x0,aisles[0][0]),(aisles[0][1],aisles[1][0]),(aisles[1][1],x1-.20)]
    for row in range(rows):
        front=yfront-row*tread
        rear=front-tread
        z=(row+1)*rise
        for a,b in sectors:
            block(terrace,(a,rear,z-.16),(b,front,z),'concreto_assento')
            block(terrace,(a,front-.13,max(.04,z-rise-.16)),(b,front,z-.16),'concreto')
        for a,b in aisles:
            for half in range(2):
                zz=row*rise+(half+1)*rise/2
                yf=front-half*tread/2
                block(terrace,(a,yf-tread/2,.04),(b,yf,zz),'concreto',top='amarelo')
    back_z=rows*rise
    block(terrace,(x0,yback,back_z-.16),(x1-.20,yfront-rows*tread,back_z),'concreto_assento')
    # Vigas inclinadas contínuas sob os patamares, sem faces sobrepostas nos espelhos.
    for x in [x0+.15]+[x0+i*5 for i in range(1,9)]+[x1-.15]:
        slope=rise/tread
        rear=yfront-rows*tread
        solid(frame,[((-1,0,0),(x-.13,0,0)),((1,0,0),(x+.13,0,0)),
                     ((0,-1,0),(0,rear,0)),((0,0,-1),(0,0,.015)),
                     ((0,slope,1),(0,yfront,-.16)),
                     ((0,-slope,-1),(0,yfront,-.66))],'concreto_estrutura')
        block(frame,(x-.13,yback,back_z-.66),(x+.13,rear,back_z-.16),'concreto_estrutura')
        block(frame,(x-.17,yback-.04,0),(x+.17,yback+.26,5.5),'concreto_estrutura')
        for y in (-43.65,-40.3):
            top=.34*(yfront-y)/tread-.12
            if top>.1: block(frame,(x-.17,y-.15,0),(x+.17,y+.15,top),'concreto_estrutura')
    # Parapeito rebocado do lado do campo, alvenaria aparente atrás como na foto.
    column_x=[x0+.15]+[x0+i*5 for i in range(1,9)]+[x1-.15]
    for left,right in zip(column_x,column_x[1:]):
        block(frame,(left+.17,yback,back_z),(right-.17,yback+.18,back_z+.95),'alvenaria_cinza')
        block(frame,(left+.17,yback+.18,back_z),(right-.17,yback+.205,back_z+.95),'reboco_menta')
        block(frame,(left+.17,yback,back_z+.95),(right-.17,yback+.205,back_z+1.02),'concreto_assento')
    # Fechamentos laterais triangulares; a subida acompanha o lance de degraus.
    for x in (x0-.20,x1-.20):
        slope=(back_z+.75-.65)/(yback-yfront)
        solid(frame,[((-1,0,0),(x,0,0)),((1,0,0),(x+.20,0,0)),
                     ((0,-1,0),(0,yback,0)),((0,1,0),(0,yfront,0)),
                     ((0,0,-1),(0,0,0)),((0,-slope,1),(0,yfront,.65))],'reboco_menta')
    # Parede baixa sob a traseira, interrompida por vãos reais (não vidros sobre reboco).
    bays=[(-27.7,-23.15),(-22.85,-18.15),(-17.85,-13.15),(-12.85,-8.15),
          (-7.85,-3.15),(-2.85,1.85),(2.15,6.85),(7.15,11.85),(12.15,16.7)]
    for index,(a,b) in enumerate(bays):
        if index<2:continue  # Dois primeiros vãos de acesso ficam abertos, como na traseira fotografada.
        if index==2:
            opening_l,opening_r=(a+b)/2-.50,(a+b)/2+.50
            for l,r in ((a,opening_l),(opening_r,b)):
                block(frame,(l,yback+.03,0),(r,yback+.23,.75),'rodape_verde')
                block(frame,(l,yback+.03,.75),(r,yback+.23,2.25),'reboco_menta')
            block(frame,(opening_l,yback+.03,2.05),(opening_r,yback+.23,2.25),'reboco_menta')
            continue
        center=(a+b)/2+(.22 if index%2 else -.22)
        half=.60 if index<5 else .75
        block(frame,(a,yback+.03,0),(b,yback+.23,.75),'rodape_verde')
        block(frame,(a,yback+.03,.75),(b,yback+.23,1.5),'reboco_menta')
        block(frame,(a,yback+.03,1.5),(center-half,yback+.23,2.25),'reboco_menta')
        block(frame,(center+half,yback+.03,1.5),(b,yback+.23,2.25),'reboco_menta')
        block(frame,(center-half,yback+.03,1.88),(center+half,yback+.23,2.25),'reboco_menta')
        block(frame,(center-half,yback+.10,1.5),(center+half,yback+.14,1.88),'vidro')
    block(frame,(x0,yback+.23,2.25),(x1,yback+.40,2.5),'concreto_estrutura')

    building='04 Bloco 5x20 - alvenaria e empenas'
    finish='05 Bloco 5x20 - esquadrias e cobertura'
    a,b,c,d=SUPPORT
    # O retângulo externo é 20 m na lateral do campo e 5 m de profundidade.
    block(building,(a,b,0),(c,d,.10),'concreto')
    # Parede de fundo com janelas altas e duas janelas verticais.
    openings=[(18.2,21.5,1.95,2.35),(22.1,25.4,1.95,2.35),
              (27.9,28.6,.95,2.45),(29.5,30.0,.95,2.45),(31.3,35.8,1.95,2.35)]
    cuts=sorted({a,c,*[v for o in openings for v in o[:2]]})
    for left,right in zip(cuts,cuts[1:]):
        opening=next((o for o in openings if o[0]<=left and o[1]>=right),None)
        intervals=[(0,opening[2]),(opening[3],2.80)] if opening else [(0,2.8)]
        for low,high in intervals:
            if low<.75: block(building,(left,b,low),(right,b+.20,min(.75,high)),'rodape_verde')
            if high>.75: block(building,(left,b,max(low,.75)),(right,b+.20,high),'reboco_menta')
    def window(left,right,z0,z1,y,outward=-1):
        block(finish,(left,y-.025,z0),(right,y+.025,z0+.045),'aco_escuro')
        block(finish,(left,y-.025,z1-.045),(right,y+.025,z1),'aco_escuro')
        for x in (left,right-.04):block(finish,(x,y-.025,z0+.045),(x+.04,y+.025,z1-.045),'aco_escuro')
        block(finish,(left+.04,y-.006,z0+.045),(right-.04,y+.006,z1-.045),'vidro')
        for i in range(1,max(2,math.ceil((right-left)/.7))):
            x=left+(right-left)*i/max(2,math.ceil((right-left)/.7))
            block(finish,(x-.014,y-.026,z0+.045),(x+.014,y+.026,z1-.045),'aco_escuro')
        # Peitoril ultrapassa a parede só 4 cm, sem chapa solta/flutuante.
        depth=(-.08,.165) if outward>0 else (-.135,.11)
        block(finish,(left-.04,y+depth[0],z0-.05),(right+.04,y+depth[1],z0+.012),'concreto_assento')
    for l,r,z0,z1 in openings:window(l,r,z0,z1,b+.08)
    for x in (a,c-.20):
        block(building,(x,b+.20,0),(x+.20,d,.75),'rodape_verde')
        block(building,(x,b+.20,.75),(x+.20,d,2.80),'reboco_menta')
    # Varanda recuada: portas montadas na parede, com verga/ombreiras e piso de apoio.
    porch_wall=d-1.25
    doors=[(23.2,24.2),(29.8,30.8)]
    cuts=[a+.20,23.2,24.2,29.8,30.8,c-.20]
    for l,r in zip(cuts,cuts[1:]):
        if (l,r) in doors:
            block(building,(l,porch_wall,2.18),(r,porch_wall+.18,2.8),'reboco_menta')
            block(finish,(l+.035,porch_wall+.055,.10),(r-.035,porch_wall+.095,2.14),'porta_verde')
            for x in (l,r-.035):block(finish,(x,porch_wall+.10,.10),(x+.035,porch_wall+.14,2.145),'aco_escuro')
            block(finish,(l,porch_wall+.10,2.145),(r,porch_wall+.14,2.18),'aco_escuro')
            block(finish,(r-.17,porch_wall+.095,1.08),(r-.08,porch_wall+.14,1.11),'metal_zincado')
        else:
            block(building,(l,porch_wall,0),(r,porch_wall+.18,.75),'rodape_verde')
            block(building,(l,porch_wall,.75),(r,porch_wall+.18,2.80),'reboco_menta')
    for x in (22,27,32):
        block(building,(x,d-.20,0),(x+.20,d,.75),'rodape_verde')
        block(building,(x,d-.20,.75),(x+.20,d,2.80),'reboco_menta')
    block(building,(a,b,2.80),(c,d,3.02),'reboco_menta')
    # Pavimento superior central e empenas abertas das alas: identidade da referência.
    upper_l,upper_r=23,31
    block(building,(upper_l,b,3.02),(upper_r,d,3.55),'reboco_menta')
    for y in (b,d-.20):
        rear=abs(y-b)<1e-6
        wl,wr,wz=(27.7,28.9,4.05) if rear else (upper_l+.55,upper_r-.55,3.55)
        block(building,(upper_l,y,3.55),(wl,y+.20,5.40),'reboco_menta')
        block(building,(wr,y,3.55),(upper_r,y+.20,5.40),'reboco_menta')
        if rear:block(building,(wl,y,3.55),(wr,y+.20,wz),'reboco_menta')
        block(building,(wl,y,5.12),(wr,y+.20,5.40),'reboco_menta')
        window(wl,wr,wz,5.12,y+.09,outward=-1 if rear else 1)
        if not rear:
            for i in range(31):
                x=wl+.07+i*.22
                if x>=wr:break
                block(finish,(x-.008,d-.09,wz),(x+.008,d-.072,5.12),'aco_escuro')
            for z in (3.65,4.35,5.02):
                block(finish,(wl,d-.086,z),(wr,d-.068,z+.018),'aco_escuro')
    for x in (upper_l,upper_r-.20):block(building,(x,b+.20,3.55),(x+.20,d-.20,5.40),'reboco_menta')
    for y in (b,d-.20):section(building,[(23,5.40),(31,5.40),(27,6.80)],y,y+.20,'reboco_menta')
    # As grandes vigas triangulares estão no plano das fachadas, não atravessam telhados.
    for y in (b+.06,d-.06):
        beam(building,(a,y,3.02),(23,y,5.288),.20,.25,'reboco_menta')
        beam(building,(31,y,5.288),(c,y,3.02),.20,.25,'reboco_menta')
    # Telhas cinza de fibrocimento: duas águas centrais e pequenas alas baixas.
    for profile in [[(22.75,5.31),(27,6.80),(27,6.92),(22.75,5.43)],
                    [(27,6.80),(31.25,5.31),(31.25,5.43),(27,6.92)],
                    [(a-.15,2.94),(23,3.25),(23,3.36),(a-.15,3.05)],
                    [(31,3.25),(c+.15,2.94),(c+.15,3.05),(31,3.36)]]:
        section(finish,profile,b-.12,d+.12,'fibrocimento')
    block(finish,(26.90,b-.14,6.92),(27.10,d+.14,6.985),'fibrocimento')
    # Fechar a cunha sob as telhas das alas: a cobertura apoia na alvenaria, sem fresta.
    wing_start=a-.15+(3.02-2.94)*6.15/.31
    wing_end=c+.15-(3.02-2.94)*6.15/.31
    for y in (b,d-.20):
        section(building,[(wing_start,3.02),(23,3.02),(23,3.25)],y,y+.20,'reboco_menta')
        section(building,[(31,3.02),(wing_end,3.02),(31,3.25)],y,y+.20,'reboco_menta')


    fence='06 Cercas - muretas e pilares'
    mesh='07 Cercas - tela e portao'
    fx0,fy0,fx1,fy1=FENCE
    # Faixas sem sobreposição nos quatro cantos.
    gate=(18.2,19.6)
    for left,right in ((fx0-.10,gate[0]),(gate[1],fx1+.10)):
        block(fence,(left,fy0-.10,0),(right,fy0+.10,.24),'mureta_antiga')
    block(fence,(gate[0]+.07,fy0-.10,0),(gate[1]-.07,fy0+.10,.045),'concreto')
    block(fence,(fx0-.10,fy1-.10,0),(fx1+.10,fy1+.10,.24),'mureta_antiga')
    for x in (fx0,fx1):block(fence,(x-.10,fy0+.10,0),(x+.10,fy1-.10,.24),'mureta_antiga')

    def corner_direction(p,out):
        if min(abs(p[0]-fx0),abs(p[0]-fx1))<1e-6 and min(abs(p[1]-fy0),abs(p[1]-fy1))<1e-6:
            return (math.copysign(2**-.5,p[0]),math.copysign(2**-.5,p[1]),0)
        return out

    def fence_post(p,out):
        out=corner_direction(p,out)
        # Junção em meia-esquadria: duas peças com o mesmo plano de encontro.
        w,h,lean,up=.14,2.18,.26,.40
        theta=math.atan2(lean,up)
        m=math.tan(theta/2)
        low=(-w/2,h+w/2*m);high=(w/2,h-w/2*m)
        tangent=(-out[1],out[0],0)
        profiles=[ [(-w/2,.10),(w/2,.10),high,low],
                   [low,high,(lean+w/2*math.cos(theta),h+up-w/2*math.sin(theta)),
                    (lean-w/2*math.cos(theta),h+up+w/2*math.sin(theta))] ]
        for poly in profiles:
            planes=[(tangent,add(p,mul(tangent,w/2))),(mul(tangent,-1),add(p,mul(tangent,-w/2)))]
            for u,v in zip(poly,poly[1:]+poly[:1]):
                nn=unit((v[1]-u[1],u[0]-v[0]))
                n=(out[0]*nn[0],out[1]*nn[0],nn[1])
                planes.append((n,(p[0]+out[0]*u[0],p[1]+out[1]*u[0],u[1])))
            solid(fence,planes,'poste_branco')

    def chain_panel(p,q,bottom=.28,top=2.12):
        length=math.dist(p,q)
        along=unit(sub(q,p))
        normal=(-along[1],along[0],0)
        def pt(s,z,offset=0):return (p[0]+along[0]*s+normal[0]*offset,p[1]+along[1]*s+normal[1]*offset,z)
        # Fios cortados no quadro de cada vão. Inclinação dupla e amarrações nos apoios.
        for sign in (-1,1):
            for k in range(math.floor(-top/.18)-1,math.ceil((length+top)/.18)+1):
                intercept=k*.18
                hits=[]
                for s in (0,length):
                    z=sign*(s-intercept)
                    if bottom-1e-8<=z<=top+1e-8:hits.append((s,z))
                for z in (bottom,top):
                    s=intercept+sign*z
                    if 0<=s<=length:hits.append((s,z))
                if len(hits)==2 and math.dist(*hits)>.005:
                    beam(mesh,pt(*hits[0],sign*.002),pt(*hits[1],sign*.002),.0045,.0045,'metal_zincado')
        for z in (bottom,1.2,top):beam(mesh,pt(0,z),pt(length,z),.009,.009,'aco_escuro')

    sides=[((fx0,fy0,0),(fx1,fy0,0),(0,-1,0)),
           ((fx1,fy0,0),(fx1,fy1,0),(1,0,0)),
           ((fx1,fy1,0),(fx0,fy1,0),(0,1,0)),
           ((fx0,fy1,0),(fx0,fy0,0),(-1,0,0))]
    for side,(p,q,out) in enumerate(sides):
        length=math.dist(p,q)
        points=[length*i/math.ceil(length/2.7) for i in range(math.ceil(length/2.7)+1)]
        if side==0:
            gs=[gate[0]-fx0,gate[1]-fx0]
            points=sorted([s for s in points if not gs[0]-.6<s<gs[1]+.6]+gs)
        else:gs=None
        axis=unit(sub(q,p))
        coords=[add(p,mul(axis,s)) for s in points]
        for pp in coords[:-1]:fence_post(pp,out)
        for i,(pp,qq) in enumerate(zip(coords,coords[1:])):
            # Fios apoiados nas pontas inclinadas; nos cantos a inclinação bisseta as laterais.
            for fraction in (.25,.60,.95):
                ends=[]
                for point in (pp,qq):
                    outward=corner_direction(point,out)
                    ends.append((point[0]+outward[0]*.26*fraction,
                                 point[1]+outward[1]*.26*fraction,2.18+.40*fraction))
                beam(mesh,*ends,.005,.005,'aco_escuro')
            if gs and abs(points[i]-gs[0])<1e-6 and abs(points[i+1]-gs[1])<1e-6:
                # Folha fechada, dobradiças/fecho apoiados nos postes; tela dentro do quadro.
                l,r=pp[0]+.09,qq[0]-.09
                y=pp[1]+.012
                for x in (l,r-.035):block(mesh,(x,y-.02,.11),(x+.035,y+.02,2.12),'aco_escuro')
                for z in (.11,2.085):block(mesh,(l+.035,y-.02,z),(r-.035,y+.02,z+.035),'aco_escuro')
                chain_panel((l+.035,y,0),(r-.035,y,0),bottom=.145,top=2.085)
                for z in (.6,1.8):block(mesh,(l-.09,y-.03,z),(l+.01,y+.03,z+.10),'metal_zincado')
                block(mesh,(r-.04,y-.045,1.1),(qq[0]+.03,y+.045,1.14),'aco_escuro')
            else:
                aa=add(pp,mul(axis,.07));bb=add(qq,mul(axis,-.07))
                chain_panel(aa,bb)
    source='// Game: Soccer Fall\n// Format: Valve\n// REFEITO DO ZERO: etapa 1. Sem entorno, sem engine. 32u = 1m.\n'
    source+='{"classname" "worldspawn" "message" "Campo grande - etapa 1" "_authoring_stage" "reconstrucao_etapa_1"}\n'
    for i,(name,solids) in enumerate(sorted(layers.items()),1):
        props={'classname':'func_group','_tb_type':'_tb_layer','_tb_name':name,'_tb_id':str(i),'_tb_layer_sort_index':str(i)}
        source+='\n{\n'+'\n'.join(json.dumps(k)+' '+json.dumps(v,ensure_ascii=False) for k,v in props.items())+'\n'+'\n'.join(solids)+'\n}\n'
    for props in [{'classname':'matter_field','origin':'0 0 0','angle':'0','length_m':'105','width_m':'68'},
                  {'classname':'info_player_start','origin':'0 -896 8','angle':'270'}]:
        source+='\n{\n'+'\n'.join(json.dumps(k)+' '+json.dumps(v) for k,v in props.items())+'\n}\n'
    metadata=dict(name=NAME,revision='reconstrucao_etapa_1',status='aguardando_revisao_visual',
                  scope=['campo','arquibancada','bloco_2_andares_5x20','cercas'],field_m=FIELD,
                  grass_bounds_m=GRASS,fence_bounds_m=FENCE,stand_bounds_m=(-STAND[2],STAND[1],-STAND[0],STAND[3]),
                  support_bounds_m=(-SUPPORT[2],SUPPORT[1],-SUPPORT[0],SUPPORT[3]),
                  support_orientation='20m ao longo da lateral; 5m de profundidade',
                  stand_rows=rows,stand_tread_m=tread,stand_rise_m=rise,canopy=False,
                  layers={k:len(v) for k,v in layers.items()},engine_imported=False,
                  notes=['Versao anterior integralmente rejeitada e removida.',
                         'Bloco 5x20 informado pelo usuario; alturas/vaos inferidos das fotografias.',
                         'Arquibancada 45x8 e demais medidas estimadas, nao levantamento.',
                         'Sem ruas, calcadas externas, casas, rio, mobiliario ou vegetacao.',
                         'Demarcacoes e gols reservados ao futuro procedural.'])
    return source,metadata


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--substituir',action='store_true')
    args=parser.parse_args()
    target=ASSETS/'src'/f'{NAME}.map'
    if target.exists() and not args.substituir:parser.exit(1,'Fonte existente: --substituir preserva backup antes de regerar.\n')
    if target.exists():
        backup=ASSETS/'autoria'/NAME/'versoes'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        backup.mkdir(parents=True,exist_ok=True)
        shutil.copy2(target,backup/target.name)
    generate()
    source,metadata=build()
    target.parent.mkdir(parents=True,exist_ok=True)
    pending=target.with_suffix('.map.pending')
    pending.write_text(source)
    pending.replace(target)
    (ASSETS/'autoria'/NAME/'implantacao.json').write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+'\n')
    print(json.dumps(metadata,indent=2,ensure_ascii=False))
