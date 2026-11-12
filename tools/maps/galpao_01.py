#!/usr/bin/env python3
"""Constroi o mapa indoor editavel, em metros; sem linhas nem traves."""
import argparse
from collections import defaultdict
import json
import math
from pathlib import Path

from compilar import ASSETS, UNITS, add, sub, mul, unit, cross
from novo import box, brush
from galpao_materiais import generate, SURFACES

NAME = 'galpao_01'
FIELD = (40, 20)
FENCE_HALF = (22.4, 11.5)
FENCE_HEIGHT = 5.4
GATE_X = (-14.9, -13.5)


def build():
    layers = defaultdict(list)
    counts = defaultdict(int)

    def solid(layer, planes, material):
        geometry=brush([(n, mul(p, UNITS)) for n,p in planes], 'indoor/'+material)
        layers[layer].append(geometry.replace(' 0 0.25 0.25',' 0 0.125 0.125'))
        counts[layer] += 1

    def block(layer, lo, hi, material, bevel=0):
        assert all(a < b for a,b in zip(lo,hi)), (lo,hi)
        geometry=box(mul(lo,UNITS), mul(hi,UNITS), 'indoor/'+material, bevel*UNITS)
        geometry=geometry.replace(' 0 0.25 0.25',' 0 0.125 0.125')
        if material=='gramado_simples':
            geometry=geometry.replace(' 0 0.125 0.125',' 0 0.03125 0.03125')
        labels={'saida':(1,1,'estrutura_clara'), 'placa_setor':(1,-1,'aco_escuro'),
                'quadro_eletrico':(1,1,'metal_galvanizado'), 'caixa_incendio':(1,1,'vermelho')}
        if material in labels:
            face_axis,face_sign,edge=labels[material]
            width,height=SURFACES['indoor/'+material].get('size',(512,512))
            lines=geometry.splitlines()
            for i in range(len(lines)-2):
                axis,sign=i//2,(-1 if i%2==0 else 1)
                if (axis,sign)!=(face_axis,face_sign):
                    lines[i+1]=lines[i+1].replace('indoor/'+material,'indoor/'+edge)
                    continue
                u_axis=1 if axis==0 else 0
                v_axis=1 if axis==2 else 2
                u_sign=sign if axis==0 else (-sign if axis==1 else 1)
                ulo,uhi=lo[u_axis]+bevel,hi[u_axis]-bevel
                vlo,vhi=lo[v_axis]+bevel,hi[v_axis]-bevel
                su=(uhi-ulo)*UNITS/width
                sv=(vhi-vlo)*UNITS/height
                u,v=[0,0,0],[0,0,0]
                u[u_axis],v[v_axis]=u_sign,-1
                ou=(-ulo if u_sign==1 else uhi)*UNITS/su
                ov=vhi*UNITS/sv
                prefix=lines[i+1].split(' indoor/')[0]
                lines[i+1]=f'{prefix} indoor/{material} [ {u[0]} {u[1]} {u[2]} {ou:.9g} ] [ {v[0]} {v[1]} {v[2]} {ov:.9g} ] 0 {su:.9g} {sv:.9g}'
            geometry='\n'.join(lines)
        elif material=='madeira_bancos':
            lines=geometry.splitlines()
            for i in (1,2): lines[i]=lines[i].replace('indoor/madeira_bancos','indoor/madeira_topo')
            geometry='\n'.join(lines)
        layers[layer].append(geometry)
        counts[layer] += 1

    def bar(layer, a, b, width, depth, material, offset=0, start_normal=None, end_normal=None):
        w = unit(sub(b,a))
        reference = (0,0,1) if abs(w[2]) < .95 else (0,1,0)
        u = unit(cross(reference,w))
        v = cross(w,u)
        center = add(mul(add(a,b),.5), mul(v,offset))
        length = math.sqrt(sum(c*c for c in sub(b,a)))
        planes=[]
        for axis,extent in [(w,length/2), (u,width/2), (v,depth/2)]:
            for sign in (-1,1): planes.append((mul(axis,sign),add(center,mul(axis,extent*sign))))
        # Cortes de encontro nos nos da trelica e na cumeeira, sem pontas sobrepostas.
        if start_normal is not None: planes[0]=(start_normal,a)
        if end_normal is not None: planes[1]=(end_normal,b)
        solid(layer,planes,material)

    def ibeam(layer,a,b,material,depth=.38,width=.22,end_normal=None):
        bar(layer,a,b,.032,depth-.064,material,end_normal=end_normal)
        for sign in (-1,1): bar(layer,a,b,width,.032,material,sign*(depth/2-.016),end_normal=end_normal)

    def round_post(layer,x,y,bottom,top,radius,material,sides=8):
        planes=[((0,0,-1),(x,y,bottom)),((0,0,1),(x,y,top))]
        for i in range(sides):
            a=2*math.pi*i/sides
            n=(math.cos(a),math.sin(a),0)
            planes.append((n,(x+n[0]*radius,y+n[1]*radius,0)))
        solid(layer,planes,material)

    def grille(a,b,bottom,top):
        length=math.dist(a,b)
        tangent=((b[0]-a[0])/length,(b[1]-a[1])/length)
        normal=(-tangent[1],tangent[0])
        def point(t,z,offset):
            return (a[0]+tangent[0]*t+normal[0]*offset,
                    a[1]+tangent[1]*t+normal[1]*offset,z)
        # Fios soldados em planos adjacentes: contato sem faces coincidentes no cruzamento.
        for i in range(1,math.ceil(length/.15)):
            if i*.15>length-.02: continue
            bar('09 Cerca e portao',point(i*.15,bottom,.003),point(i*.15,top,.003),.006,.006,'metal_galvanizado')
        for j in range(math.ceil((bottom+.004)/.15),math.floor((top-.004)/.15)+1):
            bar('09 Cerca e portao',point(0,j*.15,-.003),point(length,j*.15,-.003),.006,.006,'metal_galvanizado')

    def ring(x,y,bottom,top,outer,inner,material):
        for i in range(8):
            angle=i*math.pi/4
            n=(math.cos(angle),math.sin(angle),0)
            low,high=angle-math.pi/8,angle+math.pi/8
            planes=[((0,0,-1),(x,y,bottom)),((0,0,1),(x,y,top)),
                    (n,(x+n[0]*outer,y+n[1]*outer,0)),
                    (mul(n,-1),(x+n[0]*inner,y+n[1]*inner,0)),
                    ((math.sin(low),-math.cos(low),0),(x,y,0)),
                    ((-math.sin(high),math.cos(high),0),(x,y,0))]
            solid('07 Detalhes e instalacoes',planes,material)

    # O retangulo jogavel e reservado: 40x20, sem geometria nas zonas dos gols.
    block('01 Piso e gramado',(-24.8,-14.6,-.4),(24.8,14.6,-.035),'concreto_piso')
    block('01 Piso e gramado',(-22.5,-11.6,-.035),(22.5,11.6,0),'gramado_simples')
    # Acabamento na mesma cota do gramado, sem uma laje coplanar sob ele.
    for lo,hi in [((-24.8,-14.6,-.035),(-22.5,14.6,0)),
                  ((22.5,-14.6,-.035),(24.8,14.6,0)),
                  ((-22.5,-14.6,-.035),(22.5,-11.6,0)),
                  ((-22.5,11.6,-.035),(22.5,14.6,0))]:
        block('01 Piso e gramado',lo,hi,'concreto_piso')
    for y in (-11.69,11.69):
        block('01 Piso e gramado',(-22.5,y-.075,-.025),(22.5,y+.075,.002),'borracha')
        for i in range(150):
            x=-22.45+i*.3
            block('07 Detalhes e instalacoes',(x,y-.06,.001),(x+.014,y+.06,.008),'metal_galvanizado')

    # Casca fechada: quatro paredes e duas aguas, com empena sem vao para o exterior.
    for y in (-14.5,14.5):
        block('02 Paredes e cobertura',(-24.5,y-.12,0),(24.5,y+.12,8.52),'chapa_cinza')
    for x in (-24.65,24.65):
        block('02 Paredes e cobertura',(x-.15,-14.62,0),(x+.15,14.62,8.4),'chapa_cinza')
        for sign in (-1,1):
            solid('02 Paredes e cobertura',[
                ((-1,0,0),(x-.15,0,0)), ((1,0,0),(x+.15,0,0)),
                ((0,-sign,0),(0,0,0)), ((0,sign,0),(0,sign*14.62,0)),
                ((0,0,-1),(0,0,8.4)), ((0,sign*1.3/14.6,1),(0,0,9.8))], 'chapa_cinza')
    for sign in (-1,1):
        bar('02 Paredes e cobertura',(0,sign*14.65,8.5),(0,0,9.8),49.65,.16,'telha_escura',end_normal=(0,-sign,0))
        block('02 Paredes e cobertura',(-24.85,sign*14.65-.08,8.38),(24.85,sign*14.65+.08,8.62),'aco_escuro')
    block('02 Paredes e cobertura',(-24.85,-.12,9.72),(24.85,.12,9.89),'aco_escuro')

    def roof_height(y): return 9.8-abs(y)*1.3/14.6

    # Porticos, chapas de ligacao e trelicas reais sob a cobertura.
    for x in range(-24,25,6):
        for sign in (-1,1):
            y=sign*14.15
            ibeam('03 Estrutura metalica',(x,y,.1),(x,y,8.35),'estrutura_clara',.4,.28)
            block('03 Estrutura metalica',(x-.27,y-.27,0),(x+.27,y+.27,.1),'metal_galvanizado')
            for dx in (-.2,.2):
                for dy in (-.2,.2):
                    block('07 Detalhes e instalacoes',(x+dx-.024,y+dy-.024,.1),(x+dx+.024,y+dy+.024,.135),'aco_escuro')
            ibeam('03 Estrutura metalica',(x,y,8.3),(x,0,9.58),'aco_escuro',.3,.16,end_normal=(0,-sign,0))
            block('03 Estrutura metalica',(x-.24,y-.17,7.85),(x+.24,y+.17,8.45),'aco_escuro')
            for dz in (7.98,8.2,8.38):
                block('07 Detalhes e instalacoes',(x-.03,y-.225,dz-.025),(x+.03,y+.225,dz+.025),'metal_galvanizado')
        bar('03 Estrutura metalica',(x,-14.1,7.8),(x,14.1,7.8),.13,.16,'aco_escuro')
        for i in range(8):
            y0=-14.1+i*3.525
            y1=y0+3.525
            a=(x,y0,7.82 if i%2==0 else roof_height(y0)-.3)
            b=(x,y1,roof_height(y1)-.3 if i%2==0 else 7.82)
            bar('03 Estrutura metalica',a,b,.055,.065,'aco_escuro',start_normal=(0,-1,0),end_normal=(0,1,0))
    for y in (-13,-10,-7,-4,-1,1,4,7,10,13):
        bar('03 Estrutura metalica',(-24.6,y,roof_height(y)-.23),(24.6,y,roof_height(y)-.23),.09,.14,'aco_escuro')
    for sign in (-1,1):
        y=sign*14.24
        for z in (1.55,3.15,4.75,6.35,7.9):
            bar('03 Estrutura metalica',(-24.5,y,z),(24.5,y,z),.07,.07,'estrutura_clara')
        for left in (-24,12):
            bar('03 Estrutura metalica',(left,y,3.7),(left+6,y,7.8),.09,.09,'estrutura_clara')
            bar('03 Estrutura metalica',(left,y+sign*.10,7.8),(left+6,y+sign*.10,3.7),.09,.09,'estrutura_clara')
    for x in (-24.4,24.4):
        for y in (-9.5,0,9.5):
            ibeam('03 Estrutura metalica',(x,y,.1),(x,y,roof_height(y)-.25),'estrutura_clara',.28,.2)
            block('03 Estrutura metalica',(x-.19,y-.19,0),(x+.19,y+.19,.1),'metal_galvanizado')
        for z in (1.6,3.2,4.8,6.4,7.9):
            bar('03 Estrutura metalica',(x,-14.205,z),(x,14.205,z),.07,.07,'estrutura_clara')

    # Protecao macia da parede; o fechamento do campo e uma cerca independente.
    for sign in (-1,1):
        y=sign*14.27
        spans=[(-24.28,24.28)] if sign==1 else [(-24.28,-21.3),(-18.7,14.7),(17.3,24.28)]
        for lo,hi in spans:
            block('04 Protecoes de parede',(lo,y-.11,0),(hi,y+.11,1.35),'protecao_verde')
        for i in range(49):
            x=-24.4+i
            if not any(lo<=x<=hi-.02 for lo,hi in spans): continue
            block('04 Protecoes de parede',(x,y-.115,.03),(x+.018,y+.115,1.32),'verde_pintado')
    for sign in (-1,1):
        x=sign*24.39
        block('04 Protecoes de parede',(x-.11,-14.38,0),(x+.11,14.38,1.35),'protecao_verde')

    # Cerca antes das canaletas; deixa livres as areas de escape dos futuros gols.
    hx,hy=FENCE_HALF
    posts=set()
    def fence_run(a,b,bottom=.12):
        count=math.ceil(math.dist(a,b)/2.5)
        points=[tuple(round(a[k]+(b[k]-a[k])*i/count,6) for k in range(2)) for i in range(count+1)]
        posts.update(points)
        for a,b in zip(points,points[1:]):
            length=math.dist(a,b)
            t=((b[0]-a[0])/length,(b[1]-a[1])/length)
            inside_a=tuple(a[k]+t[k]*.03 for k in range(2))
            inside_b=tuple(b[k]-t[k]*.03 for k in range(2))
            for z in (bottom,2.7,FENCE_HEIGHT):
                bar('09 Cerca e portao',(*inside_a,z),(*inside_b,z),.03,.03,'metal_galvanizado')
            grille(inside_a,inside_b,bottom+.015,FENCE_HEIGHT-.015)
    fence_run((-hx,-hy),(hx,-hy))
    fence_run((-hx,hy),(GATE_X[0],hy))
    fence_run((GATE_X[1],hy),(hx,hy))
    fence_run((-hx,-hy),(-hx,hy))
    fence_run((hx,-hy),(hx,hy))
    fence_run((GATE_X[0],hy),(GATE_X[1],hy),bottom=2.3)
    for x,y in sorted(posts):
        block('09 Cerca e portao',(x-.08,y-.08,0),(x+.08,y+.08,.05),'metal_galvanizado')
        block('09 Cerca e portao',(x-.03,y-.03,.05),(x+.03,y+.03,5.45),'metal_galvanizado')
        block('09 Cerca e portao',(x-.035,y-.035,5.45),(x+.035,y+.035,5.48),'aco_escuro')
        for dx in (-.055,.055):
            for dy in (-.055,.055):
                block('09 Cerca e portao',(x+dx-.008,y+dy-.008,.05),(x+dx+.008,y+dy+.008,.065),'aco_escuro')

    # Folha fechada por pedido do usuario, com folgas e trinco no batente.
    a=(GATE_X[0]+.06,hy)
    b=(GATE_X[1]-.06,hy)
    for p in (a,b):
        block('09 Cerca e portao',(p[0]-.02,p[1]-.02,.1),(p[0]+.02,p[1]+.02,2.22),'metal_galvanizado')
    for z in (.12,1.1,2.2):
        bar('09 Cerca e portao',(a[0]+.02,hy,z),(b[0]-.02,hy,z),.04,.04,'metal_galvanizado')
    grille((a[0]+.02,hy),(b[0]-.02,hy),.14,2.18)
    bar('09 Cerca e portao',(a[0],hy-.025,.15),(b[0],hy-.025,2.17),.02,.02,'aco_escuro')
    for z in (.3,1.15,2.05):
        block('09 Cerca e portao',(GATE_X[0]-.025,hy-.035,z-.022),(a[0]+.01,hy+.035,z+.022),'aco_escuro')
        round_post('09 Cerca e portao',GATE_X[0]+.025,hy+.042,z-.05,z+.05,.014,'metal_galvanizado')
    block('09 Cerca e portao',(b[0]-.045,hy-.045,1.02),(b[0]+.015,hy+.045,1.16),'aco_escuro')
    block('09 Cerca e portao',(GATE_X[1]-.038,hy-.035,1.04),(GATE_X[1]-.026,hy+.035,1.15),'aco_escuro')
    bar('09 Cerca e portao',(b[0]+.01,hy+.03,1.075),(GATE_X[1]-.025,hy+.03,1.075),.018,.018,'metal_galvanizado')
    block('09 Cerca e portao',(b[0]-.03,hy+.035,1.055),(b[0]-.005,hy+.09,1.085),'metal_galvanizado',.003)

    # Abrigo dos bancos, na lateral norte; pilares, duas alturas de assento e ripas.
    for x in (-13,-9,-5,-1,3,7):
        block('05 Abrigo e bancos',(x-.13,11.82,0),(x+.13,12.09,.045),'metal_galvanizado',.003)
        block('05 Abrigo e bancos',(x-.075,11.88,.045),(x+.075,12.03,3.1),'verde_pintado')
        for dx in (-.095,.095):
            for yy in (11.84,12.07):
                block('05 Abrigo e bancos',(x+dx-.008,yy-.008,.043),(x+dx+.008,yy+.008,.058),'aco_escuro',.002)
        bar('05 Abrigo e bancos',(x,12,2.3),(x,13.1,3.2),.07,.07,'estrutura_clara')
    bar('05 Abrigo e bancos',(-13.45,12,3.12),(7.45,12,3.12),.14,.14,'verde_pintado')
    bar('05 Abrigo e bancos',(-3,11.65,3.04),(-3,14.25,3.36),21.0,.08,'telha_verde')
    bar('05 Abrigo e bancos',(-13.53,11.65,3.04),(7.53,11.65,3.04),.035,.10,'verde_pintado')
    for x in (-13.5,7.5):
        bar('05 Abrigo e bancos',(x,11.64,3.04-.01*.32/2.6),(x,14.26,3.36+.01*.32/2.6),.035,.10,'verde_pintado')
    for x in [i*.5-13.5 for i in range(43)]:
        bar('05 Abrigo e bancos',(x,11.66,3.085),(x,14.25,3.405),.018,.024,'verde_pintado')
    for y,height in ((12.58,.46),(13.46,.85)):
        joints=[-12.65]+list(range(-10,5,2))+[6.65]
        for slat in range(3):
            yy=y-.21+slat*.16
            for lo,hi in zip(joints,joints[1:]):
                block('05 Abrigo e bancos',(lo+.004,yy,height),(hi-.004,yy+.14,height+.055),'madeira_bancos',.004)
            for x in range(-12,7,2):
                round_post('05 Abrigo e bancos',x,yy+.07,height+.053,height+.058,.01,'metal_galvanizado')
                round_post('05 Abrigo e bancos',x,yy+.07,height+.058,height+.063,.007,'aco_escuro',6)
        for slat in range(3):
            z=height+.19+slat*.14
            for lo,hi in zip(joints,joints[1:]):
                block('05 Abrigo e bancos',(lo+.004,y+.25,z),(hi-.004,y+.30,z+.105),'madeira_bancos',.003)
        for x in range(-12,7,2):
            bar('05 Abrigo e bancos',(x,y-.245,height-.025),(x,y+.30,height-.025),.09,.05,'aco_escuro')
            for yy in (y-.16,y+.2):
                back=yy+.11 if yy>y else yy+.05
                block('05 Abrigo e bancos',(x-.05,yy-.05,0),(x+.05,back,.012),'borracha',.002)
                block('05 Abrigo e bancos',(x-.035,yy-.035,.012),(x+.035,yy+.035,height),'aco_escuro')
            bar('05 Abrigo e bancos',(x,y+.27,.012),(x,y+.27,height+.63),.045,.045,'aco_escuro')
            for i in range(3):
                z=height+.2425+i*.14
                block('07 Detalhes e instalacoes',(x-.009,y+.242,z-.009),(x+.009,y+.253,z+.009),'metal_galvanizado',.002)
    # Entradas fechadas, portao de servico, placas e ferragens.
    for x in (-20,16):
        y=-14.22
        block('06 Acessos e sinalizacao',(x-1.15,-14.4,0),(x+1.15,y+.14,2.7),'estrutura_clara',.006)
        block('06 Acessos e sinalizacao',(x-.003,-14.085,.06),(x+.003,-14.012,2.5),'borracha')
        for sign in (-1,1):
            xx=x+sign*.5
            block('06 Acessos e sinalizacao',(xx-.497,y+.14,.06),(xx+.497,y+.21,2.5),'porta_areia',.003)
            block('06 Acessos e sinalizacao',(xx-.49,-14.078,.02),(xx+.49,-14.005,.065),'borracha')
            block('06 Acessos e sinalizacao',(xx-.44,-14.014,.16),(xx+.44,-14.004,.34),'metal_galvanizado',.002)
            for dx in (-.27,.27):
                block('06 Acessos e sinalizacao',(xx+dx-.04,-14.016,.99),(xx+dx+.04,-13.95,1.065),'aco_escuro',.004)
            bar('06 Acessos e sinalizacao',(xx-.34,-13.95,1.03),(xx+.34,-13.95,1.03),.035,.025,'metal_galvanizado')
            block('06 Acessos e sinalizacao',(xx-.18,-14.01,1.5),(xx+.18,-13.985,2.12),'vidro_fume')
            for dz in (1.475,2.12):
                block('06 Acessos e sinalizacao',(xx-.205,-14.018,dz),(xx+.205,-13.975,dz+.025),'metal_galvanizado')
            for dx in (-.205,.18):
                block('06 Acessos e sinalizacao',(xx+dx,-14.018,1.5),(xx+dx+.025,-13.975,2.12),'metal_galvanizado')
            edge=xx+sign*.48
            for z in (.35,1.25,2.25):
                block('06 Acessos e sinalizacao',(edge-.025,-14.085,z-.03),(edge+.025,-13.998,z+.03),'aco_escuro')
                round_post('06 Acessos e sinalizacao',edge,-14.005,z-.055,z+.055,.016,'metal_galvanizado')
        block('06 Acessos e sinalizacao',(x-.33,-14.4,2.765),(x+.33,-14.32,3.075),'estrutura_clara',.004)
        block('06 Acessos e sinalizacao',(x-.30,-14.325,2.795),(x+.30,-14.31,3.045),'saida')
        for dx in (-.314,.314):
            for z in (2.782,3.058):
                block('06 Acessos e sinalizacao',(x+dx-.004,-14.323,z-.004),(x+dx+.004,-14.306,z+.004),'metal_galvanizado',.001)
        block('06 Acessos e sinalizacao',(x-1.1,-14.05,.0),(x+1.1,-12.65,.015),'borracha')
        block('06 Acessos e sinalizacao',(x+1.26,-14.4,1.5),(x+1.74,-14.18,2.2),'vermelho',.006)
        block('06 Acessos e sinalizacao',(x+1.29,-14.185,1.535),(x+1.71,-14.15,2.165),'caixa_incendio')
        for z in (1.66,2.02):
            block('06 Acessos e sinalizacao',(x+1.263,-14.184,z-.035),(x+1.290,-14.13,z+.035),'metal_galvanizado',.002)
        block('06 Acessos e sinalizacao',(x+1.67,-14.153,1.79),(x+1.70,-14.11,1.91),'metal_galvanizado',.003)
        block('06 Acessos e sinalizacao',(x+1.681,-14.112,1.81),(x+1.689,-14.107,1.83),'aco_escuro')
    block('06 Acessos e sinalizacao',(24.28,-12.4,1.4),(24.52,-5.2,5.7),'chapa_clara')
    block('06 Acessos e sinalizacao',(24.15,-12.25,.03),(24.28,-5.35,5.3),'metal_galvanizado')
    for y in (-12.3,-5.3):
        block('06 Acessos e sinalizacao',(24.05,y-.07,.05),(24.30,y+.07,5.45),'estrutura_clara')
        block('06 Acessos e sinalizacao',(24.02,y-.075,.05),(24.05,y+.075,1.25),'seguranca')
    for i in range(24):
        z=.18+i*.21
        block('06 Acessos e sinalizacao',(24.12,-12.2,z),(24.16,-5.4,z+.025),'aco_escuro')
    for y in (-8.9,8.9):
        block('06 Acessos e sinalizacao',(24.13,y-.6,6.0),(24.52,y+.6,6.8),'aco_escuro')
        for i in range(8):
            z=6.05+i*.09
            block('06 Acessos e sinalizacao',(24.05,y-.55,z),(24.14,y+.55,z+.035),'metal_galvanizado')
    for x in (-16,10):
        block('06 Acessos e sinalizacao',(x-.45,14.36,2.25),(x+.45,14.4,3.15),'placa_setor',.004)

    # Calhas, descidas, conduites e quadro; tudo fora dos 40x20 de jogo.
    for sign in (-1,1):
        y=sign*13.95
        block('07 Detalhes e instalacoes',(-24.5,y-.11,7.62),(24.5,y+.11,7.64),'metal_galvanizado')
        for yy in (y-.11,y+.095):
            block('07 Detalhes e instalacoes',(-24.475,yy,7.64),(24.475,yy+.015,7.80),'metal_galvanizado')
        for xx in (-24.5,24.475):
            block('07 Detalhes e instalacoes',(xx,y-.11,7.64),(xx+.025,y+.11,7.80),'metal_galvanizado')
        for x in (-23,21):
            round_post('07 Detalhes e instalacoes',x,y,-.02,7.63,.06,'estrutura_clara',12)
            for z in (1.1,3.6,6.1):
                round_post('07 Detalhes e instalacoes',x,y,z,z+.12,.069,'estrutura_clara',12)
            for z in (1,3,5,7):
                round_post('07 Detalhes e instalacoes',x,y,z,z+.045,.065,'aco_escuro',12)
                back=sign*14.4
                block('07 Detalhes e instalacoes',(x-.04,min(y,back),z+.01),(x+.04,max(y,back),z+.035),'metal_galvanizado')
        bar('07 Detalhes e instalacoes',(-24.4,sign*14.12,3.55),(24.4,sign*14.12,3.55),.028,.028,'metal_galvanizado')
    block('07 Detalhes e instalacoes',(21.3,-14.4,1.55),(22.0,-14.18,2.5),'quadro_eletrico',.006)
    for z in (1.75,2.3):
        block('07 Detalhes e instalacoes',(21.295,-14.19,z-.03),(21.335,-14.16,z+.03),'aco_escuro',.003)
    block('07 Detalhes e instalacoes',(21.88,-14.185,1.78),(21.93,-14.135,1.90),'aco_escuro',.004)
    bar('07 Detalhes e instalacoes',(21.65,-14.25,2.45),(21.65,-14.25,3.5375),.025,.025,'metal_galvanizado')
    bar('07 Detalhes e instalacoes',(21.65,-14.2625,3.55),(21.65,-14.12,3.55),.025,.025,'metal_galvanizado')
    for z in (2.8,3.25):
        block('07 Detalhes e instalacoes',(21.62,-14.39,z-.035),(21.68,-14.37,z+.035),'aco_escuro')
        block('07 Detalhes e instalacoes',(21.64,-14.38,z-.012),(21.66,-14.24,z+.012),'metal_galvanizado')
    for y in (-8,0,8):
        for dy in (-.13,.13):
            bar('07 Detalhes e instalacoes',(-24,y+dy,7.62),(24,y+dy,7.62),.025,.06,'metal_galvanizado')
        for x in range(-24,25,2):
            bar('07 Detalhes e instalacoes',(x,y-.13,7.6),(x,y+.13,7.6),.025,.025,'metal_galvanizado')
        for x in range(-24,25,6):
            for dy in (-.13,.13):
                bar('07 Detalhes e instalacoes',(x,y+dy,7.62),(x,y+dy,7.8),.012,.012,'aco_escuro')
    for x in (-16,8.8):
        round_post('07 Detalhes e instalacoes',x,13.0,0,.035,.22,'borracha')
        ring(x,13,.035,.75,.22,.19,'verde_pintado')
        ring(x,13,.75,.79,.24,.14,'borracha')
        for sign in (-1,1):
            xx=x+sign*.26
            bar('07 Detalhes e instalacoes',(xx,13,.57),(xx,13,.69),.018,.018,'metal_galvanizado')
            for z in (.58,.68):
                bar('07 Detalhes e instalacoes',(x+sign*.21,13,z),(xx,13,z),.016,.016,'metal_galvanizado')

    lights=[]
    for x in (-20,-12,-4,4,12,20):
        for y in (-9,-3,3,9):
            z=7.45
            block('08 Luminarias',(x-.78,y-.24,z),(x+.78,y+.24,z+.15),'metal_galvanizado')
            block('08 Luminarias',(x-.68,y-.18,z-.035),(x+.68,y+.18,z),'luz_led')
            for dx in (-.765,.765):
                block('08 Luminarias',(x+dx-.025,y-.245,z+.01),(x+dx+.025,y+.245,z+.14),'aco_escuro',.002)
            for dx in (-.58,.58):
                for dy in (-.20,.20):
                    block('08 Luminarias',(x+dx-.015,y+dy-.025,z-.02),(x+dx+.015,y+dy+.025,z+.05),'metal_galvanizado')
            for dx in (-.52,.52):
                bar('08 Luminarias',(x+dx,y,z+.15),(x+dx,y,roof_height(y)),.009,.009,'aco_escuro')
            lights.append({'classname':'light','origin':f'{x*32} {y*32} {(z-.12)*32}',
                           '_color':'1 0.96 0.89','intensity':'900','range':'20',
                           'type':'spot','pitch':'-90','inner_cone':'90','outer_cone':'120'})
    for x in (-10,-2,6):
        block('08 Luminarias',(x-.48,13.78,2.93),(x+.48,13.91,2.99),'luz_led')
        block('08 Luminarias',(x-.50,13.76,2.99),(x+.50,13.93,3.04),'metal_galvanizado',.003)
        for dx in (-.32,.32):
            bar('08 Luminarias',(x+dx,13.84,2.99),(x+dx,13.84,3.31),.012,.012,'aco_escuro')
        lights.append({'classname':'light','origin':f'{x*32} {13.84*32} {2.90*32}',
                       '_color':'.83 .92 1','intensity':'60','range':'5',
                       'type':'spot','angle':'-90','pitch':'-75','inner_cone':'70','outer_cone':'110'})

    world='// Game: Soccer Fall\n// Format: Valve\n// 32 unidades = 1 metro. Fonte gerada por tools/maps/galpao_01.py.\n'
    world+='{"classname" "worldspawn" "message" "Galpao 01 - quadra coberta"}\n'
    for index,(name,brushes) in enumerate(sorted(layers.items()),1):
        world+='\n{\n"classname" "func_group"\n"_tb_type" "_tb_layer"\n'
        world+=f'"_tb_name" "{name}"\n"_tb_id" "{index}"\n"_tb_layer_sort_index" "{index}"\n'
        world+='\n'.join(brushes)+'\n}\n'
    points=[{'classname':'info_player_start','origin':'0 0 8','angle':'0'},
            {'classname':'info_player_start','origin':'-512 -240 8','angle':'0'},
            {'classname':'info_player_start','origin':'512 240 8','angle':'180'},
            {'classname':'matter_field','origin':'0 0 0','angle':'0','length_m':'40','width_m':'20'}] + lights
    for props in points:
        world+='\n{\n'+'\n'.join(json.dumps(k)+' '+json.dumps(v) for k,v in props.items())+'\n}\n'
    return world,dict(counts)


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--substituir',action='store_true',help='Regerar a fonte: descarta edicoes manuais deste mapa')
    args=parser.parse_args()
    path=ASSETS/'src'/f'{NAME}.map'
    if path.exists() and not args.substituir: parser.exit(1,'Mapa ja existe; use --substituir apenas para regenerar a fonte conhecida.\n')
    generate()
    source,counts=build()
    path.write_text(source)
    print(json.dumps({'map':str(path),'layers':counts,'brushes':sum(counts.values())},ensure_ascii=False))
