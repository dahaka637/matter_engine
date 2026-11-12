"""Materiais procedurais do galpao: os IDs fisicos continuam sendo os da engine."""
from pathlib import Path
import math
import random


SURFACES = {
    'indoor/chapa_cinza': dict(physical='steel', color=(87, 91, 93), roughness=.76, metallic=.35, pattern='sheet'),
    'indoor/chapa_clara': dict(physical='steel', color=(177, 181, 169), roughness=.78, metallic=.15, pattern='sheet'),
    'indoor/telha_escura': dict(physical='steel', color=(38, 46, 43), roughness=.82, metallic=.35, pattern='roof'),
    'indoor/aco_escuro': dict(physical='steel', color=(47, 53, 56), roughness=.58, metallic=.45, pattern='paint'),
    'indoor/estrutura_clara': dict(physical='steel', color=(194, 195, 183), roughness=.68, metallic=.15, pattern='paint'),
    'indoor/protecao_verde': dict(physical='rubber', color=(23, 88, 62), roughness=.88, metallic=0, pattern='pad'),
    'indoor/verde_pintado': dict(physical='steel', color=(26, 104, 68), roughness=.65, metallic=.1, pattern='paint'),
    'indoor/telha_verde': dict(physical='steel', color=(27, 103, 64), roughness=.7, metallic=.2, pattern='roof'),
    'indoor/madeira_bancos': dict(physical='wood', color=(140, 99, 57), roughness=.56, metallic=0, pattern='wood'),
    'indoor/concreto_piso': dict(physical='concrete', color=(112, 115, 112), roughness=.94, metallic=0, pattern='concrete'),
    'indoor/borracha': dict(physical='rubber', color=(35, 43, 39), roughness=.9, metallic=0, pattern='rubber'),
    'indoor/rede_verde': dict(physical='steel', color=(32, 71, 52), roughness=.95, metallic=0, pattern='plain'),
    'indoor/gramado_simples': dict(physical='grass', color=(49, 106, 49), roughness=1, metallic=0, pattern='plain',
                                  normal='indoor/gramado_normal', mr='indoor/gramado_mr', parallax=.004, size=(1024,1024)),
    'indoor/porta_areia': dict(physical='steel', color=(165, 158, 133), roughness=.6, metallic=.15, pattern='paint'),
    'indoor/metal_galvanizado': dict(physical='steel', color=(166, 171, 174), roughness=.46, metallic=.7, pattern='zinc'),
    'indoor/seguranca': dict(physical='rubber', color=(197, 157, 39), roughness=.9, metallic=0, pattern='hazard'),
    'indoor/vermelho': dict(physical='steel', color=(166, 40, 29), roughness=.68, metallic=.1, pattern='paint'),
    'indoor/luz_led': dict(physical='steel', color=(228, 237, 229), roughness=.4, metallic=0,
                           pattern='diffuser', emissive=(.86, .94, 1), strength=5),
    'indoor/saida': dict(physical='steel', color=(22, 101, 60), roughness=.8, metallic=0,
                        pattern='exit', size=(768,320), emissive=(.15, .38, .18), strength=1.5),
    'indoor/quadro_eletrico': dict(physical='steel', color=(137, 145, 145), roughness=.6, metallic=.3, pattern='cabinet', size=(512,704)),
    'indoor/placa_setor': dict(physical='steel', color=(37, 70, 54), roughness=.9, metallic=0, pattern='sector'),
    'indoor/caixa_incendio': dict(physical='steel', color=(171, 38, 34), roughness=.58, metallic=.15, pattern='fire', size=(512,768)),
    'indoor/madeira_topo': dict(physical='wood', color=(137, 98, 57), roughness=.65, metallic=0, pattern='endgrain'),
    'indoor/vidro_fume': dict(physical='glass', color=(36, 51, 57), roughness=.22, metallic=.05, pattern='glass'),
}

SIZE = 512


def generate():
    from PIL import Image, ImageDraw, ImageFont
    out = Path(__file__).resolve().parents[2] / 'assets/maps/textures/indoor'
    out.mkdir(parents=True, exist_ok=True)
    font_path = str(Path(__file__).resolve().parents[2] / 'assets/fonts/roboto/Roboto-Medium.ttf')
    for index, (name, spec) in enumerate(SURFACES.items()):
        if name == 'indoor/gramado_simples':
            from gramado import generate as generate_grass
            generate_grass()
            continue
        base, pattern = spec['color'], spec['pattern']
        rng = random.Random(731 + index)
        w,h=spec.get('size',(SIZE,SIZE))
        im = Image.new('RGB', (w,h), base)
        if pattern != 'plain':
            pixels=[]
            for y in range(h):
                for x in range(w):
                    tx,ty=2*math.pi*x/w,2*math.pi*y/h
                    variation=1.5*math.sin(3*tx)*math.sin(2*ty)+rng.uniform(-2,2)
                    if pattern=='wood':
                        warp=1.8*math.sin(2*tx)+.6*math.sin(7*tx)
                        variation+=7*math.sin(30*ty+warp)+2*math.sin(90*ty+warp)
                    elif pattern=='endgrain':
                        radius=math.hypot(x-w*.4,(y-h*.55)*1.4)
                        variation+=6*math.sin(radius*.45)+3*math.sin(radius*.13)
                    elif pattern=='concrete': variation+=rng.uniform(-4,4)
                    elif pattern=='zinc': variation+=2*math.sin(36*tx+8*math.sin(11*ty))
                    pixels.append(tuple(max(0,min(255,round(c+variation))) for c in base))
            im.putdata(pixels)
        draw = ImageDraw.Draw(im)
        def shade(delta): return tuple(max(0, min(255, c+delta)) for c in base)
        if pattern in ('sheet', 'roof'):
            for x in range(0,w,32):
                draw.rectangle((x,0,x+4,h),fill=shade(-16))
                draw.line((x+5,0,x+5,h),fill=shade(22),width=3)
                draw.line((x+9,0,x+9,h),fill=shade(6),width=2)
            for y in (16,272):
                for x in range(16,w,64):
                    draw.ellipse((x-2,y-2,x+2,y+2),fill=shade(-28))
                    draw.point((x,y-2),fill=shade(35))
        elif pattern == 'pad':
            for x in range(0,w,128):
                draw.line((x,0,x,h),fill=shade(-14),width=3)
                draw.line((x+5,0,x+5,h),fill=shade(9),width=2)
                for y in range(4,h,14):draw.line((x+9,y,x+9,y+5),fill=shade(-7),width=1)
        elif pattern == 'concrete':
            for _ in range(2200):
                x,y=rng.randrange(w),rng.randrange(h)
                draw.point((x,y),fill=shade(rng.randrange(-16,17)))
            draw.line((0,0,w,0),fill=shade(-14),width=2)
            draw.line((0,0,0,h),fill=shade(-14),width=2)
            draw.line((3,3,w,3),fill=shade(5))
            draw.line((3,3,3,h),fill=shade(5))
        elif pattern == 'rubber':
            for x in range(0,w,24):draw.line((x,0,x,h),fill=shade(-6),width=3)
        elif pattern == 'hazard':
            for x in range(-w,w*2,128):
                draw.polygon([(x,0),(x+64,0),(x-448,h),(x-512,h)],fill=(30,35,30))
        elif pattern == 'diffuser':
            for x in range(0,w,16):draw.line((x,0,x,h),fill=shade(-6),width=2)
        elif pattern in ('exit','sector','cabinet','fire'):
            ink=(233,239,228)
            def label(text,xy,size,fill=ink):
                draw.text(xy,text,font=ImageFont.truetype(font_path,size),anchor='mm',fill=fill)
            draw.rounded_rectangle((10,10,w-11,h-11),radius=10,outline=shade(32),width=3)
            if pattern=='exit':
                draw.polygon([(93,75),(143,75),(143,166),(179,166),(118,235),(57,166),(93,166)],fill=ink)
                label('SAÍDA',(465,157),118)
            elif pattern=='sector':
                label('QUADRA',(256,80),48)
                label('01',(256,245),205)
                draw.line((80,386,432,386),fill=shade(35),width=2)
                label('SOCCER FALL',(256,434),34)
            elif pattern=='fire':
                label('INCÊNDIO',(256,80),64)
                draw.line((55,138,457,138),fill=ink,width=3)
                draw.ellipse((150,220,362,432),outline=ink,width=15)
                draw.ellipse((191,261,321,391),outline=ink,width=13)
                draw.line([(256,326),(256,467),(382,467),(382,379)],fill=ink,width=15)
                draw.rectangle((370,361,394,391),fill=ink)
                label('HIDRANTE',(256,560),44)
                label('MANTENHA LIVRE',(256,680),28)
            else:
                label('Q.E. 01',(256,55),42,shade(-65))
                for y in range(110,280,22):
                    draw.rounded_rectangle((65,y,447,y+6),radius=2,fill=shade(-37))
                    draw.line((65,y+7,447,y+7),fill=shade(20))
                label('ILUMINAÇÃO',(256,360),36,shade(-60))
                draw.polygon([(135,455),(209,591),(61,591)],fill=(217,177,55))
                draw.line([(143,482),(116,537),(151,531),(123,571)],fill=(39,42,41),width=8)
                label('220 V',(135,631),28,shade(-60))
        im.save(out / (name.split('/')[1] + '.png'), optimize=True)
    return len(SURFACES)


if __name__ == '__main__': print(f'{generate()} texturas do galpao geradas')
