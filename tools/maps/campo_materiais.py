"""Materiais da etapa 1, exclusivos da autoria; não altera compilador/runtime."""
import json
from pathlib import Path
import random

from PIL import Image, ImageDraw, ImageFilter

ROOT=Path(__file__).resolve().parents[2]
ASSETS=ROOT/'assets/maps'
SURFACES={
    'campo_verde':((78,115,54),'grass',.99,'turf',4),
    'reboco_menta':((184,217,193),'concrete',.94,'stucco',2),
    'rodape_verde':((57,120,89),'concrete',.94,'stucco',2),
    'concreto':((157,160,149),'concrete',.97,'concrete',2),
    'concreto_assento':((161,166,157),'concrete',.96,'concrete',2),
    'concreto_estrutura':((135,140,130),'concrete',.98,'concrete',2),
    'alvenaria_cinza':((135,139,126),'concrete',.99,'masonry',2),
    'mureta_antiga':((185,188,168),'concrete',.98,'masonry',2),
    'poste_branco':((211,213,195),'concrete',.96,'paint',2),
    'metal_zincado':((123,136,128),'steel',.66,'zinc',2),
    'aco_escuro':((49,65,58),'steel',.75,'paint',2),
    'porta_verde':((48,101,75),'wood',.88,'paint',2),
    'vidro':((54,73,65),'glass',.30,'glass',2),
    'fibrocimento':((144,147,132),'concrete',.97,'roof',2),
    'amarelo':((207,174,63),'concrete',.93,'paint',2),
}


def specs():
    return {'beirario/'+key:dict(color=color,physical=physical,roughness=rough,
            metallic=.3 if physical=='steel' else 0,repeat_m=repeat)
            for key,(color,physical,rough,pattern,repeat) in SURFACES.items()}


def generate():
    out=ASSETS/'textures/beirario'
    out.mkdir(parents=True,exist_ok=True)
    for index,(name,(base,physical,rough,pattern,repeat)) in enumerate(SURFACES.items()):
        rng=random.Random(220913+index)
        size=512
        # Ruído em duas escalas; variação suave no gramado, grão mais seco no concreto.
        low=Image.new('L',(32,32))
        low.putdata([rng.randrange(256) for _ in range(32*32)])
        low=low.resize((size,size),Image.Resampling.BICUBIC).filter(ImageFilter.GaussianBlur(4))
        lowpixels=list(low.getdata())
        pixels=[]
        for i in range(size*size):
            wave=(lowpixels[i]-128)/128
            noise=rng.uniform(-3,3)
            if pattern=='turf':
                delta=wave*9+noise*1.7
                pixels.append((int(base[0]+delta*.8),int(base[1]+delta),int(base[2]+delta*.55)))
            else:
                delta=wave*(7 if pattern in ('masonry','concrete') else 3)+noise
                pixels.append(tuple(max(0,min(255,round(c+delta))) for c in base))
        im=Image.new('RGB',(size,size));im.putdata(pixels)
        d=ImageDraw.Draw(im)
        def shade(amount):return tuple(max(0,min(255,c+amount)) for c in base)
        if pattern=='masonry':
            for row,y in enumerate(range(-32,544,40)):
                d.line((0,y,512,y),fill=shade(-25),width=2)
                d.line((0,y+3,512,y+3),fill=shade(12),width=1)
                for x in range(-140,650,140):
                    xx=x+70*(row%2)
                    d.line((xx,y,xx,y+40),fill=shade(-18),width=2)
            for _ in range(90):
                x,y=rng.randrange(size),rng.randrange(size)
                d.line((x,y,x+rng.randrange(3,17),y+rng.randrange(-1,2)),fill=shade(-rng.randrange(8,30)),width=1)
        if pattern=='roof':
            for y in range(0,size,22):
                d.line((0,y,512,y),fill=shade(-21),width=3)
                d.line((0,y+7,512,y+7),fill=shade(15),width=3)
        if pattern in ('paint','stucco','concrete'):
            for _ in range(160 if pattern=='concrete' else 60):
                x,y=rng.randrange(size),rng.randrange(size)
                r=rng.choice([1,1,2,3])
                d.ellipse((x,y,x+r,y+r),fill=shade(-rng.randrange(8,23)))
        im.save(out/(name+'.png'),optimize=True)
    target=ASSETS/'materials/beirario/superficies.json'
    target.parent.mkdir(parents=True,exist_ok=True)
    target.write_text(json.dumps(specs(),indent=2,ensure_ascii=False)+'\n')
    return len(SURFACES)


if __name__=='__main__':print(f'{generate()} materiais da etapa 1 gerados.')
