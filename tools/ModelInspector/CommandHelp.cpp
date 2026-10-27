#include "Commands.hpp"

#include <iostream>

namespace MatterEngine::ModelInspector {

int runHelp() {
    std::cout << R"(MatterModelInspector - ferramentas de modelagem do personagem (Claude/Codex)

uso: MatterModelInspector <comando> [opcoes]

COMANDOS
  info        esqueleto (juntas, regioes, medidas), malha, referencias
              --dedos (lista os 30 dedos)  --animacoes (ids para --pose)
  render      uma imagem
  folha       varias imagens numa grade: --vistas a,b  --regioes x,y  --modos m,n  --colunas N
  volta       volta completa em torno (yaw 0..360): --passos 8 --pitch 8
  analisar    checagens automaticas da malha (topologia, intersecoes, simetria,
              skin, ossos dentro do corpo, deformacao nas --poses a,b)
              --json <arq>  --imagens N (close-up de cada achado)  --max N
              --sem-intersecoes --sem-simetria --tolerancia-simetria <m>
  medir       medidas do esqueleto e cortes da malha ao longo dos ossos
              --junta <nome> --t <0..1> (um corte so)
  corte       o corte da malha por um plano (contorno de frente e no corpo)
              --junta <nome> [--t 0.5] | --altura <m do chao> | --plano frontal|sagital [--deslocamento m]
              --partes camisa,calcao (um contorno por parte)  --alvo x,y,z --raio m (enquadra)
              com --pose: a malha cortada na pose
  comparar    o modelo sobre as referencias (frente, costas, lado) na mesma escala:
              sobreposicao, diferenca de silhueta e larguras por altura
              --referencia <png> --vista <id> (uma imagem qualquer)
  poses       o modelo nas animacoes do jogo (folha) e a deformacao da malha
              --poses repouso,jog@0.25,sprint@0.3  --vista 34  --modo solido
  textura     o atlas de textura da malha (PNG): --ilhas (as arestas do mapa) --fundo
  molde       o molde da cabeca de referencia (referenceFit): o alinhamento e a distancia
              de cada vertice do queixo, da mandibula e do pescoco a referencia
  pintura     os moldes da pintura (pintura.json): cada espaco desenrolado numa imagem
              com a cor final, a grade de 5 cm e as ancoras - para desenhar uniformes
              --espaco tronco (padrao: todos)  --passo 3 (mm)  --escala 2000 (px/m)
              --lado esquerdo
  avesso      o avesso a vista em todas as animacoes do jogo (mao solta e punho):
              os triangulos virados e os pontos do weightRelax da montagem
              --instantes 6 (por animacao)  --raio 0.02 (m, de cada ponto)
  uniforme    o uniforme sobre o corpo (a montagem sem ele): a folga do pano ate a pele por
              regiao, as larguras do tronco, a linha do ombro e os cortes (uniforme.png)
  postura     a postura nas animacoes do jogo: em cada clipe, a inclinacao do tronco e do
              pescoco (da vertical e sobre o tronco), a cabeca a frente e o olhar
              --clipes a,b  --amostras 24 (folha de lado: postura.png)
  pesos       os pesos do skin dos vertices perto de um ponto (na ligacao): --perto x,y,z
              --raio 0.02 --parte calcao, ou --triangulos T2174 (os vertices dos triangulos que o "avesso" mostra)
  quadros     uma gravacao do jogo desenhada com o jogador de verdade (folha de quadros):
              --dump <arquivo> (MatterLocomotionProbe --dump, harness XPOSEDUMP, MatterAthleteProbe,
              MatterRLDynamics, MatterRLJointProbe, matter-rl folha)
              --cenario <nome|indice>  --instantes t0,t1 | --de A --ate B --quadros K
              --seguir 0|1  --distancia <m>  --colunas C  (bola amarela; vermelha no toque)
              --colisores 1 as shapes fisicas por cima (com BUILD_RL); contatos P em cruz vermelha
              o corpo do cenario pela linha "corpo" do dump (--corpo sobrepoe)
  colisores   o perfil fisico sobre a malha, em repouso ou num clipe (com BUILD_RL):
              --perfil <ragdoll.json> --pose sprint@0.3
              --poses repouso,sprint@0.3,jump_standing@0.35 (uma matriz)
              --vistas frente,direita,cima --modo raiox  --link LeftFoot (so aquele osso, de perto)
              azul/esquerda, laranja/direita, branco/centro, amarelo/COM,
              magenta/juntas, cruz cinza/origem do link
  exportar-jogo  o jogador para o jogo (o personagem do laboratorio): o perfil fisico
              medido no corpo novo (ancoras, links e colisores), a pele e o atlas
              --destino <pasta> (assets/characters/football_player troca o do jogo)
              --quinas 45 (graus)  --facetado 0.5 (o sombreamento do jogo)
  exportar-blender  o jogador para o Blender: jogador.troca.json (esqueleto, malha,
              partes, mapa e pesos) e as texturas; tools/blender/importar_troca.py
              monta o .blend   --saida <pasta>
              --poses N (o skin de cada junta em N instantes de cada clipe)
              --amplitude (e as poses da amplitude das juntas, no treino do assado)

OPCOES COMUNS
  --modelo <model.json>     padrao: assets/characters/player_model/model.json
  --malha montagem|procedural|nenhuma   montagem = o modelo novo (padrao do manifesto);
                            procedural = o corpo antigo (model_antigo.json); nenhuma = so os ossos
  --corpo <cm>,<kg>         o corpo (170 a 200 cm, 60 a 100 kg; padrao 180,80): a malha, o esqueleto
                            e o personagem do jogo nele (ex.: --corpo 200,95)
  --pose <animacao>@<s>     a cena numa pose (repouso, ligacao, sprint@0.3 ...)
                            ou dobra:RightShin=90+RightThigh=-60 (graus em volta do lado:
                            joelho + dobra, quadril - leva a coxa a frente;
                            x/z no fim: outro eixo; d: a dobradica da junta)
                            mao no fim: #reta #repouso (padrao) #aberta #punho
                            (ex.: ligacao#punho, sprint@0.3#aberta); numa dobra,
                            os dedos de partida (padrao reta; o corpo na ligacao):
                            dobra:RightThumb3=60d#punho; RightFist=60 fecha a mao
                            60% ate o punho (o "Dedos" da tela); eixos presos ao pai
  --saida <arquivo.png>     ou --saida-dir <pasta> (padrao ./inspecao)

DESENHO (render, folha, volta)
  --regiao <id>             visao_geral, cabeca, pescoco, peito, abdomen, quadril,
                            braco_*, antebraco_*, mao_*, coxa_*, canela_*, pe_*  (_direito/_esquerdo, _direita/_esquerda)
  --vista <id>              frente costas direita esquerda cima baixo 34 34e 34c
  --yaw <graus> --pitch <graus>   angulo livre (0 = de frente; +90 = lado esquerdo)
  --zoom <x>                aproxima (2 = metade da distancia)
  --alvo x,y,z --raio <m>   enquadra um ponto (espaco do modelo) num raio (detalhe de perto)
  --fov <graus>             angulo da lente (padrao 28; maior = perspectiva mais forte, mais perto)
  --orto                    projecao ortografica (sem perspectiva)
  --modo <id>               ossos solido solido_arame arame raiox normais partes pesos argila
  --junta <nome>            a junta do modo pesos
  --rotulos                 nomes das juntas      --isolar   so a regiao
  --largura/--altura <px>   --ssaa <1..4>
  --sem-grade --sem-hud --sem-ossos --sem-avesso
  --luz-frontal             a luz principal de frente (como os renders das referencias)
  --quinas <graus>          o sombreamento liso (modo solido): as faces que dobram mais que isso
                            ficam marcadas (padrao 45)
  --facetado <0..1>         o quanto volta as faces chapadas (0 liso, padrao; 1 facetado)

CONVENCOES
  espaco do modelo: z cima, x frente, y esquerda (metros); chao em z = -0,9776
  cores: osso direito laranja, esquerdo azul, centro branco; foco amarelo;
  avesso (normal invertida) magenta; achados: erro vermelho, aviso amarelo
)";
    return 0;
}

} // namespace MatterEngine::ModelInspector
