#pragma once

#include "Engine/Modeling/Assembly/EyeSocket.hpp"
#include "Engine/Modeling/Assembly/FacePaint.hpp"
#include "Engine/Modeling/Assembly/NeckShortening.hpp"
#include "Engine/Modeling/Assembly/ReferenceFit.hpp"
#include "Engine/Modeling/Assembly/ShoulderBand.hpp"
#include "Engine/Modeling/Geometry/ModelGeometry.hpp"
#include "Engine/Modeling/Mesh/MeshSculpt.hpp"
#include "Engine/Modeling/Raster/Color.hpp"
#include "Engine/Modeling/Skin/SkinSmoothing.hpp"

#include <optional>
#include <string>
#include <string_view>

// A montagem do jogador em numeros (montagem.json): as malhas base (o corpo
// e a cabeca, prontas, em glTF), onde cortar e costurar uma na outra e a
// chuteira nos pes. Alturas em metros do chao; x para a frente, y para a
// esquerda, z para cima. A montagem e da metade direita (o espelho completa).
namespace MatterEngine::Modeling {

struct AssemblyRecipe {
    // O corpo: o arquivo (relativo ao manifesto), para onde ele olha no
    // glTF e a altura (o chao na sola, o alto da cabeca original nela).
    std::string bodyFile = "base/corpo_base.glb";
    Vec3 bodyForward { 0.0f, 0.0f, 1.0f };
    float bodyHeight = 1.8f;
    // O corte do pescoco (o mesmo no corpo e na cabeca, no meio das duas
    // colunas do pescoco): a altura em x = 0 e a inclinacao (graus; a frente
    // mais baixa que a nuca). Os vertices a menos de "neckSnap" do plano vao
    // para ele (o corte segue as arestas que ja existem, sem lascas); acima
    // dele o pescoco da cabeca toma a forma do do corpo em "neckBlend"
    // metros; na costura, vertices a menos de "neckWeld" viram um so.
    float neckHeight = 1.548f;
    float neckTilt = 0.0f;
    float neckSnap = 0.009f;
    float neckBlend = 0.045f;
    float neckWeld = 0.006f;

    // A cabeca: o arquivo, para onde olha, a escala e onde fica o alto (do
    // chao); o pescoco dela vai para cima do do corpo (o centro alinhado) e
    // anda mais "headShift" (x para a frente).
    std::string headFile = "base/cabeca_base.glb";
    Vec3 headForward { 0.0f, 0.0f, 1.0f };
    float headScale = 0.555f;
    float headTop = 1.8f;
    float headShift = 0.0f;
    // A chuteira nos pes: a receita (relativa ao manifesto), a escala (sobre
    // o calcanhar) e o quanto a perna e cortada acima do ponto mais alto da
    // gola (a meia desce dali ate a gola).
    std::string bootFile = "partes/chuteira.json";
    float bootScale = 0.96f;
    float bootCollarGap = 0.02f;

    // O uniforme (ver Garment e Uniform): a camisa e o calcao feitos da pele,
    // com as alturas do chao. A camisa: a barra, o fim da manga (fracao do
    // ombro ao cotovelo), a gola (em volta do pescoco: o fundo do V na
    // frente, a altura onde o V chega ao lado do pescoco - "collarOpening"
    // graus da frente - e a de tras), a folga no peito, na barra, na manga e
    // na gola. O calcao: o alto (dentro da camisa), a barra da perna, a folga
    // nas pernas, no alto e na barra (a perna abre ate ela). O recuo (o quanto a pele entra na peca a partir
    // da barra) e como o pano sai da pele (as voltas de Taubin antes de
    // inflar, os passos da folga, os alisamentos por passo e a queda da
    // camisa e do calcao, ver ClothShape).
    bool uniform = true;
    float shirtHem = 0.95f;
    float sleeveAt = 0.55f;
    float collarDepth = 1.46f;
    float collarSide = 1.53f;
    float collarBack = 1.515f;
    float collarOpening = 75.0f;
    float shirtLoose = 0.012f;
    float shirtLooseHem = 0.022f;
    float sleeveLoose = 0.015f;
    float collarLoose = 0.01f;
    // A folga na axila (o lado de dentro da manga perto do ombro e o flanco
    // embaixo dele): maior que a da manga, o pano nao dobra quando o braco
    // sobe; 0 = a da manga.
    float armpitLoose = 0.0f;
    float shortsTop = 1.0f;
    float shortsHem = 0.66f;
    float shortsLoose = 0.018f;
    float shortsLooseTop = 0.006f;
    float shortsLooseHem = 0.028f;
    float recess = 0.02f;
    int clothRelax = 3;
    // O quanto a camisa do ombro para cima entra no Taubin: 1 arredonda como
    // o resto, 0 fica com a forma do corpo (o deltoide e o trapezio por baixo
    // do pano); a axila, abaixo, e a nuca sempre.
    float shoulderRelax = 1.0f;
    int clothSteps = 6;
    int clothSmoothing = 2;
    float clothHang = 0.25f;
    float shortsHang = 0.25f;
    // Voltas de espalhamento dos pesos da camisa (SkinSmoothing) na passagem
    // do braco para o tronco (a axila).
    int armpitWeightSmoothing = 3;
    // Os pesos dos dedos e do polegar espalhados pela superficie deles
    // (voltas; so nos vertices com peso neles, a palma em volta segura os
    // dela): a dobra de cada articulacao repartida por mais triangulos, a pele
    // de dentro das juntas nao vira do avesso quando a mao fecha ou o polegar
    // contrai.
    int fingerWeightSmoothing = 0;
    // Quanto dos pesos das coxas fica na camisa atras do quadril (o resto vai
    // para a pelve; 1 = os da pele): a barra de tras pende do tronco, a coxa
    // muito dobrada (a largada agachada) nao vira o pano do avesso; na frente
    // a barra sobe com a coxa.
    float shirtBackThighWeight = 1.0f;
    // Quanto do peso do pescoco fica na camisa (o resto vai para o alto do
    // peito; 1 = o da pele): a gola anda com o peito e o pescoco mexe por
    // dentro dela. O corte da gola herda peso do pescoco da pele de cima; com
    // ele, o pescoco estendido descia a gola de tras sobre a pele de dentro e
    // a borda virava do avesso.
    float shirtNeckWeight = 1.0f;
    // Os pesos espalhados em volta de pontos medidos (relaxWeights, na metade
    // direita; o espelho anda junto): o avesso que sobra nas poses do jogo,
    // corrigido no lugar (os pontos saem do "MatterModelInspector avesso").
    std::vector<WeightRelaxPoint> weightRelax;
    int weightRelaxIterations = 2;
    // Perto do meio (a menos disso) os pesos vao para a media com o espelho
    // (o gancho segue as duas coxas).
    float midlineBlend = 0.035f;
    // O quadril do calcao (10/10/2026): a passagem da pelve para a coxa numa
    // volta em torno da coxa; fora dela so a pelve ou so a coxa (a perna do
    // calcao rigida). O centro: na frente, a linha da virilha desenhada pelo
    // usuario, de frente, do lado de fora ("hipCreaseOuter": y e altura do
    // chao, em metros) ao gancho ("hipCreaseInner"); do lado de fora, a
    // altura da junta (a listra faz o arco curto em volta dela); atras,
    // "hipCreaseBackCenter" metros abaixo da junta (a dobra do gluteo); do
    // lado de dentro, a altura do gancho. A largura: "hipCreaseFront" na
    // frente (estreita: a coxa levantada entra na barriga num vinco fechado,
    // em vez de um arco de ~10 cm em volta da junta), "hipCreaseSide" do
    // lado de fora, "hipCreaseBack" atras (estreita e direta da pelve a
    // coxa: o gluteo fica redondo na pelve e o pano estica reto ate a coxa) e
    // "hipCreaseInside" do lado de dentro. Altura 0: desligado.
    std::array<float, 2> hipCreaseOuter { 0.0f, 0.0f };
    std::array<float, 2> hipCreaseInner { 0.0f, 0.0f };
    float hipCreaseFront = 0.035f;
    float hipCreaseSide = 0.14f;
    float hipCreaseBack = 0.07f;
    float hipCreaseBackCenter = 0.09f;
    float hipCreaseInside = 0.12f;
    // A dobra da frente: acima da linha da virilha, a cintura do calcao e a
    // barra da camisa seguem a auxiliar HipLift (uma fracao so da flexao do
    // quadril): a coxa que sobe dobra o pano para tras e a barra fica em cima
    // da perna (parada, a perna rigida subia atravessando a barra). Inteira
    // ate "hipLiftRise"[0] metros acima da barra da camisa, nada a partir de
    // [1]; em volta da coxa, da frente entre "hipLiftFront" (o cosseno do
    // angulo com a frente: nada ate [0], inteira a partir de [1]); perto do
    // meio, nada ate "hipLiftMidline"[0] metros dele, inteira a partir de
    // [1]. Rise [0, 0]: desligada.
    std::array<float, 2> hipLiftRise { 0.0f, 0.0f };
    std::array<float, 2> hipLiftFront { 0.3f, 0.8f };
    std::array<float, 2> hipLiftMidline { 0.02f, 0.08f };
    // O gluteo na extensao (21/10/2026): atras, o peso da pelve da faixa e
    // de "hipGluteRise" metros acima dela passa para a auxiliar GluteHelper
    // (uma fracao so da extensao): a coxa que vai para tras leva junto a
    // parte de baixo do gluteo e a borda da perna do calcao nao entalha nele.
    // Na flexao a auxiliar fica com a pelve: nada muda. 0: desligado.
    float hipGluteRise = 0.0f;
    int shortsSubdivision = 0;
    // O gancho fica com a malha base: a menos disso do meio, abaixo da junta
    // do quadril (0: o calcao todo). Denso ali, o agachamento fundo (as duas
    // coxas dobradas) virava lascas entre as pernas; grosso, nao.
    float shortsCoarseCrotch = 0.0f;
    // E o alto do calcao, por baixo da barra da camisa (metros a partir do
    // alto): grosso como a camisa, os dois dobram juntos (denso por baixo da
    // camisa grossa, um atravessava o outro) e a costura nao faz agulhas.
    float shortsCoarseTop = 0.04f;
    // As meias-juntas (21/10/2026): onde o vertice tem os dois ossos de uma
    // junta (joelho, tornozelo, cotovelo, pulso), a parte deles passa
    // pela auxiliar que gira a metade (a dobra nao amassa). Os auxiliares
    // estavam sem peso desde a troca para a montagem (08/10).
    bool halfJoints = false;
    // O ombro desenhado (ShoulderBand): os pesos da regiao por um campo no
    // referencial do braco. Larguras zero: desligado (os pesos de calor).
    ShoulderBand shoulderBand;
    // O meio do gancho preso na pelve: o raio em volta do ponto mais baixo
    // da costura do meio e a fracao dos pesos das coxas (pinWeights).
    float crotchPinRadius = 0.06f;
    float crotchPin = 0.4f;
    // As arestas do pano mais curtas que isso vezes a media em volta se juntam
    // (collapseShortEdges; 0 desliga).
    float clothShortEdge = 0.3f;
    // No gancho do calcao, os vertices a menos disso do meio vao para a
    // costura do meio (collapseInto; 0 desliga).
    float crotchBand = 0.022f;
    Color shirtColor = rgb(236, 236, 236);
    Color shortsColor = rgb(236, 236, 236);
    // A pintura (pintura.json, PaintDesign): as camadas do uniforme, do
    // meiao e da orelha nos espacos das pecas (PlayerPaintSpaces). Vazio =
    // so as cores das pecas.
    std::string paintFile;
    // O quanto a chuteira escurece (a cor dela vem da receita da chuteira).
    float bootShade = 0.5f;

    // A textura: o atlas unico (o lado em texels) e a densidade extra da
    // cabeca (o rosto e pintado nela).
    bool textured = true;
    int atlasSize = 2048;
    float faceDensity = 2.0f;
    // E a do cabedal e da lingua da chuteira (o cadarco pintado e fino).
    float laceDensity = 3.0f;
    // E a da camisa e do calcao (as bordas das faixas e dos numeros).
    float garmentDensity = 1.0f;
    // A largura da borda dos desenhos no bake, em texels (TextureBake
    // edgeTexels; 0 = um texel).
    float paintEdge = 0.0f;

    // O rosto pintado (os olhos, as sobrancelhas e a boca; FacePaint).
    FacePaintShape face;

    // O ajuste fino da cabeca depois do encaixe do pescoco (o queixo, a
    // mandibula, a papada): puxoes com "at" = [x, y, altura do chao] na metade
    // direita (MeshSculpt).
    std::vector<SculptPull> headSculpt;
    // A forma do rosto e do pescoco depois do pescoco mais curto: vertices
    // levados a lugares exatos ([x, y, altura do chao], metade direita;
    // MeshSculpt::moveVertices).
    std::vector<VertexMove> faceShape;
    // O queixo, a mandibula e o pescoco no molde de uma cabeca de referencia
    // (ReferenceFit), depois de faceShape.
    ReferenceFitShape referenceFit;
    // O pescoco mais curto, a mesma altura (NeckShortening): as alturas do
    // corpo na receita (barras, gola, meiao) e o esqueleto medido sao as do
    // corpo base e passam pela mesma conta.
    NeckShortening neck;
    // Os pesos da cabeca e do pescoco: o rosto preso na cabeca (os vertices
    // da cabeca a mais que "headPinAbove" acima da linha de cima do pescoco) e
    // a base do pescoco no alto do peito (ate "collarPinDepth" abaixo da gola,
    // a menos de "collarPinWidth" do meio); o pescoco passa de um ao outro.
    float headPinAbove = 0.012f;
    float collarPinDepth = 0.04f;
    float collarPinWidth = 0.075f;
    // Os pesos na faixa do pescoco entre a base dele e a linha da cabeca
    // (NeckBand): em linha ao longo dela (o alto do peito, o pescoco e a
    // cabeca) em vez do calor dos ossos - o giro do pescoco repartido. A base
    // e a gola, mas na frente nunca abaixo de "neckFrontBase" (o fundo do V e
    // peito: a pele entre ele e a base fica no alto do peito).
    bool neckGradient = false;
    float neckFrontBase = 1.5f;

    // Olhos modelados (a abertura e o globo) na cabeca: desligado - a
    // cabeca fica como veio e os olhos sao pintados na textura.
    bool modeledEyes = false;
    EyeShape eye;

    [[nodiscard]] static AssemblyRecipe defaults();
};

[[nodiscard]] std::optional<AssemblyRecipe> parseAssemblyRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
