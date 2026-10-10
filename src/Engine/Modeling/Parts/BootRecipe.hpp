#pragma once

#include "Engine/Modeling/Parts/ProfileCurve.hpp"
#include "Engine/Modeling/Raster/Color.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

// A chuteira em numeros (partes/chuteira.json). Medidas em metros; "t" vai de
// 0 no calcanhar a 1 no bico; "v" e o lado (+ para dentro do pe, - para
// fora); alturas a partir do chao (a ponta das travas).
struct BootRecipe {
    struct Stud {
        float t = 0.5f;
        float v = 0.0f;
        float baseRadius = 0.012f;
        float tipRadius = 0.0094f;
    };

    // Tamanho e posicao no pe.
    float length = 0.29f;
    // O calcanhar atras do tornozelo (a junta do pe), no rumo do pe.
    float heelBehindAnkle = 0.063f;

    // A base do cabedal (onde ele encontra a sola): meia largura dos dois
    // lados, antes de as pontas arredondarem.
    ProfileCurve medialHalfWidth;
    ProfileCurve lateralHalfWidth;
    // O topo no meio (peito do pe, lingua, bico) e a borda da abertura.
    ProfileCurve topHeight;
    ProfileCurve collarHeight;
    // A base da sola (com o bico subindo) e a espessura dela.
    ProfileCurve soleBottom;
    float soleThickness = 0.0095f;
    // Quanto a sola passa do cabedal (a borda que aparece).
    float soleOverhang = 0.0032f;
    // Na ponta do bico a sola passa do cabedal so isto e afina (fracao).
    float soleToeOverhang = 0.001f;
    float soleToeThinning = 0.4f;
    ProfileCurve soleRelief;
    std::vector<float> solePanelStations;

    // As pontas arredondadas. De cima, a largura fecha nos ultimos
    // heelRound/toeRound do comprimento (superelipse: 2 = elipse, maior = mais
    // quadrada); de lado, a altura do bico fecha em noseRound. O "endScale"
    // e o tamanho do ultimo anel (a face pequena que fecha cada ponta).
    float heelRound = 0.15f;
    float heelRoundExponent = 2.0f;
    float heelEndScale = 0.15f;
    float toeRound = 0.1f;
    float toeRoundExponent = 2.2f;
    float toeEndScale = 0.05f;
    // O bico fecha num cone curto para a frente (m): sem tampa plana na ponta.
    float toeTipReach = 0.0015f;
    float noseRound = 0.07f;
    float noseExponent = 2.2f;
    // O contraforte: o alto do calcanhar fica heelSlant metros a frente da
    // base e o meio estufa heelBulge para tras; some ate heelSlantReach (t).
    float heelSlant = 0.022f;
    float heelBulge = 0.004f;
    float heelSlantReach = 0.24f;

    // Secao: expoente da superelipse (2 = elipse; maior = mais quadrada) e
    // quanto o lado estufa acima da sola.
    float sectionExponent = 2.6f;
    float sideBulge = 1.035f;
    // O contraforte em forma de ovo: quanto o lado do calcanhar estufa (fracao
    // da meia largura) a um terco da altura; some ate t = 0,17.
    float heelSideBulge = 0.08f;

    // A abertura: de onde ate onde (t), a espessura da borda e o fundo.
    float openingStart = 0.045f;
    float openingFull = 0.085f;
    // O fundo da abertura em U: o meio comeca a abrir antes (t) que os lados.
    float openingRound = 0.03f;
    float tongueStart = 0.345f;
    float tongueTop = 0.4f;
    // Quanto o alto da lingua deita para tras (sobre a abertura).
    float tongueLean = 0.004f;
    // A janela (t) em que as paredes da abertura passam para o perfil fechado
    // na lingua (maior que a do meio: o lado nao vinca).
    float tongueWallBlend = 0.06f;
    ProfileCurve tongueHalfWidth;
    float tongueThickness = 0.0026f;
    float rimThickness = 0.004f;
    // Altura virtual das paredes acima da borda: menor = inclinam mais para
    // dentro no alto da abertura.
    float collarLean = 0.045f;
    // Inclinacao por trecho: gola estreita atras e espaco para o tornozelo.
    ProfileCurve collarLeanProfile;
    // A gola de fora fica mais baixa no meio da abertura (o tornozelo de
    // fora e mais baixo): quanto, no ponto mais fundo.
    float collarLateralDrop = 0.006f;
    float insoleAboveSole = 0.006f;

    // Estacoes ao longo do comprimento (os aneis do loft).
    std::vector<float> stations;

    // Travas.
    std::vector<Stud> studs;
    int studSides = 5;
    float studBevel = 0.003f;

    // Cadarco: as fileiras (t) e a meia distancia entre os ilhoses.
    float laceStart = 0.455f;
    float laceEnd = 0.72f;
    int laceRows = 6;
    float laceHalfSpacingStart = 0.016f;
    float laceHalfSpacingEnd = 0.0135f;
    // A tira de cima (largura ao longo do pe, espessura) e quanto ela passa
    // da linha dos ilhoses (a ponta fica na fenda).
    float laceWidth = 0.0095f;
    float laceThickness = 0.0017f;
    float laceOverhang = 0.004f;
    // O cruzado: a tira e onde ela termina, embaixo da tira de cima: no lado,
    // a fracao da distancia dos ilhoses; ao longo do pe, o recuo da fileira
    // (as pontas de duas diagonais nao se empilham).
    float crossWidth = 0.007f;
    float crossThickness = 0.0012f;
    float crossInset = 0.65f;
    float crossEndShift = 0.0029f;
    // O canal onde o cadarco deita (a lingua abaixo das abas dos ilhoses).
    float laceChannel = 0.0012f;
    // A folga entre as pecas do cadarco e o que estiver embaixo.
    float laceClearance = 0.00008f;
    // As pontas entram no cabedal: nos ultimos laceDive metros a tira de
    // cima desce e entra na fenda do ilhos; nos ultimos crossDive a diagonal
    // entra no cabedal (debaixo da tira de cima).
    float laceDive = 0.003f;
    float crossDive = 0.0025f;
    // A fenda escura do ilhos (desenhada no cabedal): o comprimento no
    // sentido do lado.
    float slotLength = 0.0022f;
    // As pecas em relevo (lingua, cruzado, tiras) sobem da pegada com a
    // parede inclinada: a pegada passa do topo isto de cada lado.
    float reliefBevel = 0.0005f;

    // O visual low-poly: deslocamento aleatorio dos vertices (com semente).
    float jitter = 0.0033f;
    float topJitter = 0.0014f;
    // O facetado na biqueira (fracao do resto): mais liso, sem amassar o bico.
    float toeJitter = 0.55f;
    // O passo (m) entre estacoes a partir do qual o tremor e cheio; abaixo
    // dele diminui junto (estacoes densas ficam lisas, sem faixa marcada).
    float jitterSpacing = 0.018f;
    // Fracao do passo entre estacoes que cada vertice anda ao longo do pe.
    float lengthJitter = 0.18f;
    unsigned seed = 7;

    // Cores.
    Color upperColor = rgb(108, 108, 112);
    Color soleColor = rgb(80, 80, 84);
    Color studColor = rgb(94, 94, 98);
    Color laceColor = rgb(94, 94, 98);
    Color eyeletColor = rgb(24, 24, 27);

    // A receita padrao (o desenho a partir das referencias).
    [[nodiscard]] static BootRecipe defaults();
};

// Le os campos presentes no JSON por cima do padrao (o que faltar fica).
[[nodiscard]] std::optional<BootRecipe> parseBootRecipe(std::string_view json, std::string* error = nullptr);

} // namespace MatterEngine::Modeling
