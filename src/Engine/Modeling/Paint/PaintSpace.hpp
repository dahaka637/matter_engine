#pragma once

#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Paint/Shape2D.hpp"
#include "Engine/Modeling/Texture/TextureBake.hpp"

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

// Os espacos da pintura: cada um da a um ponto da superficie (de algumas
// partes) uma coordenada 2D, como o molde de uma costura - o tronco da camisa
// desenrolado em volta do corpo (metros a partir da costura do lado), a vista
// de frente de uma peca (o numero), a orelha. As camadas da pintura sao
// desenhadas nesses espacos: a faixa do lado e uma tira reta de largura fixa
// no tronco desenrolado, e a borda dela nao depende de onde a malha tem
// vertices.
namespace MatterEngine::Modeling {

// Um ponto num espaco: a coordenada, o lado da superficie (-1 direito, y < 0;
// +1 esquerdo) e o tamanho do pixel nas unidades do espaco.
struct SpacePoint {
    Point2 point;
    int side = -1;
    float footprint = 0.0f;
};

// O espaco: onde cada amostra cai (nada fora dele) e as ancoras, as linhas
// com nome ("barra", "axila") que o desenho pode usar nas coordenadas.
struct PaintSpace {
    std::function<std::optional<SpacePoint>(const SurfaceSample&)> locate;
    std::map<std::string, float> anchors;
};

// O plano: a projecao em "u" e "v" (metros, a partir de "origin"); so as
// partes dadas e as faces viradas para "facing" (cosseno minimo).
[[nodiscard]] PaintSpace planarSpace(std::vector<int> parts, Vec3 origin, Vec3 u, Vec3 v, Vec3 facing, float minFacing);

// A volta desenrolada em torno de um eixo (o molde do tronco, da perna): a
// superficie fatiada em planos perpendiculares ao eixo; a coordenada e (o
// comprimento de arco na fatia a partir da costura, + para "front"; a
// distancia ao longo do eixo a partir de "origin"). A costura e a linha da
// fatia mais para "outward" (o lado direito; o esquerdo e o espelho, com o
// eixo espelhado), alisada numa reta ao longo do eixo; um pedaco de fatia que
// nao cruza a costura (partido por um buraco, como a cava da manga) fica sem
// coordenada. Cada metade da malha (y < 0 e y > 0) e fatiada com o eixo dela.
// Numa volta fechada (a perna) o arco vai de meia volta para tras a meia
// volta para a frente: o corte fica do lado oposto a costura (a costura de
// dentro), e uma fatia aberta numa brecha pequena (o recorte da barra) conta
// como fechada. Entre duas fatias o arco e a media pelo caminho curto, nunca
// atravessando o corte, e so das fatias que cortam o triangulo da amostra
// (perto da barra a fatia de baixo se parte longe dali).
struct UnrolledSpaceSettings {
    std::vector<int> parts;
    Vec3 origin;
    Vec3 axis { 0.0f, 0.0f, 1.0f };
    Vec3 front { 1.0f, 0.0f, 0.0f };
    Vec3 outward { 0.0f, -1.0f, 0.0f };
    // A distancia entre as fatias.
    float step = 0.002f;
    // So entre estes valores do eixo (a costura e ajustada so ali).
    float seamFrom = -1e9f;
    float seamTo = 1e9f;
    // Os triangulos que entram (pelo centro; vazio = todos das partes): o
    // tronco sem as mangas.
    std::function<bool(Vec3)> include;
    // So as faces viradas para fora do eixo (as viradas para ele - a dobra
    // recuada por dentro de uma barra - ficam sem coordenada) e com
    // |normal . eixo| ate isso (1 = todas). Desligado: a face de dentro fica
    // sem tinta onde a de fora tem.
    bool outerOnly = false;
    float maxAxisFacing = 1.0f;
};
[[nodiscard]] PaintSpace unrolledSpace(const Mesh& mesh, const UnrolledSpaceSettings& settings);

} // namespace MatterEngine::Modeling
