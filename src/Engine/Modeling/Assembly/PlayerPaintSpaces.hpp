#pragma once

#include "Engine/Modeling/Assembly/AssemblyRecipe.hpp"
#include "Engine/Modeling/Mesh/Mesh.hpp"
#include "Engine/Modeling/Paint/PaintSpace.hpp"
#include "Engine/Modeling/Skeleton/Skeleton.hpp"

#include <map>
#include <string>

// Os espacos da pintura do jogador montado (os moldes das pecas), medidos na
// malha final. Metros, alturas do chao; o lado esquerdo e o espelho do
// direito (cada ponto diz o lado dele):
//
//   tronco        a camisa sem as mangas desenrolada em volta do eixo de pe:
//                 x = arco a partir da costura do lado (+ para a frente),
//                 y = altura;
//   frente        a camisa vista de frente: x = +y do jogador (a direita de
//                 quem olha), y = altura;
//   costas        de costas: x = -y do jogador, y = altura;
//   manga         x = em volta do braco (a partir do alto), y = da barra da
//                 manga para o ombro;
//   gola          x = em volta do pescoco (a partir da frente), y = abaixo da
//                 borda da gola;
//   calcao        a perna do calcao desenrolada em volta da coxa: x = arco a
//                 partir da costura de fora (+ para a frente), y = ao longo
//                 da coxa a partir do quadril (para baixo);
//   calcao_frente o calcao de frente: x = +y do jogador, y = altura;
//   perna         a pele da perna: x = em volta da canela (a partir da
//                 frente), y = altura;
//   orelha        a placa de fora da orelha (medida na malha): fracoes da
//                 caixa dela, x de tras (0) para a frente (1), y de baixo (0)
//                 para cima (1).
//
// Ancoras: "barra", "axila", "gola_frente" e "gola_costas" (no tronco, na
// frente e nas costas); "barra" (no calcao, a barra da perna ao longo da
// coxa; no calcao de frente, a altura dela; na manga, 0); "borda" (na gola,
// 0); "joelho" (na perna, a altura da junta do joelho); "inicio" e "fim" (nas
// voltas, o comeco e o fim do eixo).
namespace MatterEngine::Modeling {

[[nodiscard]] std::map<std::string, PaintSpace> playerPaintSpaces(const Mesh& mesh, const AssemblyRecipe& recipe, const Skeleton& skeleton);

} // namespace MatterEngine::Modeling
