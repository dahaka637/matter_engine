#pragma once

#include "Engine/Modeling/Objects/ProceduralObject.hpp"

// A trave com quadro de fundo (o tipo "trave"): a rede presa so na estrutura
// da propria trave, sem mastros atras (pedido do usuario, 25/10; a rede em
// mastros externos e um modelo futuro). Tubos de metal redondos:
//  - o quadro da frente: os dois postes e o travessao, com o vao interno da
//    regra (7,32 x 2,44 m entre as faces de dentro; IFAB, Regra 1) e o tubo de
//    ate 12 cm (o guia da FIFA pede 10 a 12 cm); os cantos soldados em
//    meia-esquadria ou dobrados;
//  - de cada lado, o quadro de tras dobrado: a barra de cima que leva a rede
//    para tras (0,80 m), a dobra, a diagonal ate o chao (2,00 m de fundo) e a
//    barra do chao de volta ao poste; e a barra de tras no chao.
// A rede (malha quadrada de 12 cm, fio de 6 mm trancado de PP; folga de 6% no fundo, 2,5% no teto e 8% nos lados):
// o teto, o fundo inclinado e as duas laterais, presos nos tubos (a presilha a
// cada 2 malhas), e um cabo tensor de 6 mm (~225 N) entre o teto e o fundo, de
// uma ponta a outra da barra de cima. As medidas da estrutura de tras sao as dos
// fabricantes que seguem a DIN EN 748 (a regra nao define a parte de tras).
//
// O objeto fica com a boca para +x (o campo), y a esquerda, z para cima.
// Colisao: um cilindro por trecho de tubo (fixo no chao).
namespace MatterEngine::Modeling {

[[nodiscard]] ObjectType goalFrameObjectType();

} // namespace MatterEngine::Modeling
