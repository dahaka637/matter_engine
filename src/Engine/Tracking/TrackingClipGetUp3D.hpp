#pragma once

#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingClipReference3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <span>

namespace MatterEngine {

// O levantar pelo clipe (11/11, ele: "Zerar absolutamente o brain do levantar-se... Refazer ABSOLUTAMENTE o treino do
// levantar-se... Serao 4 exercicios: levantar de costas, de brucos e o levantar correndo de costas, de brucos... Todos os
// levantares serao pelos clipes do unreal engine"; 13h45: "nesse do levantar correndo nao vamos usar TODO o clipe, apenas ate a
// parte que ele fica de pe"; vault 04_locomocao/plano_levantar_unreal_11_11.md).
//
// A recuperacao segue no tempo um dos quatro clipes da Unreal com o chao corrigido para o nosso corpo e cortados de pe
// (assets/animations/clips/levantar/), ancorado no mundo (DeepMimic: o clipe nao vem para baixo do corpo - o deslocamento dele, o
// impulso do levantar correndo, e cobrado). A nota de cada passo, entre 0 e 1:
//   0,50 a pose (as juntas no pai e a raiz no mundo), 0,05 a velocidade das juntas, 0,15 maos e pes, 0,15 o centro de massa e
//   0,15 a velocidade do centro de massa (o impulso; no parado, nada de pulo).
// Longe demais do clipe (o centro de massa a mais de FarCenterMeters, ou a cabeca FarHeadMeters abaixo da do clipe) por
// FarSeconds, o episodio acaba (o encerramento antecipado do DeepMimic): com a nota sempre >= 0, acabar custa o resto - nao cair
// sempre vale mais que qualquer pose (a regra dele, 11/11 09h45).
// O pulo (11/11 15h, a calibracao: um pulinho de 10 cm no fim com a pose perfeita ainda pagava 81% - a pose nao ve a translacao, e o
// pulinho era a queixa dele do levantar antigo): com os pes do clipe apoiados (o centro de massa do pe a menos de PlantedFootHeight
// do chao), os pes do corpo acima dos do clipe - o menor dos dois, alem de HopSlack - multiplicam a nota por exp(-(x / HopWidth)^2).
// Um pe so subindo (um passo) nao conta; os dois (um pulo), sim.
class TrackingClipGetUp3D final {
public:
    struct Terms {
        float pose = 0.0f, velocity = 0.0f, endEffector = 0.0f, centerOfMass = 0.0f, comVelocity = 0.0f, hop = 1.0f, trunk = 1.0f,
              emphasis = 1.0f, total = 0.0f;
    };
    struct Widths {
        // Os nucleos do contrato (TrackingGetUpContract3D::reward) divididos por isto (1: os do DeepMimic).
        float softness = 1.0f;
        // A largura da nota da velocidade do centro de massa (m/s).
        float comVelocity = 1.5f;
        // (11/11 17h40, ele: "reforcada nos exercicios enquanto similaridade com o clipe professor, em especial o levantar de costas"):
        // a pose e maos e pes metade como antes (o exp do corpo inteiro) e metade por parte do corpo (o tronco, cada braco, cada perna;
        // cada mao e pe: a media dos exp), com mais peso na semelhanca (Parts*Weight). Com o exp da soma de todas as juntas, um braco longe zerava a pose inteira e corrigir so
        // ele quase nao mudava a nota - nenhum incentivo para o braco (as folhas do levantar de costas, 17h30).
        bool parts = false;
        // (12/11 02h50, ele: "precisamos de alguma forma de forcar melhor a semelhanca com o clipe" no levantar de costas - o braco e a
        // coluna) O reforco pela pose (0 a 2): a nota vezes (tronco x braco esquerdo x braco direito)^(emphasis/3), cada um a nota de
        // pose daquela parte (o exp dos erros das juntas dela) - um braco ou a coluna fora da pose do clipe pesa de verdade no total.
        // (A primeira forma - mais peso e nucleo mais apertado so dentro da media por partes - mexia 1,4% na nota: nao forcava nada.)
        // Subido aos poucos pelo treino (a rampa): o fator duro de uma vez (20h38, pelas maos) derrubou a semelhanca.
        float emphasis = 0.0f;
    };
    static constexpr float PoseWeight = 0.50f, VelocityWeight = 0.05f, EndEffectorWeight = 0.15f, CenterWeight = 0.15f,
                           ComVelocityWeight = 0.15f;
    // Os pesos com a pose por partes: pose 0,55, velocidade das juntas 0,05, maos e pes 0,20, centro de massa 0,10, a velocidade
    // dele 0,10.
    static constexpr float PartsPoseWeight = 0.55f, PartsVelocityWeight = 0.05f, PartsEndEffectorWeight = 0.20f,
                           PartsCenterWeight = 0.10f, PartsComVelocityWeight = 0.10f;
    // FarFacing: a diferenca maxima da altura da frente do peito (o z da frente, de -1 de brucos a +1 de costas) contra a do clipe.
    static constexpr float FarCenterMeters = 0.5f, FarHeadMeters = 0.5f, FarSeconds = 0.2f, FarFacing = 1.2f;
    static constexpr float PlantedFootHeight = 0.10f, HopSlack = 0.02f, HopWidth = 0.04f;
    // (11/11 18h22, ele: o levantar de costas "fica com o tronco esticado para tras ao inves de avancar com ele para frente... e
    // visualmente FEIO"; medido: de 1,0 a 1,6 s o tronco a 56-76 graus do clipe, o resto a 1-15) O tronco: com a pose por partes, o
    // angulo entre a linha pelve -> peito alto do corpo e a do clipe multiplica a nota por exp(-(angulo / TrunkWidth)^2) - 76 graus
    // ~0, 15 graus 0,78.
    static constexpr float TrunkWidth = 0.5f;
    // (18h22) O peito virado ao contrario do clipe (FarFacing) so encerra depois deste tempo (o rolando: o clipe rola em ~0,25 s e com
    // os FarSeconds de 0,2 qualquer atraso encerrava - a avaliacao acabava aos 0,43 s, sem chance de aprender a rolar).
    static constexpr float FarFacingSeconds = 0.4f;
    // (11/11 18h50) O rolando: sentar e levantar como no de costas deixava o peito alto em ~0 com o clipe de brucos (-1) - a
    // diferenca de 0,9-1,0 nunca passava de FarFacing (so um instante aos 0,3 s), e a avaliacao "levantava" 16/16 sem rolar
    // (semelhanca 0,33). Para o exercicio variante (o rolando) o limite e este: sentado contra de brucos encerra.
    static constexpr float FarFacingVariant = 0.7f;
    // (19h10) E a tolerancia dele: com 0,4 s tentar rolar devagar morria no mesmo 0,6 s que sentar ou ficar de costas (o clipe vira
    // de costas para brucos em ~0,3 s) - sem diferenca entre tentar e nao tentar, nenhum gradiente para o rolamento. Com 0,7 s sentar
    // (~0,85 s fora) ainda encerra; um rolamento que chega ate ~0,9 s segue e ganha o resto do clipe.
    static constexpr float FarFacingVariantSeconds = 0.7f;
    // (11/11 20h20, ele: "O de rolar esta com problemas, e parte e por conta do clipe, talvez devamos dar mais liberdade a RL para
    // elaborar o movimento proprio de rolar, pois de fato pelo clipe de forma seca e impossivel") O rolamento livre no exercicio
    // variante: ate o clipe chegar de brucos (o peito alto abaixo de FreeRollProne), a nota e so o quanto o peito ja virou para baixo
    // (FreeRollWeight x (1 - frente)/2), sem pose; o relogio do clipe espera o corpo chegar de brucos pelo caminho dele (ou adianta,
    // se ele chegar antes), o clipe e reancorado no corpo (a posicao e o rumo) e o rastreio segue dali (de quatro e o levantar). Sem
    // chegar em FreeRollSeconds, o episodio acaba (o longe).
    static constexpr float FreeRollProne = -0.6f, FreeRollWeight = 0.5f, FreeRollSeconds = 2.0f;
    // (12/11 03h20, ele: "recompensar a velocidade e agilidade com que ele consegue levantar... no levantar correndo, estimular
    // extremamente tambem o IMPULSO de arrancada") A agilidade: o relogio do clipe acompanha o corpo - a cada decisao, se o corpo esta
    // mais parecido com um quadro a frente (ate AgilityMaxRate x o tempo, em AgilitySteps degraus), o relogio vai ate ele; o premio e
    // o peso do exercicio x a nota x o adiantamento por decisao. As velocidades do clipe vao vezes o ritmo (o tempo mais rapido).
    static constexpr float AgilityMaxRate = 1.75f;
    // (12/11 03h25) A regua da semelhanca dos graficos, fixa: a nota do corpo inteiro com os nucleos de 11/11 (largura 2, velocidade do
    // centro de massa 1,5 m/s), mesmo com a nota do treino mais larga (--levantar-unreal-largura/-vel) - o antes e o depois comparaveis.
    static constexpr float RulerSoftness = 2.0f, RulerComVelocityWidth = 1.5f;
    static constexpr int AgilitySteps = 3;
    // A altura da frente do peito alto (o x do modelo): +1 de costas, -1 de brucos.
    [[nodiscard]] static float chestFront(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
        std::span<const PhysicsBodyState3D> body);

    // groundHeight: o chao sob o clipe (a altura dos pes apoiados).
    [[nodiscard]] static Terms score(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
        std::span<const PhysicsBodyState3D> body, const TrackingClipFrame3D& reference, Widths widths, float groundHeight);
    // Longe demais agora (sem o tempo: quem chama conta FarSeconds): 0 nao, 1 pela posicao ou pela cabeca, 2 so pelo peito virado.
    [[nodiscard]] static int far(const TrackingGetUpContract3D& contract, const RagdollProfile3D& profile,
        std::span<const PhysicsBodyState3D> body, const TrackingClipFrame3D& reference, float facingLimit = FarFacing);
};

} // namespace MatterEngine
