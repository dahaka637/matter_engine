#pragma once

#include "Engine/Athlete/Body/LocomotionPartner3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

struct TrackingGetUpAmpSettings3D {
    std::string assetsDirectory;
    std::uint32_t environments = 512;
    std::uint32_t threads = 0;
    std::uint64_t seed = 1;
    // A avaliacao: so as quedas (sem os comecos nos clipes).
    bool evaluation = false;
    // O episodio (s) e, no treino, a fatia que comeca num instante sorteado de um clipe de levantar.
    float episodeSeconds = 6.0f;
    float clipStartFraction = 0.25f;
    // A fatia que comeca de pe, num instante do clipe parado (31/10: ele nao se mantinha de pe depois de levantar).
    float standingStartFraction = 0.15f;
    // O levantar v2 (31/10 17h, ele): a fatia que comeca caida e levanta ja indo para um alvo (em arrancada, virado
    // para ele) e a do evitar a queda (de pe, um empurrao que o derrubaria: evitar o chao e se recompor rapido).
    float movingFraction = 0.0f;
    float avoidFraction = 0.0f;
    // A fatia que comeca na arrancada agachada (o clipe crouched_to_sprint, virado para um alvo, com o pedido de ir a
    // ele; 31/10, ele: "nao e apenas um levantar e entao ir, mas sim um levantar preparado ja indo").
    float sprintStartFraction = 0.0f;
    // O alvo do levantar em movimento: a distancia (m) e a velocidade pedida (m/s).
    float targetMinimumMeters = 4.0f;
    float targetMaximumMeters = 8.0f;
    float moveMinimumSpeed = 2.5f;
    float moveMaximumSpeed = 5.0f;
    // O empurrao do evitar a queda: o curto (N, 0,1-0,2 s) e, numa fatia, o continuo (N, s).
    float avoidMinimumNewtons = 1200.0f;
    float avoidMaximumNewtons = 1800.0f;
    float avoidHoldFraction = 0.3f;
    float avoidHoldMinimumNewtons = 350.0f;
    float avoidHoldMaximumNewtons = 550.0f;
    float avoidHoldMinimumSeconds = 0.5f;
    float avoidHoldMaximumSeconds = 1.2f;
    // O empurrao de pe do jogo de pes e do autoequilibrio parado (N, 0,1-0,2 s) e, numa fatia, o continuo (N, s).
    float pushMinimumNewtons = 400.0f;
    float pushMaximumNewtons = 900.0f;
    float pushHoldFraction = 0.0f;
    float pushHoldMinimumNewtons = 150.0f;
    float pushHoldMaximumNewtons = 350.0f;
    float pushHoldMinimumSeconds = 1.0f;
    float pushHoldMaximumSeconds = 3.0f;
    // A locomocao (31/10 22h35, ele: so esta rede daqui em diante, com "os treinos de parado, tras, direita, esquerda,
    // diagonais frente e diagonais costas e o parado girando suave"; o autoequilibrio segue o principal: "se locomover
    // resistindo a impactos e adversidades"): os exercicios basicos do B21. A fatia dos rumos (andar ou trotar num dos 8
    // rumos em relacao ao rumo do corpo, mantendo-o) e a do giro parado suave.
    float locomotionFraction = 0.0f;
    float turnFraction = 0.0f;
    // As faixas do B21: andar (nesta fatia) e trotar (m/s; o minimo de 2 do trote, o pedido dele de 31/10 07h45). A
    // avaliacao pede o meio do trote.
    float walkFraction = 1.0f / 3.0f;
    float walkMinimumSpeed = 0.8f;
    float walkMaximumSpeed = 1.4f;
    float trotMinimumSpeed = 2.0f;
    float trotMaximumSpeed = 3.0f;
    // O rumo pedido (0 a frente, de 45 em 45 graus para a esquerda, ate 7; < 0: sorteado no treino e, na avaliacao, o
    // do mundo, de 8 em 8).
    int heading = -1;
    // Os empurroes em movimento (a regua do comeco do B21: curtos de 300-600 N por 0,1-0,2 s e, numa fatia, continuos de
    // 100-180 N por 1-2 s), nos rumos e no giro; na avaliacao, um so, de 2 a 3 s depois do comeco. Sem forca (as duas
    // faixas em 0), sem empurrao (31/10 22h45, ele: "por hora tira os empurroes - so vai entrar depois que ele estiver se
    // locomovendo bem").
    float movingPushMinimumNewtons = 300.0f;
    float movingPushMaximumNewtons = 600.0f;
    float movingPushHoldFraction = 0.3f;
    float movingPushHoldMinimumNewtons = 100.0f;
    float movingPushHoldMaximumNewtons = 180.0f;
    // O premio do passo (01/11, ele: "nenhum passo ate agora"; o feet air time de Rudin et al. 2021, "Learning to Walk
    // in Minutes"): com o pedido de deslocamento, de pe, a cada pe que pousa, este peso vezes o tempo no ar (ate 0,5 s)
    // menos 0,15 s - passo de verdade ganha, arrastar perde. 0: desligado.
    float stepReward = 0.0f;
    // A rl_final (01/11, ele: "precisamos achar uma abordagem que consigamos fazer ela se locomover! e BEM, com
    // estabilidade e velocidade e excelente auto-equilibrio"; vault 04_locomocao/plano_rl_final_01_11.md). Nos rumos, as
    // faixas pelos clipes (GaitTable): a fatia da corrida e a do sprint (andar em walkFraction, o resto trota), a
    // velocidade pedida a do clipe do passo sorteado +-15%; a fatia dos comecos ja no clipe, num instante sorteado e na
    // velocidade dele (o RSI do DeepMimic e do AMP, o que ensina a correr); o tempo do episodio dos rumos e do giro (s; 0:
    // episodeSeconds); a queda encerrando esses episodios (o early termination do DeepMimic); e a troca do pedido no meio
    // deles a cada um tempo sorteado nesta faixa (s; 0 0: so a parada no meio de antes) - parar, mudar de faixa e de rumo,
    // virar correndo.
    float runFraction = 0.0f;
    float sprintFraction = 0.0f;
    float locomotionClipFraction = 0.5f;
    float locomotionSeconds = 0.0f;
    bool locomotionTermination = false;
    float commandChangeMinimumSeconds = 0.0f;
    float commandChangeMaximumSeconds = 0.0f;
    // Na avaliacao dos rumos, a faixa (-1: o meio do trote nos 8 rumos, como antes; 0 andar, 1 trote, 2 corrida, 3
    // sprint: para a frente, de pe no parado em alerta, na velocidade do primeiro clipe da faixa).
    int evaluationBand = -1;
    // A rampa da velocidade com pedido ate a pedida: (feita/pedida) elevado a isto (0,5, a raiz: um passo pequeno ja
    // vale; 1, linear: mais ganho de 2 para 4 m/s num pedido de 6 - a rl_final, 01/11 09h30, o plato das faixas altas).
    float speedRampPower = 0.5f;
    // A imitacao do clipe do passo na locomocao (01/11 14h50, ele: "o movimento dele no andar/trote e bem diferente do
    // clipe... so passinhos curtinhos e ele torto... uma solucao rapida que corrija isso, depois voltamos a soltar"): a
    // nota do DeepMimic (TrackingGetUpContract3D::reward) contra o quadro do clipe do passo pedido, no tempo dele desde o
    // comeco (em ciclo), alinhado a posicao e ao rumo do corpo, vezes este peso, somada a nota de estado. 0: desligada.
    float imitationWeight = 0.0f;
    // O imitador de um clipe so (01/11 18h, ele: "essa locomocao e INACEITAVEL"): nos rumos, so os passos deste clipe
    // (vazio: todos pelas fatias).
    std::string onlyClip;
    // O clipe nos motores (01/11 18h45, ele: "toda vez voce da a mesma desculpa para ela nao seguir nada do clipe"): na
    // locomocao, o alvo de cada junta e o do clipe do passo (no quadro do fim da decisao) vezes (1 - clipBlend) mais o da
    // rede vezes clipBlend - o clipe comanda as juntas e a rede so corrige. 1: a rede livre (como antes); soltar = subir.
    float clipBlend = 1.0f;
    // O rolar pelo clipe (02/11 01h40, ele: "nos ja tinhamos a principio o movimento do rolar-se no chao, porque nao esta
    // sendo usado para referencia?"): deitado de costas, o trecho do rolamento do levantar de costas (stand_up_back_kimodo,
    // 0 a 0,75 s: a barriga de cima para baixo) nos motores - o alvo = clipe x (1 - rollBlend) + rede x rollBlend -, sem
    // nota de imitacao (nada a trapacear); o resto do levantar com a rede livre. 1: desligado; o desmame sobe ate 1.
    float rollBlend = 1.0f;
    // A queda programada (01/11 22h50, ele: "quando ele de fato cai, eu preferia aquele nosso ragdoll que temos na
    // locomocao fisica... tenta cobrir a cabeca e se encolher... fica meio mole... e ai o ato de levantar ocorresse apos
    // isso"): fora da locomocao, a queda sem volta de quem ja ficou de pe (e os comecos ja caindo) e a da locomocao
    // fisica (ProtectiveFall3D) ate assentar, sem a rede - os passos dela dentro de uma decisao -; dai a rede levanta.
    bool protectiveFall = false;
    // No laboratorio, a queda programada passo a passo (uma decisao por vez, a acao da rede ignorada ate assentar): no
    // treino ela roda inteira dentro de uma decisao, e na tela o corpo pulava de caindo para deitado.
    bool protectiveFallStepwise = false;
    // O levantar guiado pelo clipe (01/11 23h10, ele: "nossa meta vai ser 1.5 - 1.8 segundos para ficar de pe"; "os
    // movimentos dele estao MUITO zuados... devem se aproximar mais dos clipes, e ele hoje ta ficando em pe TODO TORTO ao
    // inves de aproximar-se do IDLE ALERTA apos levantar-se"): deitado, com a rede no comando, o clipe de levantar mais
    // parecido com a pose de agora (de brucos ou de costas, normal ou espelhado, no instante mais proximo) e seguido -
    // nos motores com a mistura clipBlend e na nota com a imitacao imitationWeight -, acelerado para o clipe inteiro durar
    // getUpSeconds; no fim dele, de pe, o parado em alerta como no parado (a imitacao, a quietude e o ponto).
    bool getUpGuide = false;
    float getUpSeconds = 1.7f;
    // A troca de rede ao ficar de pe (01/11, ele: o levantar volta a ser so levantar e evitar quedas, "fazendo a troca
    // de RL no momento que recem fica de pe"): o episodio acaba (truncado, como o fim do tempo) quando ele fica de pe e
    // firme por 0,5 s - o mesmo da entrega no jogo - depois de ter estado no chao ou, no evitar a queda, do empurrao.
    bool handoffOnStand = false;
    // As quedas da locomocao (01/11, a interligacao das redes: "para que justamente um consiga aprender com o outro"): a
    // fatia que comeca no instante em que a rede da locomocao caiu no treino dela (setFallStarts; sem elas, uma queda).
    float locomotionFallFraction = 0.0f;
    // A rede da locomocao como parceira (01/11, ele: "ambas as redes aprendendo em conjunto em todos os treinos";
    // setLocomotionPartner): depois da troca ela dirige parada por partnerSeconds - se ela cair nesse tempo, a troca foi
    // ruim: partnerFallPenalty e o fim do episodio (so no treino) -; e, com partnerAvoid, o evitar a queda comeca com ela
    // de pe levando o empurrao forte, a rede do levantar assumindo na queda iminente dela (no treino e na avaliacao).
    float partnerSeconds = 2.0f;
    float partnerFallPenalty = 10.0f;
    bool partnerAvoid = false;
    // Nos comecos da queda iminente (o evitar a queda com a parceira e as quedas da locomocao), a punicao da primeira vez
    // que o tronco, o pescoco ou a cabeca tocam o chao (o evitar a queda de verdade; 0: so a do potencial).
    float touchDownPenalty = 0.0f;
    // A protecao na queda (01/11, ele: "quando ele via cair no chao tentasse se proteger e evitar de cair, levando as
    // maos, joelhos, girando quadril, coluna... para conseguir se recobrar antes de cair"): nesses comecos, ate o tronco
    // tocar o chao ou ficar de pe, o premio do primeiro apoio das maos, dos bracos ou dos joelhos no chao (uma vez).
    float supportBonus = 0.0f;
    // A flexao na queda (01/11, ele: "assim que ele cai, ele puxa os bracos e pernas dele para tras... o correto seria
    // ao contrario, para frente"): na protecao e por FlexAfterTouchSeconds depois do primeiro toque do tronco, este peso
    // vezes a flexao do passo - as maos a frente do peito e os joelhos a frente da pelve (ate 0,25 m: +1) menos a coluna
    // arqueada para tras (30 graus: -1).
    float flexReward = 0.0f;
    // O cenario sujo (01/11, ele: "nossa rede tem dificuldade de levantar-se se o cenario nao e limpo - tem algo
    // esbarrando nele, ou um boneco por cima"): no treino, a fatia dos episodios com uma parede ou caixa fixa encostada
    // no corpo (a 4-44 cm dele, de 0,4 a 1,5 m de altura, num lado sorteado) e a com um peso comprido solto caindo de
    // atravessado em cima dele (20-60 kg, como um boneco por cima).
    float obstacleFraction = 0.0f;
    float weightFraction = 0.0f;
    // Os ambientes (02/11, ele: "parede ao lado, paredes aos lados, pilares, corpos ragdoll embaixo dele (1..2 ou mais) e
    // tambem em cima dele, em terrenos inclinados" e acidentados), so no treino, cada um sorteado por si: as paredes dos
    // dois lados (o corredor, 4-44 cm do corpo, 1-2,5 m de altura), 1-4 pilares em volta, a rampa (6-18 graus) e o chao
    // acidentado (calombos de 2-8 cm) embaixo da queda, 1-3 corpos soltos embaixo da queda e um corpo solto caindo em
    // cima dele ja no chao.
    float corridorFraction = 0.0f;
    float pillarFraction = 0.0f;
    float slopeFraction = 0.0f;
    float roughFraction = 0.0f;
    float bodiesUnderFraction = 0.0f;
    float bodyOnTopFraction = 0.0f;
    // A variante ambientada do evitar a queda (02/11, ele: "com obstaculos, paredes, corpos, pilares, rampas e tudo mais,
    // bem diversificado, para ele aprender a evitar quedas em ambientes diversos"): a fatia dos episodios do evitar com
    // um ambiente garantido (parede, corredor, pilares, rampa suave, chao acidentado em volta ou corpos caidos em volta)
    // mais os outros nas fatias de cima; o chao entra antes de ele ficar de pe.
    float avoidSceneryFraction = 0.0f;
    // Os ambientes tambem na avaliacao (o laboratorio, o exercicio "levantar_ambientes"); o treino nunca liga. Cada
    // episodio dela tem um dos ambientes garantido (sorteado), mais os outros nas fatias.
    bool evaluationScenery = false;
    // Na avaliacao, o que se mede: 0 as quedas (o levantar), 1 o levantar em movimento, 2 o evitar a queda, 3 o
    // equilibrio parado (de pe, um empurrao com a regua do parado com empurrao forte do B21: o curto de 900-1200 N ou,
    // em 30%, o continuo de 205-275 N por 1-3 s), 4 os rumos (de pe, no meio do trote, mantendo o rumo do corpo, um
    // empurrao em movimento), 5 o parado (de pe, sem empurrao), 6 o giro parado suave (0,6 rad/s, metade para cada lado,
    // um empurrao em movimento), 7 as quedas da locomocao (a do mundo entre as de setFallStarts).
    std::uint32_t evaluationKind = 0;
};

// Um episodio do levantar livre que acabou (no fim do tempo).
struct TrackingGetUpAmpEpisode3D {
    std::uint32_t world = 0;
    // 0: uma queda; 1: um instante de um clipe de levantar; 2: de pe (o parado); 3: uma queda com alvo (o levantar em
    // movimento); 4: de pe com o empurrao forte (o evitar a queda); 5: a arrancada agachada com alvo; 6: um rumo (o rumo
    // pedido, 0 a 7, vem em startSide); 7: o giro parado suave; 8: uma queda da locomocao.
    std::uint32_t startKind = 0;
    // Como comecou: 0 de brucos, 1 de costas, 2 de lado (ou sentado, ajoelhado).
    std::uint32_t startSide = 0;
    float startHeadHeight = 0.0f;
    float seconds = 0.0f;
    // A primeira vez de pe (s desde o comeco; < 0: nao ficou).
    float firstStandSeconds = -1.0f;
    float meanTaskReward = 0.0f;
    float headHeight = 0.0f;
    float pelvisHeight = 0.0f;
    bool standing = false;
    // O levantar v2: a velocidade pedida no comeco (0: ficar), quando chegou ao alvo (s; < 0: nao chegou), a
    // velocidade media rumo ao alvo enquanto de pe com o pedido (m/s), se caiu depois de ja ter ficado de pe e, no
    // evitar a queda, quando se recompos (s desde o fim do empurrao; < 0: nao se recompos).
    float commandSpeed = 0.0f;
    float reachedSeconds = -1.0f;
    float meanTowardSpeed = 0.0f;
    bool fellAfterStanding = false;
    float recomposedSeconds = -1.0f;
    // A semelhanca com a pose do parado em alerta no fim (0 se nao esta de pe).
    float endPoseMatch = 0.0f;
    // Nos rumos e no giro: o erro medio do rumo do corpo (graus), de pe.
    float meanFaceError = 0.0f;
    // A coluna como a do parado em alerta no fim (TrackingGetUpPostureProbe3D::idleSpineMatch; 0 se nao esta de pe).
    float endSpineMatch = 0.0f;
    // A troca de rede (s desde o comeco; < 0: o episodio acabou sem ela).
    float handoffSeconds = -1.0f;
    // Depois da troca, se a rede da locomocao caiu no tempo dela e quanto ela dirigiu (s; < 0: sem a parceira).
    bool partnerFell = false;
    float partnerSeconds = -1.0f;
    // Se o tronco, o pescoco ou a cabeca tocaram o chao em algum momento (o evitar a queda: nao) e se, protegendo-se,
    // ele se apoiou nas maos, nos bracos ou nos joelhos antes.
    bool touchedDown = false;
    bool supported = false;
    // A flexao media na queda (a de flexReward, sem o peso; 0 sem a queda iminente).
    float meanFlex = 0.0f;
    // Se o episodio teve a parede ou o peso (o cenario sujo).
    bool obstructed = false;
    // Nos rumos, a faixa do passo do comeco (0 andar, 1 trote, 2 corrida, 3 sprint; a rl_final).
    std::uint32_t band = 0;
};

// O que o visualizador do laboratorio desenha de um mundo: como comecou, o alvo (o levantar em movimento, enquanto o
// pedido dura) e o empurrao de agora.
struct TrackingGetUpAmpWorldView3D {
    std::uint32_t startKind = 0;
    bool hasTarget = false;
    Vec3 target;
    float commandSpeed = 0.0f;
    // O pedido de deslocamento (m/s, no chao; zero sem pedido) e, nos rumos e no giro, o rumo pedido do corpo (rad).
    Vec3 moveVelocity;
    bool hasFace = false;
    float faceYaw = 0.0f;
    bool pushing = false;
    Vec3 pushForce;
};

// O lote do levantar livre (vault 04_locomocao/plano_compostos_31_10.md, "O levantar livre"): o ragdoll do jogo a 0%
// comecando de quedas sorteadas (uma pose de um clipe, o corpo inteiro num sentido sorteado, solto com os musculos
// fracos ate assentar no chao) e, no treino, tambem de instantes dos clipes de levantar. A acao e a do levantar do
// DeepMimic (o alvo dos 39 eixos); a observacao, so o corpo; a nota daqui e a da tarefa (TrackingGetUpPostureProbe3D),
// e a do estilo vem do discriminador do treinador, com os pares do AMP de cada passo e os dos clipes.
class TrackingGetUpAmpTraining3D final {
public:
    static constexpr int TicksPerDecision = 4;
    static constexpr float TickSeconds = 1.0f / 120.0f;
    // Os clipes do estilo: os de levantar (e os espelhados) tambem dao os comecos; o parado so o estilo.
    static constexpr const char* GetUpClips[4] { "stand_up_front_kimodo", "stand_up_back_kimodo",
        "stand_up_front_kimodo_espelho", "stand_up_back_kimodo_espelho" };
    // (31/10 18h, ele: o padrao de ficar de pe parado e o parado em alerta, nao o natural; com o espelho, o apoio no
    // outro pe.)
    static constexpr const char* StandingClip = "alert_idle";
    static constexpr const char* StandingMirrorClip = "alert_idle_espelho";
    // Os clipes do estilo do levantar em movimento (a arrancada e a corrida; no lugar, com a velocidade nominal somada
    // aos pares do discriminador): so o estilo e os comecos das quedas.
    static constexpr const char* MovingClips[5] { "idle_to_sprint", "crouched_to_sprint", "mixamo_running",
        "run_forward_medium", "sprint_forward" };
    // Os clipes do trote, na ordem dos rumos (de 45 em 45 graus para a esquerda): o estilo e os comecos do tipo 6. E os
    // do andar e do giro parado, so o estilo.
    static constexpr const char* HeadingClips[8] { "trot_forward", "trot_forward_left", "jog_strafe_left",
        "trot_backward_left", "trot_backward", "trot_backward_right", "jog_strafe_right", "trot_forward_right" };
    // (01/11 19h, ele: "por que temos um exercicio de andar, sendo que nao usamos andar no jogo? apenas trote, corrida e
    // sprint": os clipes de andar sairam do estilo e dos passos; ficam os giros parados.)
    static constexpr const char* LocomotionStyleClips[2] { "turn_left", "turn_right" };
    // A rl_final: as quedas com protecao (01/11; ele: "quando ele via cair no chao tentasse se proteger... levando as
    // maos, joelhos, girando quadril, coluna"; o Kimodo e os tropecos da Mixamo, com os espelhos) - o estilo e, com o
    // evitar a queda ligado, metade dos comecos no clipe, num instante da queda, com a protecao valendo (como no evitar).
    static constexpr const char* ProtectionClips[16] { "queda_protecao_frente", "queda_protecao_frente_espelho",
        "queda_protecao_tras", "queda_protecao_tras_espelho", "queda_protecao_lado", "queda_protecao_lado_espelho",
        "queda_protecao_tropeco", "queda_protecao_tropeco_espelho", "queda_protecao_tropeco_corrida",
        "queda_protecao_tropeco_corrida_espelho", "queda_protecao_tropeco_queda", "queda_protecao_tropeco_queda_espelho",
        "queda_protecao_empurrao_lado", "queda_protecao_empurrao_lado_espelho", "queda_protecao_empurrao_tras",
        "queda_protecao_empurrao_tras_espelho" };
    // A rl_final: os clipes da corrida e do sprint (o estilo e os comecos dessas faixas).
    static constexpr const char* RunClips[5] { "jog", "jog_backward", "run_forward_medium", "mixamo_running",
        "run_backward" };
    static constexpr const char* SprintClips[7] { "sprint_forward", "mixamo_sprint", "sprint", "sprint_maximo",
        "sprint_backward", "sprint_strafe_left", "sprint_strafe_right" };
    // As entradas do comando, depois das do corpo: o rumo do deslocamento pedido em relacao ao corpo (cos, sen), a
    // distancia (/10 m, ate 1; nos rumos, 1), a velocidade pedida (/5 m/s; 0: ficar onde esta) e o rumo pedido do corpo
    // em relacao a ele (cos, sen: no levantar em movimento, o alvo; nos rumos, o do comeco; no giro, o que gira).
    static constexpr std::size_t CommandSize = 6;
    // Depois do comando, o corpo contra o chao (o contrato v3, TrackingGetUpContract3D::observeGround).
    static constexpr std::size_t GroundObservationSize = TrackingGetUpContract3D::GroundObservationSize;
    // Depois do chao, a fase do clipe do passo (sen, cos de 2 pi t/T; zeros sem clipe): a entrada de fase do DeepMimic,
    // sem a qual a imitacao no tempo do clipe nao tem como sincronizar (01/11 18h, o layout levantar-v4).
    static constexpr std::size_t PhaseSize = 2;
    // Depois da fase, o ambiente (02/11, o layout levantar-v5): a altura de cada parte ao que esta embaixo dela, a
    // inclinacao do chao e os raios horizontais no quadril e no peito (TrackingGetUpContract3D::observeEnvironment).
    static constexpr std::size_t EnvironmentSize = TrackingGetUpContract3D::EnvironmentObservationSize;

    explicit TrackingGetUpAmpTraining3D(const TrackingGetUpAmpSettings3D& settings);
    ~TrackingGetUpAmpTraining3D();
    TrackingGetUpAmpTraining3D(const TrackingGetUpAmpTraining3D&) = delete;
    TrackingGetUpAmpTraining3D& operator=(const TrackingGetUpAmpTraining3D&) = delete;

    [[nodiscard]] std::size_t environmentCount() const;
    [[nodiscard]] std::size_t observationSize() const;
    [[nodiscard]] std::size_t actionSize() const;
    [[nodiscard]] std::size_t ampSize() const;
    [[nodiscard]] const std::vector<std::string>& observationNames() const;
    [[nodiscard]] const std::vector<std::string>& actionNames() const;
    [[nodiscard]] float episodeSeconds() const;
    // O minimo e o maximo (rad) de cada eixo da acao (2 por eixo).
    [[nodiscard]] std::vector<float> actionRanges() const;

    // As quedas da locomocao para os comecos do tipo 8: o corpo inteiro no instante da queda, com as velocidades (os de
    // outro numero de links ficam de fora). Troca so entre os steps.
    void setFallStarts(std::vector<std::vector<PhysicsBodyState3D>> states);
    // Quais mundos estao protegendo-se na queda agora (os comecos da queda iminente, ate o tronco tocar o chao ou ficar de
    // pe; 1 ou 0): o treinador alivia ai os custos de suavidade - o reflexo de protecao e rapido.
    void protecting(std::span<std::uint8_t> out) const;
    // Quais mundos estao num episodio de locomocao (os rumos e o giro; 1 ou 0): o treinador pesa ai a tarefa, o estilo e
    // a suavidade da locomocao (a rl_final).
    void locomoting(std::span<std::uint8_t> out) const;
    // A rede da locomocao parceira (vazia: sem). Troca so entre os steps.
    void setLocomotionPartner(std::shared_ptr<const LocomotionPartner3D> partner);
    [[nodiscard]] std::size_t fallStartCount() const;

    void reset(std::uint64_t seed);
    void observations(std::span<float> out) const;
    // A observacao do fim dos episodios que acabaram no ultimo passo (para o valor do corte).
    void finalObservations(std::span<float> out) const;
    // rewards: a nota da tarefa.
    void step(std::span<const float> actions, std::span<float> rewards, std::span<std::uint8_t> terminated,
        std::span<std::uint8_t> truncated);
    // O par do AMP do ultimo passo de cada mundo (antes e depois, 2 x ampSize), antes de um recomeco.
    void ampPairs(std::span<float> out) const;
    // Os pares dos clipes do estilo, de 1/120 em 1/120 s (2 x ampSize cada; o passo e o de uma decisao).
    [[nodiscard]] std::vector<float> referencePairs() const;
    [[nodiscard]] std::vector<TrackingGetUpAmpEpisode3D> takeEpisodes();

    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    // Os alvos de junta de agora de um mundo (os da ultima acao).
    [[nodiscard]] std::span<const RagdollDriveTarget3D> driveTargets(std::size_t world) const;
    [[nodiscard]] const RagdollProfile3D& profile() const;
    [[nodiscard]] std::string frameLines(std::size_t world) const;
    [[nodiscard]] TrackingGetUpAmpWorldView3D worldView(std::size_t world) const;
    // O cenario do mundo em arame (no mundo): as arestas das caixas (a rampa cortada em 2,5 m em volta do meio).
    void scenerySegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const;
    // Os corpos soltos do mundo (do mesmo perfil do corpo; validos ate o proximo passo).
    void sceneryBodies(std::size_t world, std::vector<const RagdollState3D*>& out) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
