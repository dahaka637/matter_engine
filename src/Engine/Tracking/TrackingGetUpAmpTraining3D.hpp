#pragma once

#include "Engine/Athlete/Body/LocomotionPartner3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Engine/Tracking/TrackingGetUp3D.hpp"

#include <cstdint>
#include <array>
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
    // Duas cabecas numa politica (a recuperacao e a locomocao): a ultima entrada diz qual age (1: a locomocao, de pe
    // com pedido de deslocamento).
    bool separateSkills = false;
    // O rastreio ruim sustentado encerra (o early termination por desvio): a media movel da nota (~0,5 s) abaixo disto,
    // depois de 1 s. 0: so a queda.
    float trackingTermination = 0.0f;
    // A forca auxiliar da locomocao (04/11 00h, a ordem dele: "forcar a forca auxiliar em 100% para os exercicios de
    // locomocao, assim ele sera obrigado a aprender"): com a cabeca da locomocao no rastreio de um clipe, a mao de fora
    // na pelve puxa a altura e a inclinacao/rumo da pelve do clipe e a velocidade pedida (PD; 1 = a mao inteira, 0 =
    // nenhuma). O treinador a afrouxa so com o dominio medido.
    float locomotionAssist = 0.0f;
    // (04/11 04h35) A mao linear de verdade: a 100% no maximo o peso do corpo (sustentar), metade dele (empurrar) e o
    // torque de tombar (peso x 1 m); a x%, x% disso; 0% nada. false: a mao de antes (cortada em 2000 N / 1500 N.m).
    bool linearHand = true;
    // (04/11 05h45) A mao tambem no tronco de cima: o peito alto, a cabeca e os bracos para a orientacao do clipe.
    bool upperHand = false;
    // (04/11 06h00) A mao em todos os ossos: a coluna, o pescoco, a cabeca, os bracos, as maos, as pernas e os pes.
    bool bodyHand = false;
    // O pe de equilibrio (04/11, ele: "usar o footplanner que ja temos na locomocao fisica"): com o clipe nos motores
    // (locomotionResidual), o pe do clipe no ar pousa deslocado pelo ponto de captura do corpo real contra o pedido -
    // o balanceFootPlacement do modo fisico - e o IK das pernas (synthesizeAthleteLegs3D) leva a perna ate la.
    bool balanceFeet = false;
    // A nota do rastreio e so a v19 desde 10/11; o campo ficou como o portao da inclinacao da referencia (leanSide, runPitch:
    // so com 10 ou mais). shortcut: o lote nasce com 1 e o treinador poe 19 no configurar - os comecos sorteados no construtor
    // saem sem a inclinacao, e o ouro da prova depende disso; tirar num passo proprio com ouro novo.
    int trackingReward = 1;
    // O parado em alerta (04/11 04h10): o peso da imitacao do clipe IDLE ALERTA na regua estrita (DeepMimic, suavidade 1),
    // no rumo do proprio corpo; 0 = como antes (o imitationWeight, se houver).
    float idleImitationWeight = 0.0f;
    // (04/11 04h50) O idle depois de ficar de pe (o IDLE ALERTA vira a referencia ao ficar de pe sem pedido) e o parado na
    // cabeca da locomocao (o clipe do idle nos motores e a correcao da rede; caido, a recuperacao).
    bool idleAfterStand = false;
    bool idleOnLocomotion = false;
    // O alcance da correcao da rede no idle da cabeca da locomocao (a fracao do de sempre).
    float idleResidualScale = 1.0f;
    // (04/11 09h00) A entrega ao idle depois de levantar: na hora, com os alvos dos motores e a mao indo ao idle neste tempo
    // (s; 0 = a de antes, com o corpo quieto IdleHandoffSeconds), e o alcance da correcao nesse tempo (a fracao do de sempre;
    // < 0: o do idle), que volta ao do idle junto.
    float idleBlendSeconds = 0.0f;
    // (05/11 02h15, ele: "ja deveria ter a emenda direta: assim que ele esta ficando de pe ja cortasse para o idle alerta... sem
    // esperar ficar de pe do levantar-se, mas o ficar em pe do idle alerta") A entrega na subida: depois de levantar do chao,
    // com os dois pes no chao, a cabeca acima disto (m), a pelve acima de RiseHandoffPelvis e o tronco acima de
    // RiseHandoffUpright, o idle assume subindo - o clipe do idle nos motores (com a transicao idleBlendSeconds) termina de
    // levanta-lo. 0: so de pe e sem subir, como antes.
    float riseHandoffHead = 0.0f;
    // (04/11 11h25) A naturalidade da recuperacao (o levantar e o evitar): a multa por decisao com toda a aspereza e todas as
    // juntas no limite (0 = sem).
    float recoveryNatural = 0.0f;
    // (04/11 12h) A mao linear empurra no plano no trote de lado (2 e 6) - false: como nas costas, que nao puxam desde 07h00
    // (medido: sem o puxao o trote de lado melhora - semelhanca 0,81 -> 0,86, pes 0,050 -> 0,027; a corrida de lado, sem ele,
    // ainda nao anda).
    bool lateralHandPush = true;
    // (04/11 12h, ele: "para a corrida lateral vamos manter a velocidade, de mais permissividade no movimento das pernas para
    // possibilitar movimentos mais amplos") Na corrida de lado (2 e 6), o alcance da correcao nas pernas vezes isto, e a
    // semelhanca das pernas e dos pes cobrada pela metade.
    float lateralRunLegReach = 1.0f;
    // (04/11 12h35, ele: "a velocidade do trote 4 m/s (todas direcoes); na corrida 6 m/s para frente (e diagonais frente),
    // 5,5 para tras e 4,5 lados - fazer ele aprender a pegar impulso, fazer forca contra o solo, sem tentar apenas aumentar a
    // cadencia") No rastreio, o pedido fixo por rumo do trote e da corrida (m/s; 0: o do clipe +-15%, como antes). Com ele o
    // clipe fica na cadencia dele (escala 1): a velocidade a mais tem de vir da passada e do empurrao.
    std::array<float, 8> trotSpeeds {};
    std::array<float, 8> runSpeeds {};
    // (04/11 13h40) A coluna da referencia na largada de costas, de pe (rad, < 0 para tras; 0: so a BackLean de sempre).
    float backStartLean = 0.0f;
    // (04/11 14h) Nas curvas (5-7): > 0, o rumo persegue o percurso desenhado e mais longe dele que isto (m) por 0,5 s
    // encerra como a queda ("fora do percurso"); 0, o rumo gira no relogio do plano, como antes.
    float courseLimit = 0.0f;
    // (04/11 14h05) Nas curvas, a referencia inclinada para dentro na inclinacao que equilibra a curva (atan(v x giro / g)).
    bool curveLean = false;
    // (05/11 20h10, ele: "a pisada do trote... ele pisa em linha reta... verifica se isso esta correto, pois e estranho": a
    // rede pisa com 2,7 cm entre os pes contra 18,6 do clipe.) > 0: nos rumos retos e no sprint, a largura entre os pes (de
    // lado, no rumo do corpo) diferente da do clipe derruba a semelhanca: exp(-(diferenca / isto)^2); 0: sem (m).
    float footWidthSlack = 0.0f;
    // (05/11 21h, ele: "atacar as trocas de direcao") Nas trocas que usam a meia-volta (media, forte e a livre grande), na
    // corrida: antes dela a freada ate o trote no mesmo rumo, a meia-volta trotando (a da corrida freava de 5 m/s a quase 0 em
    // 0,3 s, impossivel ate com a mao cheia: 11-20% de quedas por troca contra 0% no trote) e a corrida de volta 0,6 s depois.
    bool runTurnBrake = false;
    // (05/11 21h40) A meia-volta entra no pe certo: espera ate um ciclo o pe que o clipe planta primeiro pousar no instante
    // do plantio do clipe (36% de quedas no pe trocado contra 0% no certo).
    bool runTurnAlign = false;
    // (05/11 22h40, ele: a media nao para a inercia, a forte engata direto sem arrancada) A troca assentada: de costas, a
    // freada ate 1,5 m/s antes da meia-volta (todas as quedas medidas na forca de agora comecam de costas).
    bool turnSettle = false;
    // (05/11 23h20, ele: "a forte engata direto sem arrancada gradual... o clipe agachado tem boa base para arrancada em base
    // baixa") Na media e na forte, a saida da meia-volta pela arrancada agachada (crouched_to_sprint) e dela o passo.
    bool turnLaunch = false;
    // (07/11, o "Levantar correndo") No levantar em movimento, de pe e com o pedido de velocidade, quem dirige e a cabeca da
    // locomocao (a recompensa continua a do levantar em movimento: correr ate o alvo); sem, a recuperacao do comeco ao fim.
    bool movingHandoff = false;
    // (11/11 14h, ele: "Zerar absolutamente o brain do levantar-se... Serao 4 exercicios... pelos clipes do unreal engine") O
    // levantar pelo clipe (TrackingClipGetUp3D; vault 04_locomocao/plano_levantar_unreal_11_11.md): a fatia dos episodios (o
    // comeco 9, primeiro no sorteio), a dos que comecam no meio do clipe (o RSI do DeepMimic; o resto deitado no quadro 0 com a
    // coluna e as pernas variadas ate clipGetUpStartNoise rad), o tempo da locomocao depois da troca (cair nele encerra) e as
    // larguras da nota. Os clipes e as opcoes por setClipGetUp. Na avaliacao, 11 a 14: de costas, de brucos, correndo de costas,
    // correndo de brucos.
    float clipGetUpFraction = 0.0f;
    float clipGetUpMidStartFraction = 0.5f;
    float clipGetUpAfterSeconds = 1.5f;
    float clipGetUpStartNoise = 0.2f;
    float clipGetUpSoftness = 1.0f;
    float clipGetUpComVelocityWidth = 1.5f;
    bool clipGetUpParts = false;
    // (12/11 04h40, ele: a fase de fogo do levantar, "100% adaptativo, o clipe perde tanto rigor" - vault
    // 04_locomocao/plano_fogo_do_levantar_12_11.md) O levantar adaptativo: os exercicios do clipe sem o rastreio na nota - a do levantar
    // livre de 31/10 (o potencial da subida, o de pe, os custos; correndo, a velocidade ao rumo) e o juiz no estilo -, sem o longe e sem
    // queda que encerre ate o orcamento; a troca quando firme de pe (ou correndo). O clipe so da o comeco, o pedido e a regua.
    bool clipGetUpAdaptive = false;
    // (12/11 05h00, ele: "junto dela tambem vai continuar a recompensa por ele melhorar sua velocidade e agilidade para levantar") No
    // adaptativo, na entrega (so depois de firme - o bote nao paga): isto x a fracao do tempo do clipe ate o ponto de entrega que ele
    // poupou ((tempo do clipe - o dele) / tempo do clipe, de 0 a 1).
    float adaptiveSpeedBonus = 0.0f;
    // (12/11 13h45, ele: "normalizar os tempos de levantar limpos... de costas e de bruços 1-1,2 s, os correndo 1-1,2 s... nao queremos
    // mais rapido (para nao ficar irrealista), nem mais lento") Com adaptiveTimeMax > 0, o premio da velocidade vira o da faixa: inteiro
    // com o tempo da tentativa entre adaptiveTimeMin e adaptiveTimeMax (encolhidos pelo trecho do clipe que falta, na tentativa que
    // comeca no meio), caindo a zero em 0,5 s (escalados) fora dela - mais rapido ou mais lento.
    float adaptiveTimeMin = 0.0f, adaptiveTimeMax = 0.0f;
    // (13/11 07h45, ele: "o tempo de levantar, no de costas e de brucos em 1,3-1,4 s e os de levantar correndo em 1,2 s") A faixa dos
    // correndo a parte; com adaptiveTimeMaxRun 0, a de pe vale para todos.
    float adaptiveTimeMinRun = 0.0f, adaptiveTimeMaxRun = 0.0f;
    // (13/11 07h55, ele: a meta de 1,5 s "nem mais, nem menos") Em quantos segundos fora da faixa o premio dela zera (era fixo em 0,5 s; o de
    // costas a 0,85 s ficava 0,6 s abaixo de 1,45 e sem sinal nenhum).
    float adaptiveTimeFall = 0.5f;
    // (13/11 10h55, medido: o tempo para levantar no treino, limpo x sob fogo - costas 1,09 x 1,27 s, brucos 1,36 x 1,93, correndo
    // 1,11 x 1,41 e 1,07 x 1,91.) A faixa valia nos dois, e a rede, sem saber de antemao se vem pancada, ficava no meio: o fogo puxava
    // para mais rapido, o limpo para mais devagar, e dobrar o premio nao movia nada. Com isto >= 0, no levantar com fogo (fireKind != 0)
    // a faixa vira so um teto - o maximo dela mais esta folga (s) -, sem piso; o ritmo dele fica pelos limpos. < 0: a faixa nos dois.
    float adaptiveTimeFireSlack = -1.0f;
    // (13/11 11h30, ele: "essa cabeca e peito torto quando levanta" - o de costas. Medido na subida, cabeca a 0,9-1,5 m: a cabeca
    // 8-13 graus para tras da linha do tronco (o clipe: 20-39 a frente), tombada de lado 30-37 (o clipe 6-25), o peito torcido contra
    // os quadris 25-63 (o clipe 10-32); o de brucos perto do clipe. O adaptativo nao media nada disso e o juiz, saturado, nao via.)
    // A forma da cabeca e do peito: no levantar adaptativo com a recuperacao no comando e a cabeca acima de 0,6 m, um custo por passo
    // de peso x (1 - forma); a forma compara a cabeca contra o peito (para tras, de lado) o peito contra a pelve (torcao e, 12h25, arqueado para tras) e o olhar para cima com a media
    // do proprio clipe NA MESMA ALTURA DA CABECA (nao no tempo: o ritmo e a tecnica seguem dele), com folga. 0: nada.
    float getUpFormWeight = 0.0f;
    // (13/11 17h45, ele: "normalizar os tempos... o de costas levanta muito mais rapido que o de brucos... proximo de 1.5 segundos... o de
    // costas pelo menos 1.2") getUpPaceWeight: no levantar adaptativo limpo de pe (sem fogo, sem o correndo), ate a entrega, custa por
    // passo estar com a cabeca mais alta que a do clipe tocado no ritmo de getUpPaceSeconds ate a entrega (do instante do comeco da
    // tentativa): w x clamp((cabeca - alvo - 0,05 m) / 0,25 m, 0, 1). Mais baixo ou outro movimento nao custa - so subir antes da hora.
    // Medido (13/11, r110380): a rede pulava a pausa ajoelhada do clipe (~0,75 m) e ia de 0,45 a 1,5 m em 0,5 s. 0: desligado.
    float getUpPaceWeight = 0.0f, getUpPaceSeconds = 1.5f;
    // (13/11 13h30, ele: "sem melhoras nenhuma quanto a mao" - medido: 100% dos de costas ainda apoiam a esquerda virada depois de 45 min;
    // dentro da forma a mao custava ~2 por levantar, ~10x menos que a subida que mudou) A mao virada com peso proprio: por passo,
    // isto x (1 - nota da mao), fora da forma. 0: a mao dentro da nota da forma (metade dela com mao apoiada).
    float getUpPalmWeight = 0.0f;
    // (14/11 20h10, ele: "porque nao poderiamos usar o mesmo metodo que ja foi usado... para trazer a semelhanca com o clipe" - a perna
    // esquerda do de costas indo para tras no ajoelhar; o rastreio do corpo inteiro dilui a perna em 3 de 18 partes) A forma da perna
    // esquerda, como a da cabeca e do peito: no levantar adaptativo limpo (sem fogo) com a recuperacao no comando e a cabeca acima de
    // 0,6 m, nos exercicios da mascara (bit e = exercicio e), um custo por passo de peso x min(1,5, (erro - 5 cm) / 30 cm); o erro e a
    // media do joelho (a canela) e do pe esquerdos contra a pelve, nos eixos dela, contra a media do clipe na mesma altura da cabeca
    // (+-0,1 m) perto do relogio dele (+-0,4 s). 0: nada.
    float getUpLegWeight = 0.0f;
    std::uint32_t getUpLegMask = 0u;
    // (14/11 20h40, ele: "os bracos estabanados na arrancada, e os passinhos parado/curtos" - visto: o clipe sai inclinado 20-30 graus
    // com passadas longas e a rede fica com o tronco reto trotando no lugar; os bracos seguem o balanco do clipe num corpo reto) Na janela
    // da largada, de pe, um custo por passo de peso x min(1,5, (inclinacao do clipe - a do corpo - 5 graus) / 15 graus) x o peso da
    // janela: a inclinacao e a do peito alto contra a pelve, no rumo do movimento, contra a vertical; so o MENOS inclinado que o clipe
    // custa. 0: nada.
    float launchLeanWeight = 0.0f;
    // (14/11 22h10, o Codex consultado: "pagar a velocidade cedo fora dos multiplicadores de forma") Na largada da corrida de frente (a
    // faixa da corrida, rumos 0, 1 e 7), com a janela da largada como envelope: a nota do rastreio vira (1 - s x janela) x ela + s x janela
    // x p, p = a velocidade do centro de massa no rumo sobre max(1 m/s, a pedida agora) - ganhar velocidade cedo paga por si, mesmo com a
    // pose longe do clipe (medido: a rede ficava reta trotando no lugar, 1,0-1,2 m/s a 1 s). 0: nada.
    float launchSpeedShare = 0.0f;
    // (15/11 06h30, ele: "ainda falta a arrancada da corrida... ha clipe para isso, basta ele seguir o clipe da arrancada para correr
    // corretamente") Na largada de pe da corrida de frente (rumos 0, 1 e 7, o clipe de largada tocando - starting), a nota da
    // semelhanca do levantar (TrackingClipGetUp3D::score: a pose por partes, a linha do tronco, maos e pes, o centro de massa e a
    // velocidade dele, o pulo) contra o quadro do clipe no relogio dele, posto sob a pelve no rumo do clipe, com a velocidade do
    // centro de massa da referencia a da rampa do clipe (os clipes sao no lugar). Soma isto x a nota x a janela da largada. 0: nada.
    float launchImitationReward = 0.0f;
    // (13/11 14h, ele: "no treino do parado, gostaria de que o quadril dele ficasse retinho alinhado" - medido: no parado a linha dos
    // quadris caida -14,9 graus em media, acima de 5 graus 96% do tempo; o clipe 0; os pes 21 / 38 cm para os lados contra 31 / 31) No
    // parado (a locomocao no comando, sem pedido de movimento, de pe), um custo por passo de peso x (1 - e^-(inclinacao/6 graus)^2), a
    // inclinacao de lado da pelve nos eixos do corpo. 0: nada.
    float idlePelvisLevelWeight = 0.0f;
    // (13/11 07h45, ele: "reforcar melhor a postura nao estabanada e auto-equilibrio no pos levantar"; medido: de pe, 1,5 s depois de
    // levantar o centro de massa a 0,6-0,9 m/s, o peito girando 3 rad/s, ~4 passos e o ponto de captura fora da base 80% do tempo; 19-31%
    // caiam depois.) No levantar de pe, de pe com a recuperacao no comando (o firme e a janela de firmeza): a nota de ficar de pe vira
    // (1 - peso) x ela + peso x a calma (o centro de massa parado, o peito sem girar, o ponto de captura na base, os dois pes no chao). 0
    // desliga.
    float settleCalmWeight = 0.0f;
    // (13h45, ele: "bonifica consideravelmente o impulso tambem para pegada de velocidade pos levantar... quando ele esta quase de pe que
    // se impulsiona para frente") Nos correndo, na entrega: isto x a velocidade ao alvo de 60% (zero) a 100% (inteiro) do pedido.
    float runImpulseBonus = 0.0f;
    // (14h05, ele: "vamos premiar tambem a VELOCIDADE de locomocao do nosso levantar correndo pos-levantar... para ele engatar melhor o
    // impulso inicial de forma mais limpa e eficaz") Nos correndo, depois de ficar de pe e antes da troca: isto x a velocidade ao alvo
    // sobre a pedida (0-1) a cada decisao; e a janela da rede do levantar ate a troca, nos correndo (s; 0: a de sempre).
    float runSpeedReward = 0.0f, runSettleSeconds = 0.0f;
    // (14h30, ele: "sinto que esta faltando essa expertise nele para conseguir se levantar independente de sua posicao") No treino, a
    // fracao dos comecos deitados com a pose girada 60-180 graus em torno do comprido do corpo (de lado, virado); o clipe pelo lado em que
    // ele ficou, no trecho mais parecido (como na tentativa nova). Marcados no bit 128 das adversidades (fora do tempo limpo).
    float getUpPoseVariety = 0.0f;
    // (15h05, a analise das 15h00: sob fogo, 40-80% caiam depois de ficar de pe - na troca por tempo fixo a locomocao congelada pegava o
    // corpo ainda balancando; no limpo 3-10%) Com handoffCalmSpeed > 0, a troca depois da janela minima so quando calmo por 0,1 s (de
    // pe: o centro de massa abaixo disto, m/s; correndo: ereto, >= 80% do pedido ao alvo e < 0,6 m/s de lado), ou no maximo
    // handoffMaxSeconds depois da entrega (correndo, handoffMaxSecondsRun).
    float handoffCalmSpeed = 0.0f, handoffMaxSeconds = 1.5f, handoffMaxSecondsRun = 1.8f;
    // (12/11 07h50) No adaptativo, a firmeza depois da entrega: a rede do levantar segue no controle por isto (s) antes da troca - a
    // entrega (o sucesso) e a forca parando seguem no de pe da postura. Medido: so com o continuo, o de costas entregava 87% e caia
    // depois 64% - inclinado contra a forca que some em 0,15 s, passado na hora ao idle congelado. 0: a troca na entrega, como antes.
    float adaptiveSettleSeconds = 0.0f;
    // (12/11 07h55) O soltar da forca continua do fogo depois da entrega (s, linear ate zero). Com 0,15 s a rede de agora inclinada
    // contra 0,75 do peso caia de qualquer jeito - a janela de firmeza sozinha piorou o firme (28% -> 8-15%, sonda r88413).
    float fireReleaseSeconds = 0.15f;
    // (13/11 12h55, com a troca direta - adaptiveSettleSeconds 0 -: sob fogo, 42-51% dos de pe caiam depois da entrega, 68-85% deles
    // 0,4-1 s depois - inclinados contra o continuo que sumia ja com o idle no comando; limpos 0-4%.) Com isto, o soltar comeca na
    // primeira vez que ele fica firme de pe (a do riseFirmFrom), nao na entrega: com fireReleaseSeconds = o firme da entrega (0,2 s), a
    // forca acaba ainda com a rede do levantar e o idle recebe o corpo solto. (13h15) E nenhuma pancada nova depois do firme.
    bool fireReleaseAtFirm = false;
    // (12/11 08h20, ele: "tentar se levantar novamente, APOS fracassar nas tentativas anteriores (nunca desistir)") No adaptativo, o
    // fracasso abre uma nova tentativa: o relogio do clipe ja no ponto de entrega (ja devia estar de pe) e a cabeca abaixo de
    // RetryHeadMeters por RetrySeconds (a queda pela pancada, o continuo derrubando, ele mesmo caindo; 09h00: antes, a cabeca 0,6 m
    // abaixo da do clipe - nos correndo o clipe sobe rapido e a tentativa nova fracassava em 0,4 s, 7-19 tentativas sem levantar)
    // -> o relogio do clipe volta ao comeco, no clipe do
    // lado em que ele esta (de costas ou de brucos, do mesmo tipo: de pe ou correndo), reancorado no corpo, e o orcamento recomeca; o
    // fogo segue. Medido antes (r88413): deitado depois da queda, o relogio seguia ate o fim do clipe - a fase dizia "ja de pe" -, e a
    // recuperacao de verdade era ~0% (1 em 139).
    // (09h45, ele: "melhorar a velocidade de tentar levantar de novo apos fracassos ou instabilidades... bem reativo... nao precisar
    // esperar estar na pose tal para tentar levantar, mas se adaptar a cada cenario e situacao") A tentativa nova tambem quando ele
    // perde o que ganhou (a cabeca RetryDropMeters abaixo do ponto mais alto da tentativa e abaixo de RetryHeadMeters), por
    // RetrySeconds; o relogio vai ao trecho do clipe mais parecido com a pose de agora (nao ao comeco), e o premio da velocidade conta
    // o tempo da tentativa. (14h15) Tambem quando ele virou para o outro lado no chao (de costas num clipe de brucos e vice-versa).
    bool adaptiveRetry = false;
    // (12/11 08h45) Nos correndo, a parte da forca continua do fogo contra a corrida (o rumo ao alvo) limitada a isto x o peso; a parte
    // de lado e a favor seguem. Medido (r89140, por rumo): contra a corrida o correndo de brucos (0,64 do peso) levantava 0-5% e o de
    // costas (0,43) 4-33%, de lado e a favor 84-100% - correr a ~2,5 m/s contra 0,64 do peso e subir correndo uma rampa de ~40 graus.
    // 0: sem limite.
    float fireRunHeadwindCap = 0.0f;
    // (12/11 09h15, ele: "colocarmos paredes no laboratorio do RL exclusivamente para esses treinos de levantar, ~20-~50 m da area de
    // nascimento... tambem vai ajudar ele a aprender a se levantar quando esta sendo prensado contra a parede") No levantar pelo clipe,
    // quatro paredes em quadrado em volta do nascimento, a uma distancia sorteada por episodio entre estes dois (m); 0: sem.
    float getUpArenaMin = 0.0f, getUpArenaMax = 0.0f;
    // (12/11 12h30, a fase 2 - ele: "REFORCAR os desafios (... incluindo novas, ... 30-50% de sucesso)"; o cartao em
    // plano_fogo_do_levantar_12_11.md) As adversidades novas do fogo do levantar, fora da prova; 0 / 1: desligadas.
    // fireShoveSeries: ate quantas pancadas por episodio, sempre antes de ficar de pe (a regra dele das 06h20 segue);
    // fireLegShare: a fracao das pancadas nas pernas (a coxa, a canela ou o pe: a rasteira);
    // fireWallShare (+ fireWallMin/Max, m): a fracao dos episodios com o continuo com uma parede do lado para onde ele empurra (o
    // prensado; nos correndo, so quando ela nao fecha o rumo da corrida);
    // fireTurnShare (+ fireTurnSeconds): a fracao dos episodios com o continuo em que ele troca de rumo (+-45-135 graus) a cada ~isto s.
    // fireAfterStandShare (12h45, ele: as pancadas tambem logo depois de ficar de pe, "desde que para fins estatisticos o sucesso
    // obtido conste e ela aprenda melhor com isso"): a fracao dos episodios com pancada que leva mais uma 0,1-0,4 s depois da entrega -
    // na janela de firmeza (a rede do levantar no controle); o sucesso segue contado no de pe, o "firme" diz se ele aguentou.
    int fireShoveSeries = 1;
    float fireLegShare = 0.0f, fireWallShare = 0.0f, fireWallMin = 0.5f, fireWallMax = 3.0f, fireTurnShare = 0.0f, fireTurnSeconds = 1.5f;
    float fireAfterStandShare = 0.0f;
    // (12h50) fireSlipShare: a fracao dos episodios com fogo sobre a grama molhada (uma placa de 20 x 20 m em volta do nascimento; o
    // treino e no concreto); fireGrabShare + fireGrabFraction: a fracao dos episodios com fogo com um agarrao - uma forca de isto x o
    // peso puxando uma mao (o adversario segurando), 0,5-1,5 s, antes de ficar de pe.
    float fireSlipShare = 0.0f, fireGrabShare = 0.0f, fireGrabFraction = 0.3f;
    // (13h05, ele: "o objetivo... baixar o sucesso para 30-50%"; os de pe ficavam em ~85% com as pancadas no teto - levantavam de novo
    // em ~1 s entre uma e outra) O intervalo da serie: a pancada seguinte entre estes dois (s) depois do fim da anterior.
    float fireSeriesGapMin = 0.4f, fireSeriesGapMax = 1.2f;
    // (13h30, ele: "dar uma normalizada/suavizada nas pancadas, pois elas estao muito fortes (jogando o boneco de um lado para o
    // outro)") A duracao da pancada fora da prova (s): o mesmo empurrao (m/s ao centro de massa) com menos pico de forca.
    float fireShoveSecondsMin = 0.1f, fireShoveSecondsMax = 0.2f;
    // (13h40, ele: "da para deixar as medias de 350-800, ficaria melhor") A forca da pancada fora da prova sorteada entre estes dois (N),
    // igual nos quatro exercicios, no lugar do nivel em m/s; 0: pelo nivel.
    float fireShoveNewtonsMin = 0.0f, fireShoveNewtonsMax = 0.0f;
    // (13/11 09h35, ele: "as pancadas secas do fogo deveriam ocorrer mais cedo, entre 0s - 1s que e o periodo que de fato ele esta
    // levantando ainda (nao que nao possa ser depois, mas a maioria)") fireEarlyShare: a fracao dos episodios em que a primeira pancada vem
    // pelo tempo, de 0,05 s a fireEarlyWindow; o resto pela altura da cabeca (o meio ou o quase fim, meio a meio). 0: o sorteio de antes
    // (um terco cada, o tempo de 0,15 a 0,6 s). Medido antes (r105700): 32-62% das pancadas no primeiro segundo.
    float fireEarlyShare = 0.0f, fireEarlyWindow = 1.0f;
    // (11/11 16h25) O peso de cada exercicio no sorteio do treino (de costas, de brucos, correndo de costas, correndo de brucos) -
    // o Python poe pela dificuldade (--levantar-unreal-prioridade).
    // (Vazio: os exercicios iguais.)
    std::vector<float> clipGetUpWeights;
    // (06/11 04h40, ele: na corrida de frente "comecasse com a velocidade progressiva... so no comecinho, para evitar esse pulo
    // que ele da no comeco para comecar a correr... apos o clipe de pegar impulso") A largada progressiva da corrida: com o
    // pedido fixo, o impulso (idle_to_sprint) nesta escala (0: 1, como antes); o ciclo entra na cadencia em que ele acaba e
    // sobe ate a cheia em runLaunchEaseSeconds; e o pedido sobe dali a runLaunchAcceleration (0: StartAcceleration).
    float runLaunchScale = 0.0f, runLaunchEaseSeconds = 0.0f, runLaunchAcceleration = 0.0f;
    // (06/11 04h55) A cauda do rumo: o fator "reto" da nota da velocidade com esta fracao numa regua larga (0,6 rad) - so a
    // estreita (CourseSlack 0,10 rad) zerava acima de ~12 graus e a largada torta de 20-60 graus sem a mao nao tinha
    // gradiente para endireitar. 0: so a estreita, como antes.
    float courseTail = 0.0f;
    // (08/11 18h45) O rumo da nota numa velocidade suavizada mais longa (s; 0: a de 0,25 s de sempre).
    float courseSmoothing = 0.0f;
    // (20h05) Nos retos, o deslocamento fora do rumo pedido mais que isto (rad; 0: nao) por StraySeconds encerra como a queda.
    float courseAngleLimit = 0.0f;
    // (08/11 23h55) Os potenciais do levantar (Ng 1999) tambem nos passos da cabeca da locomocao. De pe valem 24, e
    // 0,99 x 24 - 24 cobrava 0,24 por passo de qualquer corrida (a nota do trote lateral e 0,18). false: sem os
    // potenciais ali, e a queda de pe custa 2x FallPenalty, o que o potencial cobrava junto na queda.
    bool locomotionShaping = true;
    // (23h55) A velocidade manda: andando, a nota vezes speedGate + (1 - speedGate) x a nota da velocidade (0: nao).
    // Num empurrao e ate 1,5 s depois, nunca abaixo de SpeedGatePushed.
    float speedGate = 0.0f;
    // (09/11 01h20) O joelho para dentro da linha quadril-tornozelo alem do clipe (+ 2 cm de margem) derruba a semelhanca:
    // exp(-(excesso / kneeInwardSlack)^2), andando. 0: nao.
    float kneeInwardSlack = 0.0f;
    // (09/11 03h40) No idle da cabeca da locomocao, a pelve abaixo da do idle alem de 1,5 cm derruba a semelhanca da pose:
    // exp(-(excesso / idleHeightSlack)^2). 0: nao.
    float idleHeightSlack = 0.0f;
    // (09/11 03h50) Na parada final, o idle so com o corpo tambem abaixo de StopBodySpeed (ou StopBodyTimeout depois da ordem).
    bool stopWaitBody = false;
    // (07/11, a receita nova) A faixa da velocidade: no treino, o pedido fixo do rumo vezes um sorteio por episodio em
    // [speedRangeLow, 1] - a rede treina em pedidos alcancaveis e aprende a velocidade como um controle continuo; a
    // avaliacao sempre na meta. 1: o pedido fixo, como antes.
    float speedRangeLow = 1.0f;
    // (07/11, ele: "no Parado, seria interessante que a quietude fosse recompensada, sem afetar e claro o equilibrio") A
    // quietude do parado: o custo das juntas alem da folga (o respirar do idle) e do sobe-desce do centro de massa; o peso
    // dela na nota do parado sobe de 0,15 para 0,25. 0: a quietude de antes.
    float idleQuiet = 0.0f;
    // (07/11, a sonda das trocas: a forte acabava virada - 10 de 24 - ou caida, a media caida em 15 de 24, logo depois da troca;
    // o pedido freava e reacelerava a 6 m/s^2, nivel de atleta de elite sem girar o peito.) A aceleracao da velocidade de
    // referencia nas manobras, m/s^2 (era a constante ManeuverAcceleration).
    float maneuverAcceleration = 6.0f;
    // (07/11, ele: "tentar fazer o nosso levantar nao dar esse pulinho no final") O custo de chegar de pe subindo rapido: perto
    // do topo (o centro de massa acima de RiseTopHeight), k (vz - RiseTopSpeed)^2 por decisao, com a recuperacao no comando.
    float riseOvershoot = 0.0f;
    // (06/11 06h) A cauda da inclinacao de lado: o fator da semelhanca pela inclinacao do peito contra a do clipe com esta
    // fracao numa regua larga (0,35 rad) - so a estreita (RollSlack 0,08 rad) zerava acima de ~10 graus e na curva de corrida
    // sem a mao a rede inclinada 12-28 graus para FORA nao tinha gradiente para inclinar para dentro. 0: como antes.
    float rollTail = 0.0f;
    // (06/11 07h, a investigacao: "tirar os penhascos da recompensa") Sem os penhascos: a inclinacao de lado cobrada tambem
    // nas curvas (5-7), contra a do clipe (que inclina para dentro), numa regua larga (0,35 rad) - nas curvas ela nao era
    // cobrada e a rede corria inclinada para FORA; o antebraco ("Forearm") nos bracos - caia no tronco e o cotovelo era
    // cobrado na regua estreita da coluna; e uma cauda larga (1/4) no pulo da pelve (morria a 7 cm - o salto da arrancada
    // voltou: 13 cm custavam o mesmo que 7) e no pe cruzado da corrida (morria a 5 cm). Desligado: como antes.
    bool cliffFree = false;
    // (06/11 08h15, o A/B: as partes em separado - a inclinacao nas curvas e o pe cruzado ajudaram as curvas e a corrida
    // 0/1/7; a cauda do pulo PIOROU o salto da arrancada (70 -> 95%: amaciou a punicao dos pulos grandes) e o antebraco e o
    // pulo, que valem em todos, sao os suspeitos da piora nos trotes 3/5 e nas corridas 2/6.)
    bool cliffCurveRoll = false, cliffFootCross = false, cliffForearm = false, cliffHop = false;
    // (06/11 08h25, o salto da arrancada voltou - 20% -> 70% das largadas com a pelve acima de 0,92 m em 2 h: o pulo da pelve
    // na nota satura (um de 13 cm custa o mesmo que um de 7) e na arrancada a velocidade pesa mais) Na janela da arrancada da
    // corrida, a nota cai em linha reta com a pelve acima da do clipe alem de 3 cm, ate 60% a launchRiseSpan alem; 0: sem.
    float launchRiseSpan = 0.0f;
    // (08/11, ele: "sem semelhanca alguma com os clipes... sem atender a velocidade": a 0% o relogio do clipe nunca esta na
    // fase do passo do corpo - a semelhanca no relogio ~0 em toda rede, a professora inclusive - e as comparacoes com o
    // clipe no relogio (a pose, o pulo da pelve, a inclinacao, o giro) cobram fase em vez de forma: o passo com voo, fora
    // de fase, paga como pulo. Ligado: no pouso do pe esquerdo nos rumos retos, a fase do clipe volta ao pouso esquerdo dele.)
    bool phaseSync = false;
    // (08/11 03h45: a imitacao da v18 morta longe do clipe - as pernas a 1,24 pagavam 0,30 exp(-3,7) = 0,007, a coluna e a
    // cabeca ~0; so os pes tinham inclinacao. Andando, os expoentes da imitacao divididos por isto - a regua larga do
    // comeco -; parado e no giro, como antes. 1: sem.)
    float imitationLoosen = 1.0f;
    // (08/11 05h15: nas trocas a falha e o peito virar - 14-29% das manobras acabam "viradas" (o peito a mais de 100 graus
    // da vista por 0,5 s), quase nenhuma cai - e so o encerramento cobrava: nenhum sinal antes. Andando, a nota vezes
    // 1 - g + g exp(-(excesso / 0,6 rad)^2), o excesso o rumo da pelve contra o do clipe alem de 20 graus; 0: sem.)
    float facingGate = 0.0f;
    // (08/11 18h40) O peito para a vista nos diagonais retos: o portao do peito alto do corpo contra o da referencia.
    float diagonalChestGate = 0.0f;
    // (08/11 08h40, ele: "PRINCIPALMENTE corrigir esses movimentos zuados dos bracos": na locomocao livre os bracos viraram
    // ferramenta de equilibrio - a mao sobe a cabeca uma vez por passada. >= 0: andando, com a rede no volante e um clipe
    // seguido, os motores dos bracos recebem o alvo e a velocidade do clipe (na fase do passo, com a fase presa) mais a
    // correcao da rede, isto vezes (saida - meio); < 0: livres.)
    float armClipReach = -1.0f;
    // (08/11 10h40) O mesmo para o pescoco e a cabeca (< 0: livres).
    float headClipReach = -1.0f;
    // (06/11 08h50, a vigilia: o trote lateral a 0% cai na largada - em 1,2 s, o corpo girando, 0,6 m/s contra 4 pedidos -: de
    // pe a 4 m/s de lado em 0,8 s sem a mao ele nao arranca) A largada dos rumos de lado (2 e 6, trote e corrida) a esta
    // aceleracao (m/s2; a rampa dura o pedido / ela); 0: a de todos, StartAcceleration (5).
    float lateralStartAcceleration = 0.0f;
    // (06/11 09h40, a vigilia: o levantar a 0% nem sai do chao - o clipe puro sem a mao ergue a pelve so a 0,42 m (com a mao, a
    // 0,98): os motores do rastreio seguem o clipe com 20% da rigidez (TrackingStiffnessScale) e a rede so desvia um pouco os
    // alvos. A rigidez dos motores no levantar guiado (a escala; 0: a do rastreio, 0,2) - o musculo, nao uma forca de fora.)
    float riseStiffness = 0.0f;
    // (06/11, a locomocao livre: sem o clipe nos motores a velocidade-alvo era 0 e o amortecimento freava a passada inteira -
    // a professora no modo livre de pe so 8-24% no trote de frente, contra 100% com o clipe. Com isto, na cabeca da
    // locomocao a velocidade-alvo dos motores e a do proprio alvo da rede - (alvo de agora - o da decisao anterior) / dt -,
    // como o clipe dava a dele; sem clipe nenhum.)
    bool freeVelocityFeedforward = false;
    // (06/11 17h25) A escala do amortecimento dos motores da cabeca da locomocao sem clipe (0: a do rastreio, 0,5).
    float freeDampingScale = 0.0f;
    // (06/11 17h40, ele: "liberdade total nos motores") A rede da locomocao livre da tambem a velocidade-alvo de cada eixo:
    // a acao ganha, depois dos canais do passo, um eixo de velocidade por eixo (rad/s, ate VelocityActionLimit). Sem ela, a
    // velocidade-alvo era 0 e nem a copia perfeita da professora andava (de pe 0,83 -> 0,25): o clipe dava a dele.
    bool velocityActions = false;
    // (06/11 17h55) Com velocityActions, a cabeca da recuperacao tambem usa as velocidades da acao (o levantar).
    bool recoveryVelocity = false;
    // (06/11 22h10) Os pesos do sorteio das 7 manobras no treino (troca suave, media, forte, livre, curva suave, S, fechada).
    std::array<float, 7> maneuverWeights { 0.12f, 0.16f, 0.16f, 0.16f, 0.12f, 0.14f, 0.14f };
    // (06/11 10h50, a vigilia: as trocas media/forte paradas em ~15% ha 2 h - no atlas, a meia-volta saindo de LADO cai 30-69%
    // no trote e 100% correndo (de frente 5-23%): de lado ela entra a 4 m/s, a do trote) De lado (2 e 6), a freada ate esta
    // velocidade antes da meia-volta (m/s; 0: sem - em 05/11 a 1,5 m/s, na forca de entao, derrubava no passo lateral).
    float turnSettleLateralSpeed = 0.0f;
    // (09/11 04h30) De frente (0, 1, 7), a freada ate esta velocidade antes da meia-volta das trocas; 0: sem.
    float turnSettleFrontSpeed = 0.0f;
    // (09/11 04h35) A meia-volta das trocas tocada nesta fracao da velocidade do clipe (1: como ele; 0,6: 1,7x mais longa).
    float runTurnTimeScale = 1.0f;
    // (09/11 05h) Na parada final, a passada que desacelera (a cadencia com a velocidade, o idle abaixo de 0,5 m/s) em todos os
    // rumos - antes so nos laterais.
    bool stopGaitAll = false;
    // (09/11 06h15) No idle da cabeca da locomocao, fora do empurrao, a nota inteira vezes 1 - g + g x a pose (a altura e o peito alto
    // contra os do idle: 6 graus de folga, largura 10). 0: nao.
    float idlePoseGate = 0.0f;
    // (09/11 08h55) Com stopClips, os clipes de parada so de frente e nas diagonais da frente (de costas e de lado, a freada antiga).
    bool stopClipsFront = false;
    // (09/11 10h10) Com stopClips, o soltar da tecla do comando tambem pelo clipe de parada (nos rumos em que ele vale).
    bool stopClipsCommand = false;
    // (09/11 10h50) Freando (a parada final e o soltar da tecla), o portao da velocidade nunca abaixo disto; 0: como andando.
    float stopGateFloor = 0.0f;
    // (09/11 14h, ele: "nao acho que 'seguir' clipe vai ser uma boa ideia para frear, pois e algo relacionado a recuperacao
    // tambem.. e isso precisa ser emergente do RL") A parada livre: na parada final dos retos e no soltar da tecla do comando, a
    // cabeca da locomocao dirige ate ficar parada de pe (a recuperacao so caido ou baixo) e a nota e o objetivo - a velocidade a
    // zero, de pe, o peito na vista, a qualidade do parado -, sem referencia que freia, clipe de parada nem pes presos ao idle.
    bool freeStop = false;
    // (10/11 04h, ele: "conseguiu a freada, mas sacrificou a postura") A postura na parada livre: a nota inteira da parada vezes
    // 1 - G + G x pose, pose = o peito alto e a cabeca contra os do idle (folgas 5,7 e 8,6 graus), do comeco ao fim da parada. Sem
    // ela o tronco ia livre ate 20 graus e a cabeca da locomocao - a mesma do trote e da corrida - dobrou em tudo (diario 42). 0: sem.
    float stopPosture = 0.0f;
    // (11/11 03h29, ele: "ao frear fica todo molengo e nunca volta a pose do idle alerta"; "a pose deve ser cobrada sempre, mas
    // sem prejudicar o auto-equilibrio". Medido no codigo: parado, a nota da parada livre e 0,70 + 0,30*stand - 70% so por ter
    // parado, a pose do idle 6%.) E5-7: conforme ele para (near), a nota passa a ser a pose do idle alerta na regua estrita
    // (o DeepMimic inteiro, a coluna junta a junta inclusive): (1 - k*near)*nota + k*near*face*(0,25 + 0,75*pose),
    // pose = metade a semelhanca estrita e metade a cauda longa por junta. 0: sem.
    float stopIdlePose = 0.0f;
    // (11/11, ele: "a pose deve ser cobrada sempre, sem prejudicar o auto-equilibrio"; o plano 04_locomocao/plano_solucoes_11_11.md)
    // O parado unico: no parado (o exercicio e o de depois de frear, com o parkStop), a nota e pose x firmeza - a pose na regua de
    // cauda longa que nao satura (a forma, cada junta contra o idle alerta, e o lugar: a base dos pes, o giro da pelve e do peito
    // para a vista, a inclinacao do peito e a altura), a firmeza 0,5 + 0,5 x equilibrio x quietude -, sem as janelas que
    // dispensavam a pose (depois do empurrao e ao assentar). E o parado de depois de frear conta como parado no treinador
    // (locomoting = 0: os pesos e o juiz do parado). false: como antes.
    bool idleUnified = false;
    // (11/11 08h40, ele: "nao existe mais freio, agora e locomocao -> pose IDLE alerta, direto... se ele para a locomocao vai para
    // o idle alerta e pronto") stopToIdle: a ordem de parar (o fim dos retos, a tecla solta no comando) poe o alvo direto no idle
    // alerta na vista (parkStop / commandIdle) - sem clipe de parada e sem parada livre.
    bool stopToIdle = false;
    // (11/11 09h28, ele: "correndo -> idle alerta, mas fazendo de forma inteligente a recuperacao necessaria para nao cair... ele ja
    // estava conseguindo frear antes"; o stopToIdle tirou a freada que ele sabia e ele passou a cair.) idleAfterStop: a freada fica
    // como era (a parada livre, com os clipes de frente) e, freado - abaixo de FreeStopSettleSpeed -, ele entra no idle alerta
    // (parkStop), com a nota do parado. false: como antes.
    bool idleAfterStop = false;
    // (12/11 16h, vault 04_locomocao/plano_fogo_da_locomocao_12_11.md) parkedOnLocomotion: o idle estacionado depois de uma parada
    // (parkStop, stopPhase 2, o idle alerta seguido) fica na cabeca da locomocao. Sem isso, com o pedido zero, a escolha da cabeca
    // passava o controle a recuperacao 1-2 s depois da ordem de parar (medido na suite: 100% -> 0% da locomocao) e o parado unico
    // depois de frear nunca rodava nela. Caido ou baixo, segue a recuperacao. false: como antes.
    bool parkedOnLocomotion = false;
    // (12/11 16h30, o fogo da locomocao v1 - vault 04_locomocao/plano_fogo_da_locomocao_12_11.md; ele: "a implementacao da fase de
    // FOGO, igual nos fizemos [no levantar], ate as mesmas caracteristicas e visual... nesse primeiro momento bem mais SUTIL... a meta e
    // que no final consigamos trazer exatamente o mesmo tipo e niveis de forca que o levantar ja sofre"; "criar seus proprios movimentos
    // sem amarras para auto-equilibrio mediante as adversidades... sem sacrificar movimentos LIMPOS"):
    //  - locoFireShare: a fatia dos episodios de locomocao do treino com fogo (rumos, manobras, estrela, giro e parado; o comando nao,
    //    a ordem dele de 08/11). Na avaliacao, so com locoFireExam (a prova) ou fireShown (o laboratorio, como o treino).
    //  - locoFireLevels: o nivel L (0-1) de cada grupo (fireGroup: frente, lados, costas, giro, parado, o resto). Tudo sai de L, de
    //    sutil (L = 0) ao fogo do levantar de hoje (L = 1): o continuo 0,05 -> 0,5 do peso, a pancada seca 80-180 -> 350-800 N
    //    (fireShoveNewtons*, por fireShoveSeconds*), de 1 a fireShoveSeries pancadas por serie (fireSeriesGap* entre elas), o agarrao
    //    0,08 -> fireGrabFraction do peso. Como no levantar: a rasteira (fireLegShare), o vento que vira (fireTurnShare, fireTurnSeconds),
    //    a grama molhada (fireSlipShare, crescendo com L de 40% a 100% dela) e, so no parado e no giro, o prensado (fireWallShare).
    //    Contra a corrida, no maximo fireRunHeadwindCap do peso. As adversidades vem uma a uma (o continuo, a serie de pancadas, o
    //    agarrao; metade dos continuos com uma serie dentro), da primeira em 1-2,5 s (parado 0,5-1,5 s) a outra 1,5-3,5 s depois.
    //  - locoFireRelief: a liberdade sob o fogo (o rework). A perturbacao p de agora (a forca externa / 0,2 do peso, caindo a zero em
    //    1,5 s depois de cada pancada, agarrao ou virada do vento; 0,5 na grama molhada) vezes isto e: a janela de improviso da nota
    //    andando (a imitacao mais larga e com menos peso, mais peso a velocidade e ao equilibrio, mais folga na inclinacao); a firmeza
    //    sem o tropeco do passo rapido nem o giro alem do clipe; a inclinacao contada sem a da forca continua (a vertical efetiva); no
    //    parado, a quietude, o deslize e os passos sem custo e a pose mais solta; o virado com 1 s a mais. Enquanto houver forca, as
    //    janelas do empurrao (o rumo, o lento, fora do percurso, o idle perdido, o portao da velocidade) ficam abertas. 0: sem.
    //  - locoFireExam / locoFireExamLevel: a prova (na avaliacao) - uma adversidade por mundo k, a partir de 2 s: (k / 8) % 4 = 0 o
    //    continuo por 4 s, 1 a serie de 3 pancadas, 2 o agarrao por 1 s, 3 a grama molhada e uma pancada; o lado (do corpo) por k % 4:
    //    frente, tras, esquerda, direita; tudo no nivel locoFireExamLevel (1: o fogo do levantar).
    // (12/11 17h, ele: o parado "nascesse em uma posicao torta propositalmente, e um pouco mais alto... aprender justamente a se
    // REAJUSTAR (o mais rapido que ele conseguir) e ficar firme") crookedStartShare: a fatia dos parados do treino que nascem tortos
    // (crookPose: cada parte girada ao acaso em torno da junta e o corpo 3-8 cm acima do chao); crookedStartBonus: o premio de reajustar
    // (o lugar - a base, o giro, o peito na vertical, a altura - >= crookedFixClose, a coluna alinhada >= 0,6 e quieto por 0,3 s), cheio no
    // comeco e zero em 2,5 s.
    float crookedStartShare = 0.0f, crookedStartBonus = 0.0f, crookedFixClose = 0.8f;
    // (12/11 20h15) O parado torto sem o fogo da locomocao: ele e um exercicio de postura (reajustar e ficar no idle); com o fogo em 90%
    // dos episodios a postura do parado limpo se desfez em 15 min (a semelhanca 0,61 -> 0,51, o reajuste 37% -> 12%).
    bool crookedNoFire = false;
    // (12/11 20h45, a revisao com o Codex) O premio da rapidez do reajuste do torto zera em crookedFixSpan s: em 2,5 s, com o reajuste
    // tipico em 2,7-3,6 s, ele quase nunca pagava e o reajuste ficou sem sinal (37-50% -> 16-25% em 10 min, o parado normal estavel).
    float crookedFixSpan = 2.5f;
    // (12/11 17h20, ele, a estrela de 8 pontas: "tirar o encerramento automatico do exercicio por distanciamento do alvo... ao inves disso
    // seria interessante recompensar o nao distanciamento" e "nas intercalacoes dos alvos da estrela, botar alguns com a obrigatoriedade
    // de fazer curva, podendo ser mais aberta ou mais fechada... sinalizando adequadamente a curva") starPathWeight > 0: sair do caminho
    // da perna nao encerra mais; a nota andando vezes 1 - w + w / (1 + (d / starPathWidth)^2), d a distancia ao caminho (fora da passagem
    // pela ponta e da meia-volta); a chegada a ponta pede estar a ate 1,5 m dela ou do caminho. starCurveShare: a fatia das pernas que sao
    // um arco (a flecha entre starCurveMin e starCurveMax da corda, para um lado sorteado); o pedido segue a tangente do arco.
    float starPathWeight = 0.0f, starPathWidth = 1.2f, starCurveShare = 0.0f, starCurveMin = 0.15f, starCurveMax = 0.45f;
    // (12/11 18h, ele: no COMANDO "a conexao entre os dois brains, o da locomocao e do levantar... em perfeita harmonia") commandGetUp:
    // caiu no comando, a cabeca do levantar levanta (o clipe de pe do lado do corpo, no trecho da pose, adaptativo, sem fogo) e, de pe e
    // firme, devolve a locomocao no idle alerta com o jogador parado; a queda nao encerra (a multa fica). false: como antes.
    bool commandGetUp = false;
    // (13/11 17h, ele: "os treinos nao encerrarem so porque houve a queda... aprender com as falhas... como cair melhor, se recuperar
    // melhor"; o cerebro unico) fallToCommand: com commandGetUp, a queda num exercicio de locomocao (os rumos, as manobras, a estrela,
    // o giro) nao encerra - o episodio vira comando no instante da queda (o jogador emulado com a tecla e o shift do que ele fazia) e
    // levanta como no comando; firme, o jogador retoma. O episodio segue contando no grupo de antes (sceneryPreset = a manobra
    // original). false: como antes.
    bool fallToCommand = false;
    // (12/11 18h, ele: "levantar correndo... quando ele ta caido mas quer correr") A fracao das quedas andando em que o jogador segue
    // segurando a tecla: o levantar correndo, e a troca ja correndo na frente da vista.
    float commandRunGetUpShare = 0.5f;
    // (12/11 18h40, ele: na troca do levantar correndo "o correto e assim que ele comeca ficar de pe ja trocar para a locomocao.. nunca
    // deixar o brain do levantar ser usado para a locomocao em si" e "cortar o exercicio... pegar os primeiros dois passos so apos
    // levantar com sucesso e pronto") runHandoffAtStand: no correndo a entrega e a troca no de pe (cabeca acima de 1,3 m, tronco a
    // menos de ~45 graus, 0,2 s) sem exigir a velocidade e sem a janela depois; runAfterSeconds (>= 0) o tempo da locomocao depois
    // da troca no exercicio do correndo (os dois passos), no lugar do clipGetUpAfterSeconds. false / -1: como antes.
    bool runHandoffAtStand = false;
    float runAfterSeconds = -1.0f;
    // (12/11 19h10, ele: "um exercicio que treina focado na TROCA de brains... inicia caido, de costas ou brucos, e precisa levantar,
    // haver a troca de brain para o da locomocao e ele ir ate um ponto proximo se locomovendo, marcado no mapa, coisa de 5 - 10
    // metros... aleatorio se vai ser de costas ou brucos, e se vai ser normal ou correndo (se for correndo, ai a locomocao tambem e
    // correndo)") getUpGoShare: a fracao dos levantares pelo clipe que sao o "levantar e ir" - deitado no quadro 0, sem o fogo do
    // levantar, o ponto a getUpGoMin-Max m para onde o clipe termina virado (de pe, +-60 graus) ou correndo (+-20); depois da troca
    // a locomocao vai ate ele (o trote no de pe, a corrida no correndo), o rumo virando para o ponto; chegar a 1 m e o sucesso e
    // encerra. Na avaliacao, com >= 1: todos, e o exercicio sorteado. No registro o comeco vira 11. 0: como antes.
    float getUpGoShare = 0.0f, getUpGoMin = 5.0f, getUpGoMax = 10.0f;
    float locoFireShare = 0.0f;
    std::array<float, 6> locoFireLevels { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    float locoFireRelief = 0.0f;
    // (12/11 20h, a revisao do fogo) A escala do intervalo depois de cada evento do fogo da locomocao (1: 1,5-3,5 s andando, 1,0-2,5 s
    // parado). Com o evento seguinte logo depois, a forca estava la quase o episodio todo e a rede andava sempre travada - a velocidade e a
    // postura do limpo cairam; com intervalos maiores o certo e reagir.
    float locoFireGapScale = 1.0f;
    bool locoFireExam = false;
    float locoFireExamLevel = 1.0f;
    // (11/11 09h50, ele: "o treino do giro parado esta errado: deveria fazer giros fixos de tipo 60, 90, 180, 360 graus, em
    // velocidades variadas, inversao de giro, as vezes gira, espera e ai gira de novo; e recompensar ele melhor por manter a coluna
    // toda reta") turnVariety: cada giro e 60, 90, 180 ou 360 graus, na velocidade do clipe x 0,6-1,4, o lado sorteado de novo, e
    // 30% das vezes antes dele uma pausa de 0,5-2 s no idle alerta. turnSpineReward: no giro, o bonus da coluna alinhada - a cadeia
    // Pelvis-Abdomen-Chest-UpperChest-Neck, cada junta 1/(1+(angulo/5 graus)^2), a media (spineAlignment) - vezes isto, somado a
    // nota; 09h55, ele: "no parado tambem" - no parado ela entra no bonus da nota com este peso.
    bool turnVariety = false;
    // (11/11 10h30, ele: "faca... em definitivo... o que for necessario... isso vale para o girar parado tambem"; a receita do parado
    // que ja ficou imovel, o RL v1 de 12/10 - os pes parados no milimetro, 0,5 cm de oscilacao, o tronco a 3 graus/s -: custos de
    // quietude densos e sem saturar.) stillCost: no parado (com a nota de idleRewardPose) os custos - o giro do tronco e da pelve
    // min(w2, 4)/4, cada pe apoiado escorregando ou girando (v + 0,1 giro vertical, ate 0,2 m/s) e o centro de massa andando (ate
    // 0,2 m/s) - vezes isto, comendo da nota sem nunca deixa-la negativa; no giro parado, o tronco e a pelve tombando (o giro em
    // volta do vertical e livre) e o pe escorregando no chao. 0: sem.
    float stillCost = 0.0f;
    // (11/11 11h30, ele: "no clipe do parado, vamos remover os movimentos do clipe, deixa-lo 100% estatico... nosso foco e o RL se
    // embasar nele 100% estatico; e eu devo ver como esta exatamente pelo clipe do exercicio") idleStatic: o idle alerta (e o espelho)
    // seguido no quadro 0, fixo - a nota, os alvos dos motores, os passos e a vista do laboratorio. false: o clipe tocando.
    bool idleStatic = false;
    float turnSpineReward = 0.0f;
    // (Ele: "Parado - penaliza micromovimentos, penaliza fora da pose") Com idleUnified, no parado: a nota
    // max(0, 0,3 + 0,7 pose x firmeza - idleMotionCost M - idlePoseCost D), com M = 1 - exp(-movimento/0,25) (o giro medio dos elos
    // ao quadrado, rad2/s2, mais 10 x a velocidade do centro de massa ao quadrado) e D a distancia a pose (as juntas em 0,15 rad, a
    // base 8 cm, os giros 12 graus, a inclinacao 8 graus, a altura 3 cm; cada uma ate 3), de 0 a 1. Nunca negativa: cair nao compensa.
    float idleMotionCost = 0.0f;
    float idlePoseCost = 0.0f;
    // (11/11 08h58, ele: "no exercicio do parado vamos refatorar as recompensas / punicoes: recompensar ele o quao mais proximo da
    // pose alvo maior a recompensa; quanto menos movimentos maior recompensa; e punir deslizar pes") Com idleRewardPose > 0 (e
    // idleUnified), a nota do parado e: idleRewardPose x proximidade + idleRewardStill x quietude - idleSlideCost x deslize, com a
    // proximidade 0,5 forma (cada junta, 1/(1+(erro/0,15 rad)^2)) + 0,5 lugar (base, giros, inclinacao e altura, cada um
    // 1/(1+(d/largura)^2)) - sobe sempre que ele chega mais perto, sem saturar -; a quietude 1/(1 + movimento/0,25) (o giro medio
    // dos elos ao quadrado mais 10 x a velocidade do centro de massa ao quadrado); o deslize, a soma da velocidade no chao (m/s) dos
    // pes apoiados, ate 0,5. Mais o custo pequeno dos passos do gerador.
    float idleRewardPose = 0.0f;
    float idleRewardStill = 0.0f;
    // (13/11 v1, a volta a base: depois de parar ele ficava com os pes paralelos - o idle alerta tem um pe ~14 cm a frente - porque quieto
    // na base errada valia quase o mesmo e o passo de acerto nao se pagava no resto do episodio.) idleStillBase (0-1): a quietude do parado
    // vale 1 - idleStillBase x (1 - perto da base dos pes), sem fogo; 0 desliga.
    float idleStillBase = 0.0f;
    // (13/11 v1, a revisao: no comando e na estrela a referencia do rumo salta ou gira rapido de proposito - a vista de ate 120 graus, o giro
    // de ate 143 graus/s, a troca de perna - e o "virou de costas" (100 graus por 0,5 s) encerrava e multava o corpo que ainda virava: das
    // "quedas" do comando e da estrela livre medidas, ~80% eram esse fim, nao queda.) turnedGrace: s a mais no limite do virado enquanto a
    // referencia gira rapido e ate turnedGrace s depois; 0 desliga.
    float turnedGrace = 0.0f;
    // (13/11 v1, o braco estabanado na freada) stopArmsReach >= 0: freando (stopping), a correcao da rede nos bracos (armClipReach) vira
    // esta - 0 = os bracos so no clipe da parada; < 0 desliga.
    float stopArmsReach = -1.0f;
    // (14/11 v1, o levantar de costas parecido com o clipe) getUpImitationMask: os exercicios do levantar (bit = o indice em clipGetUpClips)
    // cujos alvos de imitacao (locomotionTargets, a imitacao supervisionada do treino) sao o clipe do comeco no ritmo de
    // getUpImitationPace s, limpos; com a mascara ligada os outros levantares ficam sem alvo; 0 = como antes.
    int getUpImitationMask = 0;
    float getUpImitationPace = 1.5f;
    // (14/11 21h45, ele: 'cobrar mais semelhanca com o clipe para os nossos levantares correndo') O ritmo proprio dos correndo (s ate o
    // ponto de entrega; os de pe ficam no getUpImitationPace); 0: o mesmo dos de pe.
    float getUpImitationPaceRun = 0.0f;
    // (15/11 07h15, ele: "quando ele se levanta, ele ja va girando para a direcao onde ele deve olhar, onde a camera estaria apontada... mesmo
    // que tenha que fazer um 180") getUpViewReward > 0: nos levantares de pe com vista - o levantar e ir de pe (o ponto em qualquer rumo, a
    // vista nele desde o comeco) e o levantar do comando (a vista de antes da queda) -, a vista entra na observacao do levantar (contra o rumo
    // do corpo, bodyYaw, como ja era no comando), a nota soma isto x a subida (cabeca de 0,15 a 1,5 m) x o peito para a vista ((1 + cos)/2) e
    // na troca a vista fica onde estava (antes virava para o tronco). 0: como antes.
    float getUpViewReward = 0.0f;
    // (15/11 07h12, ele: "aumentar a velocidade do levantar de brucos... 0,3 s mais lento que o de costas... ambos com as mesmas velocidades")
    // A escala do tempo do levantar de brucos de pe (o exercicio 1): a faixa do tempo (adaptiveTimeMin/Max), o ritmo (getUpPaceSeconds) e o
    // ritmo da imitacao (getUpImitationPace) vezes isto. 1: como antes.
    float getUpProneTimeScale = 1.0f;
    // (15/11 08h30, ele: "passe a penalizar a mao ficar travada atras das costas, em todos os casos") handBehindWeight > 0: de pe (a cabeca
    // acima de 1,3 m), cada mao mais de 10 cm atras da pelve e a menos de 15 cm da linha do meio (no quadro da pelve) por mais de 0,3 s
    // seguidos custa por passo isto x (o tempo alem de 0,3 s / 0,5 s, ate 1), em todos os exercicios. Medido nas capturas de 15/11: o
    // balanco do braco do clipe passa pela regiao por no maximo 0,23 s seguidos (retas, largada, levantares); a mao travada fica. 0: nada.
    float handBehindWeight = 0.0f;
    // (15/11 09h10, ele: "nessa fase de correcao mais forte.. o sucesso ao levantar deve ficar mais fraco do que o ganho que ele tera por se
    // assemelhar ao clipe") O premio da chegada do levantar adaptativo (adaptiveSpeedBonus) nos levantares de pe vezes isto; os correndo
    // ficam com o inteiro. 1: como antes.
    float getUpStandBonusScale = 1.0f;
    // (14/11 v1) getUpImitationReward: a nota por passo da semelhanca da pose com esse clipe no ritmo (TrackingClipGetUp3D::score, alinhado
    // ao rumo e a raiz do corpo), nos levantares limpos da mascara ate a troca; 0 desliga.
    float getUpImitationReward = 0.0f;
    // (14/11 v1) getUpImitationFar: abaixo desta semelhanca com o clipe no ritmo por getUpImitationFarSeconds, o levantar da mascara acaba
    // (o fim cedo do levantar pelo clipe de 11/11); 0 desliga.
    float getUpImitationFar = 0.0f, getUpImitationFarSeconds = 0.6f;
    // (15/11 12h15, ele: o levantar de costas "da uns passos em rumo a uma certa direcao... deveria permanecer no mesmo lugar 100% das vezes")
    // getUpImitationAnchor: nos levantares de pe, o clipe no ritmo fica preso no mundo desde o primeiro passo (o rumo do comeco e a raiz
    // do corpo ali), como o DeepMimic - o pe de apoio do clipe parado no chao e a deriva cobrada. Desligado: realinhado a cada passo ao
    // rumo cabeca - raiz e a raiz do corpo (de pe o rumo cabeca - raiz vira a inclinacao do tronco e gira: medido, a rede andava ~56 cm
    // depois de de pe contra ~12 cm do clipe).
    bool getUpImitationAnchor = false;
    // (15/11 12h20, ele: "o proprio clipe BUGADO se desloca pelo mundo") clipUnderPelvis: o quadro "posto sob a pelve" do corpo poe a
    // PELVE do clipe na pelve do corpo. Desligado: poe a ORIGEM do clipe ali e a pelve dele fica deslocada pelo deslocamento da raiz do
    // clipe (o idle alerta: 5,5 cm para tras e para o lado; o levantar de costas: 38 cm), andando junto com o corpo - um alvo que o corpo
    // nunca alcanca nas maos e pes e no centro de massa (a imitacao da locomocao, a do parado, a da freada livre e o fantasma desenhado).
    // O fantasma do levantar pelo clipe passa a ser o clipe preso no mundo (o da nota da agilidade).
    bool clipUnderPelvis = false;
    // (15/11 12h23, ele: "esse bug ai tambem ocorre no clipe do parado") idleClipAnchor: no parado, o clipe do idle da imitacao (o
    // da pose e o da nota de rastreio) fica com a pelve na ancora do parado (idleAnchor: onde o parado comecou, re-ancorada so no
    // pouso de um passo do gerador), nao na pelve de agora - preso no mundo, a deriva custa nas maos, pes e centro de massa. O rumo
    // segue o do corpo (01/11: cobrar o rumo do idle derrubou o levantar).
    bool idleClipAnchor = false;
    // (15/11 12h50, ele: "notei um leve desalinhamento das pernas nas locomocoes frontais pelo menos, as outras... acredito que tambem")
    // legAlignWeight: nos retos, a semelhanca vezes (1 - peso) + peso x exp(-soma dos erros^2 / (2 x legAlignWidth^2)), os erros sendo o
    // rumo de cada coxa (para onde o joelho aponta) e de cada pe, contra a pelve, menos o mesmo no clipe (pelo eixo de lado do elo, que fica
    // no plano na flexao). A regua das juntas cobrava 2% por 5 graus no pe e 7% por 20 graus no joelho: medido 12h48, a corrida de frente
    // com os pes ~5 graus girados para a direita, a diagonal 7 com a perna esquerda 16-18 graus para dentro. 0 desliga.
    float legAlignWeight = 0.0f, legAlignWidth = 0.1745f;
    // (15/11 13h00, a revisao: no caminho ativo do parado - idleUnified com idleRewardPose - nada via o lugar no mundo: a forma e por
    // rotacao, a base e relativa a pelve, a quietude e os custos sao velocidade; arrastar 3 cm/s quase nao custava e a distancia
    // acumulada saia de graca - o parado se afastava 14,6 cm em 5 s.) idleHoldWeight: o bonus do parado vezes (1 - peso) + peso x
    // 1/(1 + (d / idleHoldWidth)^2), d a distancia da pelve ao ponto onde o parado comecou (refeito so depois de empurrao ou fogo).
    // A base de 0,5 de ficar de pe nao muda (11/11, ele: "a recompensa por ficar na pose nunca pode ser maior que nao cair"). 0 desliga.
    float idleHoldWeight = 0.0f, idleHoldWidth = 0.10f;
    // (15/11 15h, a revisao profunda - 04_locomocao/revisao_profunda_rl_15_11.md) reviewFixes: (a) as misturas entre clipes - o clipe sem
    // ciclo fica no fim em vez de enrolar para o comeco, o de ciclo leva o rumo acumulado dos ciclos (o giro), e a mistura leva a
    // inclinacao do tronco e das pernas de quando comecou; na troca do levantar para o parado, a mistura parte do clipe do levantar
    // na fase da nota (o relogio adaptativo), parada, no rumo dele - antes um quadro velho do relogio do comeco, as vezes enrolado ate o
    // deitado; (b) o fogo alivia o 'aligned' do parado, o custo de quietude, a coluna e o peito do giro e o quadril reto do parado, como
    // ja aliviava os outros termos. Desligado: como antes.
    bool reviewFixes = false;
    // (15/11 15h10, a revisao - item 5) launchSideMargin: na largada (o corpo ainda abaixo da velocidade de andar, os dois pes no chao),
    // a margem do equilibrio da firmeza so de lado, no eixo de lado do rumo pedido - como andando. Antes, parado, a base inteira: o ponto
    // de captura a frente dos pes, que a arrancada precisa (cair para a frente e o correr), era cobrado nos primeiros ~0,3 s (10 cm fora:
    // o equilibrio 0,06, a firmeza x0,44). Desligado: como antes.
    bool launchSideMargin = false;
    // (15/11 15h15, a revisao - itens 11 e 12) runWidthWeight: correndo e no sprint retos (fora os laterais), a semelhanca vezes (1 - peso)
    // + peso x exp(-((abertura - a do clipe - 4 cm) / 6 cm)^2), so quando os pes estao MAIS abertos que no clipe alem de 4 cm (a largura do
    // trote nao vale na corrida - o clipe no corpo fisico varia - e a base da corrida de frente abriu de 16 para 22 cm, o clipe 12).
    // legAlignCommand: as pernas alinhadas (legAlignWeight) tambem no comando andando, fora das misturas entre clipes. 0/false desligam.
    float runWidthWeight = 0.0f;
    bool legAlignCommand = false;
    // (17/11, o plano de desentortar - a auditoria) referenceSideFix: o rolamento do peito (retas e curvas), a largura dos pes (corrida e
    // trote) e o joelho para dentro medem o CLIPE no eixo de lado do proprio clipe (o rumo da referencia) e o corpo no dele; antes os dois
    // no eixo do corpo, e com a pelve virada a inclinacao do clipe para a frente vazava como inclinacao de lado exigida. false: como antes.
    bool referenceSideFix = false;
    // (17/11, o plano de desentortar - Fase 1, item 3) phaseSyncBothFeet: a fase do clipe acertada no pouso dos DOIS pes (antes so no
    // esquerdo: o passo direito nunca era alinhado). Desligado: so o esquerdo.
    bool phaseSyncBothFeet = false;
    // (17/11, Fase 1, item 2) headingBiasWeight: nas retas com pedido, o rumo MEDIO da pelve contra o do clipe (a media movel de ~0,6 s,
    // o balanco do passo sai) derruba a semelhanca: (1 - peso) + peso / (1 + (vies / headingBiasWidth)^2). Antes o rumo so ia ao log e o
    // episodio so acabava a 100 graus. 0: sem.
    float headingBiasWeight = 0.0f;
    float headingBiasWidth = 0.10f;
    // (17/11, Fase 1 - D2, ele: "Igualar") contactForceObservation: a entrada da forca externa do fogo soma a forca dos contatos que nao
    // sao o apoio (outro corpo que se move; parede ou coisa estatica de normal de lado), a mesma conta no treino e no jogo - no jogo ela
    // era sempre zero. O run.json leva forca_contatos. Desligado: so as forcas do treino.
    bool contactForceObservation = false;
    // (17/11, Fase 1) straightCommand: os fatores da reta (o peito de lado, o rumo medio da pelve, as pernas alinhadas, a largura e o
    // pe cruzado) tambem no COMANDO quando ele esta numa reta - a tecla segurada correndo (estado 0), a vista parada (sem giro) e fora das
    // misturas entre clipes. O comando e o modo do jogo e so a manobra 0 os tinha: ali o torto saia de graca. Desligado: so a manobra 0.
    bool straightCommand = false;
    // (17/11, o plano de desentortar - Fase 2) lateralBiasWeight: nas retas, o LADO medio (~0,6 s, o balanco do passo sai) da cabeca, do
    // peito alto e do meio dos pes contra a pelve, no eixo de lado do corpo, menos o mesmo no clipe (no eixo dele) - o que se ve de tras
    // (a regua visual: a cabeca 5 cm, o peito 3,5 cm a direita, o pe direito 16 contra 11,6 cm). A semelhanca vezes
    // (1 - peso) + peso / (1 + (rms / lateralBiasWidth)^2). 0: sem.
    float lateralBiasWeight = 0.0f;
    float lateralBiasWidth = 0.03f;
    // (17/11, Fase 2 - a literatura: a imitacao >= 50-70% da nota, em produto) imitationGate: andando, a nota inteira vezes
    // (1 - g) + g x min(1, semelhanca / imitationGateFull). 0: sem.
    float imitationGate = 0.0f;
    float imitationGateFull = 0.6f;
    // (16/11, o plano de melhoria do RL - foco 5, ele: "treinar melhor o RL com colisoes no mundo e com outros jogadores")
    // locomotionSceneryFraction: a fatia dos episodios de locomocao (de pe, no treino; o comando e o parado tambem) com 2-5 coisas paradas em
    // volta do caminho -
    // caixas, paredes curtas e pilares do tamanho de um jogador - de 1 m atras a 8 m a frente, quase todas a 0,55-2,15 m da linha do pedido
    // (12% das de mais de 3 m na frente, na linha: a trombada). Os raios do ambiente da rede so viam o livre na locomocao (as paredes so
    // existiam no levantar e no prensado do fogo) e no jogo qualquer parede perto saturava a entrada. 0 desliga.
    float locomotionSceneryFraction = 0.0f;
    // (15/11 15h20, ele: "o parado sempre distribui o peso em cima da perna esquerda, ao inves de ficar perfeitamente na pose do clipe")
    // idleWeightCenter: o bonus do parado vezes (1 - peso) + peso x 1/(1 + (d / 2,5 cm)^2), d a diferenca entre o centro de massa do
    // corpo e o do clipe do idle, cada um contra o meio dos proprios pes, no rumo da propria pelve (o fogo alivia). Nada media para
    // onde ia o peso: a base e relativa a pelve e a forma e por rotacao. 0 desliga.
    float idleWeightCenter = 0.0f;
    // (15/11 15h20, ele: "a corrida nao deveria ser abaixo de 5,5 m/s") speedWidthFraction: a largura do termo da velocidade, fracao da
    // pedida (nunca abaixo de 1 m/s). 0,35 (o de antes): a 6 m/s, 2,1 m/s de largura - 84% da pedida valia 0,82 do termo e subir ate a
    // pedida rendia pouco. Menor = mais cobranca perto da pedida.
    float speedWidthFraction = 0.35f;
    // (15/11 15h56, ele: "a arrancada da corrida piorou, bem mais estabanada agora" - com os bracos calmos de 10/11, que mudam o alvo dos
    // motores; desligados.) launchArmsCalm: na largada, a semelhanca vezes (1 - peso x lw) + peso x lw / (1 + (excesso / 0,4)^2), o excesso
    // sendo a velocidade media das maos contra o peito alto alem da do clipe no mesmo instante (no ritmo da largada) mais 0,15 m/s - a
    // regua da arrancada mede 1,8 m/s contra 1,4 do clipe. A rede aprende a acalmar os bracos; o alvo dos motores nao muda. 0 desliga.
    float launchArmsCalm = 0.0f;
    // (14/11 v1) clipSideNow: o "virou para o outro lado" da tentativa nova compara o peito com o do clipe no quadro de agora (nao o do
    // comeco) - para o clipe de costas que rola e levanta de brucos (levantar_costas_rola); desligado: como antes.
    bool clipSideNow = false;
    // (14/11 v1) clipTrackMask: bit k = o levantar limpo (sem fogo, fora do comando) do exercicio k pelo rastreio do clipe mesmo com o
    // adaptativo ligado; 0: como antes.
    std::uint32_t clipTrackMask = 0u;
    float idleSlideCost = 0.0f;
    // (10/11 04h, ele: "desde agora precisamos cobrar a coluna correta, cabeca, bracos com mais rigor durante as locomocoes e
    // parado") A regua da imitacao da coluna, da cabeca e dos bracos vezes isto (andando, no giro e nas trocas); 1: sem.
    float upperRigor = 1.0f;
    // (10/11 04h, ele: "a arrancada da corrida que e estranha, deveria ser a mesma arrancada igual do unreal engine") De pe, nos retos
    // da corrida: a arrancada da Unreal do rumo (corrida_start_<rumo>_<pe>, o pe pelo idle) antes do ciclo, a velocidade pedida a da
    // raiz dela, a saida a 92% da velocidade de cruzeiro e o ciclo no quadro com as pernas mais parecidas; sem: o idle_to_sprint na
    // frente e a rampa nos outros rumos.
    bool unrealStarts = false;
    // (10/11 06h05) A escala de reproducao da arrancada da Unreal (0: a que leva a velocidade de saida dela a pedida, ate 1,25).
    float unrealStartScale = 0.0f;
    // (10/11 06h35, ele: "nada de passinhos nem lentidao, apenas o IMPULSO real da corrida") Na janela da largada: o progresso inteiro na
    // velocidade (todos os rumos) e a imitacao pelo quadro mais parecido do clipe de largada (+-0,3 s do relogio).
    bool liveLaunch = false;
    // (10/11 07h05, ele: "o maior ganho de velocidade inicial com o menor numero de passos, sem bracos estabanados e pulinhos") Na
    // janela da largada, cada pe que pousa paga (tempo no ar - 0,15 s) x isto: o passinho perde, a passada longa ganha. 0: sem.
    float launchStepReward = 0.0f;
    // (10/11 07h10, ele: "executar literalmente o movimento do clipe e reforcar em largada, pelo menos ate ele aprender") A parcela do
    // clipe nos motores na janela da largada (o alvo e a velocidade-alvo); o treino a desce ate 0. 0: sem.
    float launchClipShare = 0.0f;
    // (10/11 07h15, aprovado por ele: "mao so na largada") O nivel da mao na pelve na janela da largada (linear); o treino o desce a 0.
    float launchHand = 0.0f;
    // (10/11 08h20, ele: "se conseguir pelo menos tirar os passinhos no lugar e os bracos estabanados na largada, estara otimo") Na
    // janela da largada: cada pe que pousa sem ter andado 15 cm no rumo pedido custa ate isto (para tras, o maximo); 0: sem.
    float launchInPlacePenalty = 0.0f;
    // E os bracos da pose do idle a do clipe com a velocidade do corpo (cheios a 1,5 m/s).
    bool launchCalmArms = false;
    // (10/11 10h40, a coluna acoplada - jointCouplings do perfil) Nos eixos acoplados da coluna, a velocidade-alvo e a do proprio
    // alvo de posicao da rede (a diferenca dele de uma decisao para outra), nao a do canal livre da acao: medido, a rede usava o
    // canal como torque puro contra o batente (a coluna travada no maximo, 82-100% do parado). false: o canal livre, como antes.
    bool spineVelocityFeedforward = false;
    // (10/11 11h45, ele: "quando parado, sempre o IDLE alerta e o alvo... recobrar a pose assim que possivel, com o minimo de
    // passos", "nao e troca de objetivo"; TrackingBasePosture3D) A postura base como atrator, com o pedido de velocidade zero (da
    // ordem de parar ao parado, nos empurroes): estar perto do idle alerta pela regua que nao satura (na parada livre e na base do
    // parado), o progresso para ele premiado a cada decisao (o potencial -d, Ng 1999) com este peso, na proporcao do quanto o corpo
    // ja freou, e cada passo de ajuste custando BasePostureStepCost. 0: como antes.
    float basePosture = 0.0f;
    // (10/11 13h45, ele: "as poses gerais ficaram molengas desengoncadas... ele precisa ficar mais rigido... ao frear e pos-freio ele
    // fica todo molengo") O tonus do corpo: a rigidez e o amortecimento de todos os motores vezes isto (o limite de torque - a forca -
    // nao muda). 1: como antes.
    float motorTone = 1.0f;
    // (10/11 14h, ele: "ao frear e pos-freio ele fica todo molengo"; medido: na freada a lombar no batente (44 graus) e o tronco
    // caido em C, e a nota nao puxava de volta - os nucleos do tronco (gaussianas de ~10 graus) davam ~0 a 45 graus.) A inclinacao
    // da pelve, do peito e da cabeca contra o clipe na regua de cauda longa, 1/(1+(d/10 graus)^2), nesta parcela da nota inteira do
    // rastreio (locomocao, freada, giro): longe do clipe ainda puxa. 0: como antes.
    float postureTail = 0.0f;
    // (10/11 14h25, ele: "pos-freio... parece a coluna quebrada") O tonus so da coluna (Abdomen, Chest, UpperChest, todos os eixos):
    // a rigidez e o amortecimento vezes isto. Medido: as 3 juntas de 440 N.m/rad em serie dao ~150 N.m/rad no tronco, contra ~250-440
    // in vivo com os musculos pre-ativados (Moorhouse & Granata 2005; Andersen 2004) e ~785 (Cholewicki 2000). 1: como antes.
    float spineTone = 1.0f;
    // (17/11, o plano de desentortar - os bracos abertos) O tonus so dos bracos (UpperArm, Forearm, Hand): a rigidez e o amortecimento vezes
    // isto. Medido na r130240 correndo: o ombro 18 graus fora do clipe e o cotovelo 2,5; com a correcao da rede nos bracos a 1/3 (0,05) os
    // cotovelos seguiam 7 cm para fora de cada lado - o motor do ombro nao segura o clipe contra o balanco do tronco. 1: como antes.
    float armTone = 1.0f;
    // (17/11, o Codex achou a causa dos bracos abertos) clipVelocityRotation: a velocidade-alvo do clipe nos bracos e na cabeca (os eixos
    // que seguem o clipe: armClipReach, headClipReach, e a janela da largada) por JUNTA, como o backend a entrega ao Jolt - a velocidade
    // angular no referencial da junta do filho (SetTargetAngularVelocityCS): w = Log(conj(Exp(a_agora)) Exp(a_proximo)) / dt. Antes, a
    // derivada de cada coordenada: no ombro (3 eixos, 60-70 graus) as duas diferem ate 6 rad/s e o braco abria. Medido pelo Codex na
    // r130240 (n132): cotovelos 61,6 -> 46,3 cm (clipe 45,4), maos 60,8 -> 44,3 (clipe 43,4). Desligado: como antes.
    bool clipVelocityRotation = false;
    // (17/11, o tronco ereto e a passada curta - a imitacao supervisionada dos alvos do clipe, --imitar) imitationVelocityTargets: os
    // alvos do clipe da imitacao (locomotionTargets) com a velocidade-alvo de cada eixo do clipe - por junta, como o backend a quer (a mesma
    // conta do clipVelocityRotation) - no lugar das saidas de velocidade; antes so as posicoes vinham e as 39 saidas de velocidade eram
    // puxadas para zero (a imitacao freava os motores: em 16/11 o --imitar 1 levou as pernas a 6,6 graus do clipe e a corrida de 5,0 a
    // 3,6 m/s). Desligado: como antes.
    bool imitationVelocityTargets = false;
    // (17/11, o plano de desentortar - Fase 3, o auto-equilibrio "MUITO" melhor; a literatura: DReCon, projeteis pequenos e frequentes em
    // ponto sorteado do corpo aumentam muito a robustez sem impacto visivel no estilo - em todos os episodios, entao a rede nao anda "com
    // medo" de um episodio especial) projectileRate: na locomocao de pe (treino), por segundo, a chance de um projetil - um impulso de
    // projectileImpulse N.s (sorteado) num elo sorteado, de lado e um pouco para cima ou para baixo, em projectileSeconds; ele entra na
    // entrada da forca externa enquanto age (como os empurroes). 0: sem.
    float projectileRate = 0.0f;
    std::array<float, 2> projectileImpulse { 10.0f, 40.0f };
    float projectileSeconds = 0.06f;
    // (10/11 14h30, ele: "pos-freio parece a coluna quebrada", "a coluna torta"; medido: a rede empurra as juntas da coluna contra os
    // batentes para se enrijecer - a lombar no batente de flexao na freada, a torcao no batente no parado.) O custo do batente: por
    // decisao, o torque que os batentes da coluna fazem (todos os eixos) alem de JamSlack, em unidades de JamScale, vezes isto.
    // Encostar no fim nao custa; empurrar o osso contra ele, sim (como o termo de limite de junta do legged_gym). 0: como antes.
    float jamCost = 0.0f;
    // (10/11 15h, ele: "ele nao consegue ficar imovel, os pes deslizando... eu quero ele estatico quando parado"; medido: apoiados,
    // os pes deslizam 5,5 cm/s e giram 13 graus/s sem nunca sair do chao.) Com o pedido zero e o corpo parado (voltando a nada ate 0,5
    // m/s), cada pe apoiado que desliza ou gira no chao custa isto vezes min(2, v/5 cm/s + giro/0,5 rad/s) por decisao. O pe no ar
    // (o passo de reequilibrar) nao paga. 0: como antes.
    float footSlipCost = 0.0f;
    // (10/11 15h25; medido: o clipe do idle tocado direto nos motores, sem a rede, nao fica de pe parado - 11 passos, os pes deslizam
    // 247 cm em 8,5 s: sem compensar a gravidade, cada motor e uma mola que o peso afunda (o joelho ~9 graus) e nenhuma pose fica.) A
    // compensacao da gravidade nos motores da cabeca da locomocao: o torque que segura o peso de cada osso na pose de agora (o tonus
    // antigravitacional), dentro da mesma forca maxima do motor. false: como antes.
    bool motorGravity = false;
    // (10/11 04h40, ele: "ensinar a fazer pelos clipes, para ter base e DEPOIS de estar fazendo o movimento dos clipes deixar liberdade")
    // Com stopClips e freeStop: na parada final dos retos, a parada da Unreal do rumo (onde ha uma boa) e, quando ela assenta, a livre
    // (a cabeca da locomocao ate ficar de pe); sem clipe para o rumo, a livre direto.
    bool stopClipsBase = false;
    // (10/11 05h, ele: "ve qual dos tres fica melhor") A parada de frente por tres referencias - a da Unreal, a tocha da Mixamo e a "Running
    // To Turn" sem o giro -, sorteada a cada parada no treino (stopFrontVariant -1) ou fixa (0, 1, 2) na medida.
    bool stopFrontVariants = false;
    int stopFrontVariant = -1;
    // (05h10) As variantes no sorteio do treino (bits: 1 Unreal, 2 tocha, 4 virada, 8 "Running To Stop").
    int stopFrontVariantMask = 7;
    // (10/11 05h25, ele: "faca" - a "Running To Stop" com a entrada gradual) A entrada no clipe de parada: a passada se mistura nele por
    // stopEntryBlend s (a inclinacao para tras cresce aos poucos) e o instante de entrada nos primeiros stopEntryWindow s dele. 0,2 e 0,3: como antes.
    float stopEntryBlend = 0.2f;
    float stopEntryWindow = 0.3f;
    // (09/11 15h, a etapa 2 do mesmo principio) A troca livre: nas trocas media, forte e livre e na troca de tecla do comando, o
    // pedido novo (a tecla, ou a vista girando na meia-volta) vira o objetivo por uma janela de tempo fixo pelo tamanho da troca -
    // a velocidade nova, o peito na vista, de pe -, sem passada nem meia-volta para seguir; no fim dela o rastreio volta no ciclo
    // do rumo novo, na fase das pernas do corpo.
    bool freeTroca = false;
    // (09/11 15h40) Na parada livre, ja parado (perto de zero), a nota inteira vezes 1 - freeStandGate (1 - pose), pose = a altura
    // da pelve vezes o peito alto reto contra os do idle - as folhas da r75500 paravam de pe, mas curvadas e agachadas 4-6 cm, a
    // base mais estavel pagava quase o mesmo. 0: so a parcela somada de antes.
    float freeStandGate = 0.0f;
    // (09/11 17h30, a etapa 3a) O parado livre: no parado com a cabeca da locomocao (o exercicio parado e o do equilibrio com
    // empurroes), a nota e a do parado da parada livre - quieto, de pe e ereto na altura do idle, o peito na vista, a pose do idle
    // so como termo leve -, sem o gerador de passos e sem o fim por pose perdida (os pes fora do lugar do idle); cair encerra.
    bool freeIdle = false;
    // (09/11 19h15, ele: "dar uma reforçada nas POSES em geral... a coluna mole, arqueada e arrancadas estranhas... a pose
    // parado se recontorce") O reforço das poses: andando, a nota inteira vezes 1 - poseBoost (1 - pose), pose = a coluna (o erro
    // por junta contra o clipe), a inclinação alem da do clipe e a pelve abaixo da dele; na arrancada a coluna e a cabeça
    // cobradas inteiras; no parado (da parada livre e do exercício parado), a coluna contra a do idle no portão da pose. 0: sem.
    float poseBoost = 0.0f;
    // (10/11 00h20, ele: "cobrar semelhanca alta com o CLIPE SEM interferir nos momentos de que ele necessita sair do clipe para se
    // recuperar, como pos empurroes, recuperacoes e freadas") O empurrao livre: andando, durante o empurrao e PushFreeSeconds
    // depois, sem imitacao - a nota e voltar ao movimento pedido, de pe, com o peito na vista.
    bool freePush = false;
    // (10/11 00h35, ele: "o Parado precisa ser mais reforcado ainda essa semelhanca com o clipe") No parado livre e no parado da
    // parada livre, a semelhanca com o idle (a regua do contrato com suavidade 2) vale este peso da qualidade de pe; o resto
    // (altura, peito reto, quieto) o complemento. 0: a qualidade de antes (a semelhanca 0,20).
    float freeIdleMatch = 0.0f;
    // (10/11 00h35, ele: "os passos normais... nas freadas e recuperacoes, nao precisem ser esses passos elevados para cima (trotar
    // parado)... algo mais natural") Nas fases livres (parada, parado, pos-empurrao), o pe acima da altura de repouso alem desta
    // folga (m) custa ate 40% da nota (largura 6 cm). 0: sem.
    float freeStepHeight = 0.0f;
    // (10/11 01h30) O parado dobrado encerra como a queda, sozinho (sem o resto do poseBoost): parado, abaixo de 0,5 m/s e fora dos
    // 2 s depois de um empurrao, o peito alto a mais de 34 graus do idle por 0,5 s. Sem ele o parado voltou a dobrar na cintura.
    bool idleBentEnd = false;
    // (11/11, E5-4) A linha final do episodio com o feedback do passo final: o begin() do recomeco roda dentro do step e
    // zerava o locomotionFeedback antes de o Python le-lo - a nota (o teto do custo), o portao do estilo e as medias rastreio_*
    // contavam um zero. Ligado, o feedback do passo final volta depois do begin() (o proximo decide o zera de novo).
    bool finalRow = false;
    // (10/11 01h45) O parado visto como o parado da parada livre: no parado da cabeca da locomocao, sem o relogio do clipe do idle
    // na observacao e com a vista contra o peito de verdade (antes fixa em "de frente"). A mesma rede fica de pe depois de uma
    // parada livre e se dobra na cintura no exercicio parado - o teste de que o contexto liga o modo dobrado.
    bool idleLikeStop = false;
    // (10/11 01h50, ele: "Esta totalmente errado a forma do parado.. investigue profundamente e corrija") Medido: o parado agachado
    // 15 cm e o peito 18 graus, e ali a nota plana - a altura num nucleo de 3 cm, a semelhanca 0,03. A altura e o peito alto com o
    // nucleo fino mais uma rampa larga (ate 20 cm e 40 graus), e a semelhanca com o idle tambem na regua larga (suavidade 6).
    bool idleWide = false;
    // (10/11 02h30) A simetria nos clipes ciclo_*: o espelho (mirror_info) so ligava nos nomes antigos (jog, jog_strafe_left...) e
    // os retos de agora usam ciclo_trote_* e ciclo_corrida_* - a perda de simetria estava desligada (o espelho em 0% dos retos), e o
    // lateral direito ia a 92-94% da pedida com o esquerdo a 11-40%. Meio ciclo: frente e costas; pares: esquerda e direita.
    bool mirrorCycles = false;
    // (06/11 12h, o atlas: correndo, a meia-volta saindo de costas ou de lado cai 93-100% NO MEIO do giro (~0,3 s depois da
    // troca), com 1,5 m/s para tras ou 4 de lado contra o giro - de frente, 8-23%) De costas, a freada ate esta velocidade
    // antes da meia-volta (m/s; 0: TurnSettleSpeed, 1,5).
    float turnSettleBackSpeed = 0.0f;
    // (06/11 13h05, a vigilia - para ELE decidir: a meia-volta da media/forte saindo de costas ou de lado cai 93-100% correndo
    // no meio do giro, e freada nenhuma resolveu) Saindo de costas ou de lado (rumos 2-6), a media e a forte trocam o rumo de
    // peito fixo, como a suave (sem a meia-volta); de frente seguem pela meia-volta. Desligado: como antes.
    bool trocaFixedChestNonForward = false;
    // (06/11 01h10, ele: "a troca de direcoes ainda esta muito fragil... pense bem o que fazer para ajudar ele a aprender
    // melhor") A pratica das trocas: o rumo de partida das trocas 1-3 pelos pesos dados (o treino manda, pelas quedas de
    // cada rumo; todos 0: por igual), o intervalo entre trocas 1-2 s e a troca que chega no meio da anterior esperando.
    bool trocaPractice = false;
    std::array<float, 8> trocaHeadingWeights {};
    // (05/11 22h20/22h55, ele: "na corrida ele esta cruzando passos") > 0: na corrida para a frente e nas diagonais da frente,
    // o pe apoiado mais para dentro que o mesmo pe do clipe (alem de 1,5 cm) derruba a semelhanca: exp(-(falta / isto)^2) (m).
    float footCrossSlack = 0.0f;
    // (05/11 21h10, o levantar) A estabilidade medida no idle (o portao do levantar e do parado) fora da janela de cada
    // empurrao e com a ancora recomecando depois dele; a recompensa nao muda.
    bool idleReanchor = false;
    // (04/11 23h40, ele: "nas trocas de direcao nao ha movimento de pegar no tranco... nem de freada e inversao") Nas trocas
    // (1-4), a coluna da referencia (nos motores) inclina com a aceleracao da velocidade pedida no rumo do peito, atan(a / g),
    // ate este angulo (rad): freando para tras, arrancando no rumo novo para a frente - qualquer angulo de troca; 0: sem.
    float accelLean = 0.0f;
    // (04/11 14h15) No trote de frente e nas diagonais da frente: a coluna da referencia para a frente (rad) e, na largada de
    // pe, a da largada, voltando a ela com a janela da arrancada (0: como o clipe).
    float frontLean = 0.0f;
    float frontStartLean = 0.0f;
    // (05/11 00h40, ele: "no corrida, nao seria melhor se o corpo todo ficasse mais inclinado para frente? nao a coluna apenas
    // mas o corpo todo") Na corrida de frente e nas diagonais da frente (e nas curvas da corrida): a referencia inclinada
    // runLean (rad) - a pelve e a coluna retas, como a inclinacao do trote - com runLeanLegs dela nas pernas (0: so o dorso;
    // 1: o corpo todo girado nos quadris). 0: como o clipe.
    float runLean = 0.0f;
    float runLeanLegs = 0.0f;
    // (05/11 04h10, ele: "como fizemos com a corrida de frente, inclinar o corpo todo... nesse caso para tras, algo sutil")
    // Na corrida de costas e nas diagonais de costas (rumos 3-5): a referencia inclinada runBackLean (rad) para tras, com
    // runBackLeanLegs dela nas pernas, como a runLean.
    float runBackLean = 0.0f;
    float runBackLeanLegs = 0.0f;
    // (05/11 01h05, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s, e ai usamos essa nova pose" - a
    // imagem "CORRETA": o corpo todo tombado para a frente, a perna de tras em linha com o tronco) O sprint (a faixa 3): so
    // para a frente, o clipe sprint_maximo largando de pe pela arrancada, no pedido fixo sprintSpeed (m/s; 0: o do clipe),
    // com a inclinacao sprintLean (rad, o dorso reto) e sprintLeanLegs dela nas pernas, como a runLean.
    float sprintSpeed = 0.0f;
    float sprintLean = 0.0f;
    float sprintLeanLegs = 0.0f;
    float idleBlendReach = -1.0f;
    // (04/11 15h, ele: "os passos de recuperacao e estabilizacao, predict de movimento e etc. para o auto-equilibrio... nos dois
    // cerebros"; vault 04_locomocao/plano_passos_de_recuperacao_04_11.md) O equilibrio:
    //  - balanceObservation: as entradas do equilibrio (o layout levantar-v7, antes da habilidade) - o ponto de captura contra a
    //    base e os pes, a margem, a ameaca, omega, o giro do corpo (TrackingGetUpContract3D::observeBalance) e o estado do
    //    gerador de passos (zeros fora do idle da locomocao);
    //  - stepChannels: os 4 canais da rede para o passo, depois dos eixos da acao (crus, -1..1);
    //  - balanceSteps: o gerador de passos (TrackingBalanceSteps3D) no idle da cabeca da locomocao, com a nota do idle que
    //    deixa o passo acontecer (a semelhanca contra a referencia do passo, o equilibrio, a base do idle);
    //  - idleThroughPushes: o idle segue no empurrao (a locomocao o segura com o passo; caindo, a recuperacao);
    //  - balanceDrillFraction/balancePushLevel: a fatia dos comecos de pe (o parado) com empurroes, sem a mao - o equilibrio
    //    parado -, e o nivel deles (0-1 das faixas pushMinimum/MaximumNewtons e do continuo; o curriculo do treinador).
    // (balanceObservation e stepChannels mudam os tamanhos: so no construtor.)
    bool balanceObservation = false;
    bool stepChannels = false;
    // (11/11, E5-6: a nota do parado cobra a deriva da ancora - idleAnchor, onde o parado comecou, re-ancorada depois de cada passo
    // do gerador -, mas a entrada nao a tinha.) idleAnchorObservation: 2 entradas antes da habilidade - a ancora na frente e ao
    // lado da pelve, em 10 cm (ate 3); zeros fora do parado ancorado da locomocao. Muda o tamanho: so no construtor.
    bool idleAnchorObservation = false;
    // (11/11 18h05, ele: "encontrar uma forma com que o levantar de costas rolando nao seja contaminado pelo levantar de costas
    // normal... ele parece estar imitando o levantar de costas normal") A variante do levantar pelo clipe na entrada, antes da
    // habilidade (a ultima): os dois comecam deitados de costas, sem pedido e no mesmo relogio - sem ela a rede nao tinha como
    // saber qual fazer. Vale a de setClipGetUpVariants do exercicio enquanto a recuperacao segue o clipe; 0 no resto. No jogo,
    // quem escolhe o levantar a liga.
    bool clipVariantObservation = false;
    // (12/11 06h40, ele: "forcar a adaptacao inteligente a conseguir enfrentar as adversidades" - vault plano_fogo_do_levantar_12_11, as
    // entradas da forca e da tendencia) As 6 entradas do fogo, antes da habilidade (depois da variante): a forca externa no corpo (o
    // continuo do fogo do levantar e o empurrao ou a pancada de agora; no jogo, a dos contatos) e a aceleracao do centro de massa
    // (suavizada em ~0,1 s), no rumo do corpo (frente, lado, cima), em pesos do corpo e em g. Muda o tamanho: so no construtor.
    bool fireObservation = false;
    bool balanceSteps = false;
    bool idleThroughPushes = false;
    float balanceDrillFraction = 0.0f;
    float balancePushLevel = 0.5f;
    // (As sondas do gerador: a carga maxima do pe que sai, o peso da carga e o pe no ar pelo IK a partir da pelve de agora.)
    float stepLoadLimit = 0.8f, stepLoadWeight = 0.4f;
    // (04/11 15h30, a sonda: o levantar acaba num pulinho - os dois pes no ar ~0,35 s, a cabeca 14 cm acima de de pe - e entrega
    // o corpo embalado) O levantar que chega equilibrado: na entrega ao idle, este premio vezes a qualidade da chegada (o ponto
    // de captura dentro da base, sem velocidade no plano, sem subir); de pe quase (cabeca acima de 1,2 m) com a recuperacao,
    // os dois pes no ar custam isto por decisao (0: desligados).
    float arrivalBonus = 0.0f;
    float flightPenalty = 0.0f;
    // (04/11 16h40, a folha do levantar: ele sobe explosivo, os bracos para o alto, passa do ponto e cai de costas - entregue no
    // meio do tombo, a 0,4-1,1 m/s e o ponto de captura 10-88 cm fora da base; o premio da chegada e de uma decisao so) A
    // subida por cima dos pes: com a recuperacao no comando e a cabeca acima de RiseHeadHeight, o ponto de captura fora da
    // base (ate 30 cm) e a velocidade do centro de massa no plano (ate 1 m/s) custam ate isto por decisao (0: desligada).
    float riseBalancePenalty = 0.0f;
    // (04/11 21h35: com a multa da subida o levantar sobe 98%, mas a entrega ainda pega o corpo a 0,5-1 m/s e 84-90% caem
    // no idle) A entrega calma: depois de levantar, a recuperacao so entrega com o ponto de captura dentro da base (ate 5 cm
    // fora) e o centro de massa abaixo de 0,4 m/s no plano - ou este prazo (s) de pe; 0: a entrega na hora de antes.
    float calmHandoffSeconds = 0.0f;
    // (04/11 15h45, a avaliacao do treino: parado sem a mao, sem empurrao, sem cair 0-12% - o idle so fica de pe com a mao; o
    // alcance da correcao no idle, 0,02 rad nas pernas, nao cabe nem a estrategia do tornozelo) A autoridade sob ameaca: no
    // idle, o alcance vai do de sempre (idleResidualScale, a pose) a este (a fracao do alcance cheio) com a ameaca do ponto de
    // captura (1 - margem/8 cm: de 0,4 - 4,8 cm dentro da borda - ao cheio na borda). 0: desligada.
    float idleThreatReach = 0.0f;
    // (04/11 23h10, ele: "APENAS o levantar-se em cabeca separada, o ato de ficar em pe na cabeca da locomocao") O evitar a
    // queda no idle da cabeca da locomocao: de pe no comeco ela segura, o empurrao incluido; a recuperacao so caido.
    bool avoidOnIdle = false;
    bool stepSwingFromBody = false;
    // A forca auxiliar da corrida (a faixa 2 nos rumos; < 0: a mesma da locomotionAssist): a corrida comeca com a mao
    // dela, sem esperar o trote.
    float runAssist = -1.0f;
    // As manobras do rastreio (04/11, ele: "adaptar os treinos de TROCAS de direcoes e curvas que possuiamos no b21"): a
    // fatia dos episodios de rumo com manobra e, na avaliacao, a manobra (-1 nenhuma; 0 uma por mundo em ciclo; 1 troca
    // suave 45 graus, 2 media 90, 3 forte 135-180 - as tres de peito fixo -, 4 troca livre - o olhar gira para o rumo novo
    // a 4 rad/s e o peito vai junto -, 5 curva suave R 12-18 m, 6 curva em S R 5-10, 7 curva fechada R 3-8 - as curvas
    // com o peito no caminho). E a forca auxiliar delas (< 0: a do rumo).
    float maneuverFraction = 0.0f;
    int evaluationManeuver = -1;
    float maneuverAssist = -1.0f;
    // A forca auxiliar por exercicio (04/11 03h45, ele: "um sistema em que POR TREINO ele ve o quao bem ele esta adaptado
    // aquele nivel de forca auxiliar... e vai diminuindo dinamicamente"): 0-7 os rumos do trote, 8-15 os da corrida, 16-22
    // as manobras (1-7), 23 o giro parado; < 0 usa a do grupo (locomotionAssist/runAssist/maneuverAssist).
    // (24, 04/11 04h55: o parado - e o idle depois de levantar - na cabeca da locomocao.)
    // (25-27, 04/11 11h50, ele: "adicionar os exercicios das curvas agora em corridas": as curvas 5-7 na faixa da corrida.)
    // (28, 05/11 01h05, ele: "vamos fazer um treino de SPRINT com a velocidade meta de 8 m/s"): o sprint para a frente.
    // (29, 05/11 04h40, ele: "vamos fazer seguir o clipe para o levantar; no futuro, depois que tiver bonitinho e funcional,
    // vamos soltando devagar"): o levantar pelo clipe (getUpTracking).
    // (30-35, 08/11: a estrela - trote fixo, livre, girando; corrida fixo, livre, girando.)
    // (36, 08/11 14h40: o comando.)
    static constexpr std::size_t DrillAssistCount = 37;
    static constexpr std::size_t StarDrill = 30;
    static constexpr std::size_t CommandDrill = 36;
    static constexpr std::size_t SprintDrill = 28;
    static constexpr std::size_t GetUpDrill = 29;
    std::array<float, DrillAssistCount> drillAssist { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };
    // A cabeca da locomocao corrige o clipe (03/11 14h10): com ela agindo, o alvo de cada junta e o do clipe do passo
    // (no quadro do fim da decisao) mais a acao, em radianos - a correcao residual (o v8 do RL v1, a unica marcha com cara
    // humana do projeto); sem isto, a mistura do clipBlend.
    bool locomotionResidual = false;
    // Fixed curriculum layout in the laboratory; -1 preserves the benchmark.
    int sceneryPreset = -1;
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
    // (08/11, ele: "em TODOS os treinos atuais, as pancadas e empurroes... os empurroes continuos de 3-5 segundos e as pancadas
    // secas... a fase de fogo") A duracao dos continuos em movimento (s; antes 1-2) e a fatia dos episodios do treino com
    // empurroes (1: todos, como antes).
    float movingPushHoldMinimumSeconds = 1.0f;
    float movingPushHoldMaximumSeconds = 2.0f;
    float pushEpisodeShare = 1.0f;
    // (08/11) O ponto do empurrao sorteado (o tronco repartido, o peito alto, a pelve ou um ombro); sem, o tronco repartido.
    bool pushVariety = false;
    // (08/11 12h30) O fogo por grupo (no treino): as faixas (pancada min, max, continuo min, max; N) da frente, dos lados, das
    // costas, do giro, do parado e do resto; sem, as faixas unicas.
    bool fireGrouped = false;
    // (08/11 18h05) O laboratorio com a caixa "Empurroes": numa avaliacao, os empurroes como no treino (o grupo, o intervalo).
    bool fireShown = false;
    // (08/11 13h05) Nos retos de lado, a regua das pernas e dos pes vezes isto (1: a de todos).
    float lateralLegStrict = 1.0f;
    // (08/11 13h50) Durante o empurrao e ate 1,5 s depois, a regua da imitacao pela metade.
    bool pushImitationRelief = false;
    std::array<std::array<float, 4>, 6> fireGroupBands {};
    // (08/11) No levantar, o corpo inteiro no ar depois de tocar o chao (a cabeca ate 1,2 m): a multa por decisao.
    float riseFlightPenalty = 0.0f;
    // (08/11) Nos retos, andar no rumo abaixo desta fracao da velocidade de referencia por 1 s (depois de 2,5 s, fora do
    // empurrao) encerra como a queda (o fim 7, "lento"); 0: sem.
    float slowLimit = 0.0f;
    // (12h25) O piso por grupo: a frente (0, 1, 7), os lados (2, 6), as costas (3, 4, 5); slowLimit e o maior (> 0 liga).
    // (09/11 01h20) E por faixa: 0-2 o trote, 3-5 a corrida - o trote levava o piso da frente a 0,80 e metade das corridas
    // acabava por lento. Com 3 valores no setSlowLimits, os dois iguais (como antes).
    std::array<float, 6> slowLimits { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
    // (08/11, ele: "6 exercicios novos, da estrela de oito pontas: trote peito livre, fixo e girando; corrida peito livre,
    // fixo e girando... garanta bem a boa estruturacao das fases de aprendizado... inicialmente vamos deslocar uma % dos
    // treinos a estrela") A fatia dos episodios de rumo na estrela; os pesos e as fases (1-4) dos 6, na ordem trote fixo,
    // livre, girando, corrida fixo, livre, girando; a duracao do episodio dela (s). Na avaliacao, evaluationManeuver 8 peito
    // fixo, 9 livre, 10 girando, 11 os tres em ciclo pelo mundo.
    float starFraction = 0.0f;
    std::array<float, 6> starWeights { 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f };
    std::array<int, 6> starPhases { 1, 1, 1, 1, 1, 1 };
    float starSeconds = 14.0f;
    // (08/11 14h40, ele: a rede preparada para os comandos do jogo) O comando - o jogador emulado (teclas, shift, mouse, paradas):
    // a fatia dos episodios de rumo, a fase (1-4) e a duracao do episodio (s). Na avaliacao, evaluationManeuver 12.
    float commandFraction = 0.0f;
    int commandPhase = 1;
    float commandSeconds = 20.0f;
    // (08/11 15h20, ele: "ao final da rodada a PARADA") A parada no fim dos retos: a fatia dos episodios retos do treino com ela,
    // a janela do instante (s), a freada da referencia (m/s^2) e se a avaliacao tambem para (no meio da janela).
    float stopFraction = 0.0f;
    float stopEarliest = 4.5f, stopLatest = 5.5f;
    float stopDeceleration = 5.0f;
    bool stopInEvaluation = false;
    // (08/11 16h20) A referencia da parada pelas paradas da Unreal (trote_run_stop_* e corrida_stop_*, 8 rumos, os dois pes):
    // o clipe da freada de verdade - o passo que planta e assenta - em vez do passo da corrida com a velocidade caindo.
    bool stopClips = false;
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
    // (05/11 04h40) O levantar pelo clipe - a receita da locomocao (03-04/11): com o levantar guiado, o clipe de levantar
    // inteiro nos motores e a cabeca da locomocao corrigindo por cima (a correcao residual), a nota do rastreio e a mao de
    // fora na pelve e no corpo na forca do exercicio GetUpDrill, descendo pelo dominio; no fim do clipe, de pe, o idle.
    // (O guiado de 01/11 so misturava 30% do clipe e nao tinha mao: ele aprendeu a ficar deitado imitando o comeco.)
    bool getUpTracking = false;
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
    // um empurrao em movimento), 7 as quedas da locomocao (a do mundo entre as de setFallStarts), 8 o equilibrio parado
    // com empurroes, 9 e 10 o levantar deitado de costas e de brucos (com setLyingStarts).
    std::uint32_t evaluationKind = 0;
    // A professora da locomocao (03/11 18h27, ele: "gravar os movimentos que o b21 faz para se locomover nas 8
    // direcoes... FORCAR absolutamente o nosso rl_final a fazer EXATOS mesmos movimentos, e aprender com eles, e ai apos
    // ele dominar isso nos vamos afrouxando"; setLocomotionTeacher): nos episodios dos rumos, a fatia teacherShare com
    // ela no volante do comeco ao fim (o movimento EXATO dela; o resto, a aluna dirige com ela em sombra - o rotulo - e
    // retomando no desvio). Com ela os rumos saem de pe, num rumo sorteado de 0 a 7, sem paradas, trocas nem rampa, no
    // pedido da tabela: teacherSpeeds o pedido a ela (m/s, por rumo) e teacherStudentSpeeds o da aluna (o que ela faz).
    float teacherShare = 1.0f;
    // Os rumos como no treino com a professora, mesmo sem ela (o laboratorio e a prova da aluna sozinha): de pe, no
    // pedido teacherStudentSpeeds, sem rampa, fase, paradas nem trocas. (03/11 20h30, ele: o giro parado e o parado tambem
    // - "ele precisa aprender melhor a ficar na pose IDLE ALERTA quando parado": com isto ou com a professora, o giro e o
    // parado sao da cabeca da locomocao, com o musculo dela, e a professora ensina os dois.)
    bool taughtHeadings = false;
    // O parado tambem ensinado (da cabeca da locomocao). 03/11 22h35, ele: "ate o parado esta caindo... sendo que ele JA
    // SABE se equilibrar parado": o parado ensinado tirava o parado da cabeca que sabe (a recuperacao, com o musculo dela).
    // Falso: o parado e da recuperacao, como antes.
    bool taughtStanding = false;
    // No giro parado com a professora, a media do peito que dispara a retomada (graus; ela mesma atrasa girando).
    float teacherTakeoverTurnChest = 40.0f;
    // O parado ensinado com o proprio IDLE ALERTA (03/11 20h50: a B21 parada mede pose 0,64 e coluna 0,27 contra ele - a
    // rl_final de antes, ~0,67 e ~0,69 -; e o clipe sozinho nos motores, sem equilibrio, desaba em ~3 s): a professora
    // hibrida - as pernas da B21 (o equilibrio) e o tronco, os bracos e a cabeca na pose do clipe (o musculo do tronco
    // dela e o da aluna). Falso: a B21 inteira tambem no parado.
    // 0: a B21 inteira; 1: o tronco, os bracos e a cabeca no clipe; 2: todas as juntas no clipe, com as antecipacoes de
    // contato dela nas pernas (o equilibrio dela).
    int teacherStillClip = 2;
    // Nos episodios da aluna, a professora dirige o comeco (s, sorteado; 0: nao) e entrega no embalo dela.
    float teacherLeadMinimum = 0.0f;
    float teacherLeadMaximum = 0.0f;
    // A professora em sombra depois de entregar (o rotulo em sombra e a retomada). Falso e sem retomada: ela para de rodar
    // quando entrega - o rotulo em sombra nao e usado com a imitacao so no volante (22h25: as decisoes por segundo).
    bool teacherShadow = true;
    std::array<float, 8> teacherSpeeds { 3.0f, 3.0f, 1.5f, 1.5f, 1.5f, 1.5f, 1.5f, 3.0f };
    std::array<float, 8> teacherStudentSpeeds { 1.9f, 1.45f, 1.05f, 1.2f, 1.25f, 1.2f, 1.1f, 1.45f };
    // A intervencao (a aluna no volante; o revisor de 03/11: na queda iminente de 25 graus a propria B21 ja cai): o
    // tronco alem de teacherTakeoverTilt (graus) ou tombando mais rapido que teacherTakeoverTiltRate (graus/s), o peito
    // alem de teacherTakeoverChest da vista (graus, media de ~0,25 s), ou o deslocamento alem de teacherTakeoverTravel do
    // rumo (graus, acima de 0,5 m/s): ela assume por teacherTakeoverSeconds (0: sem intervencao).
    float teacherTakeoverTilt = 15.0f;
    float teacherTakeoverTiltRate = 45.0f;
    float teacherTakeoverChest = 25.0f;
    float teacherTakeoverTravel = 20.0f;
    float teacherTakeoverSeconds = 1.0f;
    // shortcut: sem efeito desde a limpeza de 10/11 (a locomocao e sempre o rastreio); so o Workbench
    // (TrackingViewer3D.cpp) ainda o escreve. Sair junto com essa linha dele.
    bool locomotionTracking = true;
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
    int sceneryPreset = -1;
    // A professora (setLocomotionTeacher): a fracao das decisoes com ela no volante, quantas vezes ela retomou, as
    // decisoes rotuladas e a maior ajuda de fora dela (N; tem de ser 0). Nos rumos, o desvio do deslocamento medio contra
    // o rumo pedido (graus, + a esquerda).
    float teacherShare = 0.0f;
    std::uint32_t takeovers = 0;
    std::uint32_t labeled = 0;
    float teacherAssist = 0.0f;
    float travelError = 0.0f;
    // (04/11) Por que acabou: 0 o tempo, 1 o tronco no chao, 2 a perda do equilibrio sem tocar o chao (cabeca abaixo de
    // 1,1 m ou pelve abaixo de 0,5 m), 3 o corpo virado alem do rumo do clipe, 4 o rastreio ruim, 5 outro (parceiro).
    std::uint32_t endCause = 0;
    // (04/11) As medias da semelhanca com o clipe e da estabilidade da nota do rastreio no episodio (0-1; -1 sem rastreio).
    float meanImitation = -1.0f, meanStability = -1.0f;
    // (04/11 15h, os passos) Os passos de recuperacao e de estabilizacao do gerador, se foi o equilibrio parado (com
    // empurroes), quantos empurroes, o pior ponto de captura fora da base de pe no idle (m) e quando ficou quieto de novo
    // depois do ultimo empurrao (s; < 0 nao ficou).
    std::uint32_t recoverySteps = 0, settleSteps = 0;
    bool balanceDrill = false;
    std::uint32_t pushes = 0;
    float worstCaptureOut = 0.0f;
    float quietAgainSeconds = -1.0f;
    // A qualidade da chegada na entrega ao idle depois de levantar (0-1; < 0 sem entrega).
    float arrivalQuality = -1.0f;
    // (05/11 23h) Na corrida para a frente e nas diagonais da frente, a fracao dos apoios com o pe apoiado mais de 3 cm para
    // dentro que o mesmo pe do clipe - o passo cruzado (< 0: sem apoios medidos).
    float footCross = -1.0f;
    // (08/11) Na estrela, as pernas feitas e a fase; e se o episodio acabou caindo durante um empurrao ou ate 2 s depois.
    std::uint32_t starLegs = 0;
    int starPhase = 0;
    bool fellInPush = false;
    // (08/11) A distancia da pelve ao ponto de parada de referencia no fim (m; < 0 sem parada).
    float stopError = -1.0f;
    // (12/11 03h20) No levantar pelo clipe, os segundos do clipe percorridos ate a entrega e o ponto dela (< 0 sem entrega): com o
    // tempo real da entrega (handoffSeconds), o tempo para levantar.
    float getUpCovered = -1.0f, getUpTarget = -1.0f;
    // (12/11 04h40, a fase de fogo do levantar) O cenario (0 limpo, 1 pancada, 2 continuo, 3 os dois), o momento da pancada (1 comeco,
    // 2 meio, 3 quase fim), o rumo (0-7 a partir da frente do corpo), a fracao sorteada do nivel, o tamanho (o continuo em fracao do
    // peso; a pancada em m/s ao centro de massa), se foi ao chao depois da pancada, quando ficou firme de pe (s; < 0 nunca) e o
    // instante do clipe do comeco (s).
    float fireKind = 0.0f, fireTiming = 0.0f, fireDirection = 0.0f, fireFraction = 0.0f, fireSize = 0.0f, fireDown = 0.0f;
    float upSeconds = -1.0f, clipStartSeconds = 0.0f;
    // (12/11 08h20, ele: "tentar se levantar novamente, APOS fracassar nas tentativas anteriores (nunca desistir)") As tentativas do
    // levantar adaptativo (1 = so a primeira; settings.adaptiveRetry).
    float attempts = 1.0f;
    // (12/11 09h45) O tempo da ultima tentativa ate levantar (s), so quando levantou numa tentativa nova; -1 sem.
    float retrySeconds = -1.0f;
    // (12/11 12h30, fase 2) As adversidades novas do episodio (bits: 1 pancadas em serie, 2 rasteira, 4 prensado, 8 vento que muda,
    // 16 a pancada logo depois de ficar de pe, 32 o chao escorregadio, 64 o agarrao, 128 o comeco em pose variada).
    float fireExtras = 0.0f;
    // (12/11 16h30, o fogo da locomocao - settings.locoFireShare) Se teve fogo, o nivel (L), quantas adversidades vieram e a fracao do
    // tempo sob fogo (desde a primeira) em que ele seguia a tarefa: andando, no rumo pedido (30 graus) a >= 60% da velocidade pedida;
    // parado, de pe e quase parado (< 0,5 m/s); -1 sem.
    float locoFire = 0.0f, locoFireLevel = 0.0f, locoFireEvents = 0.0f, locoFireKeep = -1.0f;
    // (12/11 17h, settings.crookedStartShare) Se nasceu torto e quando reajustou (s; -1 nao reajustou).
    float crooked = 0.0f, crookedFixedAt = -1.0f;
    // (12/11 17h20, a estrela) A distancia media ao caminho da perna (m; -1 fora da estrela) e as pernas em curva feitas.
    float starPathError = -1.0f, starCurves = 0.0f;
    // (12/11 18h, settings.commandGetUp) No comando: as quedas, os levantares de volta e o tempo medio deles (s; -1 sem).
    float commandFalls = 0.0f, commandRises = 0.0f, commandRiseSeconds = -1.0f, commandRunRises = 0.0f;
    // (12/11 19h10, settings.getUpGoShare) O levantar e ir: 1 de pe, 2 correndo (0 fora dele), quando chegou ao ponto (s; -1 nao chegou)
    // , a distancia do ponto no comeco e a que faltava no fim (m).
    float getUpGo = 0.0f, goArrivedAt = -1.0f, goDistance = -1.0f, goLeft = -1.0f;
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
    // (04/11 11h50, ele: "corrigir o problema de nao aparecer as marcacoes no cenario das curvas") O caminho pedido a frente
    // nas curvas (pontos no chao a cada ~1 m: o arco que falta e a reta ate o proximo, que e sorteado).
    std::vector<Vec3> path;
    // (08/11) A estrela: o centro e as 8 pontas (o centro primeiro), o ponto de agora (-1 o centro) e o seguinte, a perna, a
    // fase, o peito (8 fixo, 9 livre, 10 girando), o angulo da proxima troca (graus) e a velocidade de chegada pedida (m/s).
    bool star = false;
    std::vector<Vec3> starPoints;
    int starFrom = -1, starGoal = -1, starNext = -1, starLegs = 0, starPhase = 0, starChest = 0;
    float starTurnDegrees = 0.0f, starArrive = 0.0f;
    // (12/11 17h20) O caminho da perna de agora e o da seguinte (retas: 2 pontos; curvas: o arco em pontos) e a flecha delas (0 reta).
    std::vector<Vec3> starLegPath, starNextPath;
    float starBend = 0.0f, starNextBend = 0.0f;
    // (08/11 14h40) O comando de agora (o jogador emulado), em texto: as teclas, o shift, o estado e a vista.
    std::string commandText;
    // (12/11 05h30, ele: "se eu marcar a checkbox Empurroes eu devo ver os treinos que ocorrem com os empurroes/pancadas... com um
    // visual muito mais bonito... que mostra de onde vem a pancada/empurrao, a forca, direcao") O fogo do levantar deste mundo
    // (TrackingGetUpAmpEpisode3D::fireKind): o tipo, o momento, o rumo (0-7; +8 de cima) e a inclinacao (rad), a fracao do nivel e o
    // tamanho (fracao do peso; m/s da pancada), a forca continua de agora (N, mundo, ja na rampa), a pancada (a hora, se ja veio, o elo,
    // a forca em N, quanto dura) e o rumo dela agora (unitario, para o aviso antes), o tempo do episodio, o fim do orcamento, quando
    // levantou firme, quando entregou, quando foi ao chao, o peso (N) e os niveis do exercicio (o continuo, a pancada do momento).
    bool clipGetUp = false;
    int fireKind = 0, fireTiming = 0, fireDirection = 0, fireShoveLink = -1;
    float fireAbovePitch = 0.0f, fireFraction = 0.0f, fireSize = 0.0f, fireShoveSize = 0.0f;
    Vec3 fireHold, fireShoveForce, fireAim;
    float fireShoveAt = -1.0f, fireShoveSeconds = 0.0f;
    bool fireShoveDone = false;
    float time = 0.0f, fireBudget = 0.0f, upAt = -1.0f, deliveredAt = -1.0f, downAt = -1.0f, weight = 0.0f;
    float fireLevelHold = 0.0f, fireLevelShove = 0.0f;
    // (12/11 12h50, a fase 2) O agarrao puxando agora (-1 sem) e o chao escorregadio.
    int fireGrabLink = -1;
    Vec3 fireGrabForce;
    bool fireSlip = false;
    // (12/11 16h30) O fogo da locomocao neste mundo (os campos do fogo acima valem para ele tambem; sem o levantar: o orcamento e o
    // episodio, nada de "levantou") e o nivel L dele.
    bool locoFire = false;
    float locoFireLevel = 0.0f;
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
    // (12/11 06h40) As entradas do fogo (TrackingGetUpAmpSettings3D::fireObservation).
    static constexpr std::size_t FireObservationSize = 6;
    // Os clipes do estilo: os de levantar (e os espelhados) tambem dao os comecos; o parado so o estilo.
    static constexpr const char* GetUpClips[4] { "stand_up_front_kimodo", "stand_up_back_kimodo",
        "stand_up_front_kimodo_espelho", "stand_up_back_kimodo_espelho" };
    // (31/10 18h, ele: o padrao de ficar de pe parado e o parado em alerta, nao o natural; com o espelho, o apoio no
    // outro pe.) (07/11, ele: "o pose de pe passa a ser a parada da Unreal": a base parada do Game Animation Sample, em
    // ciclo e no lugar, e o espelho dela.)
    // (08/11 18h05, ele: "vamos trocar a pose do parado IDLE para o nosso antigo IDLE ALERTA... ao inves do da Unreal, pois eu
    // sentia que aquela pose tinha uma base mais forte".)
    static constexpr const char* StandingClip = "alert_idle";
    static constexpr const char* StandingMirrorClip = "alert_idle_espelho";
    // Os clipes do estilo do levantar em movimento (a arrancada e a corrida; no lugar, com a velocidade nominal somada
    // aos pares do discriminador): so o estilo e os comecos das quedas.
    static constexpr const char* MovingClips[5] { "idle_to_sprint", "crouched_to_sprint", "mixamo_running",
        "run_forward_medium", "sprint_forward" };
    // Os clipes do trote, os do modo padrao do jogo (character.json; 03/11: os trot_* do trot.py, rejeitados em 05/10,
    // sairam): a frente, a esquerda, de costas e a direita. As diagonais usam o da frente ou o de costas com o corpo
    // virado para o rumo, como o modo padrao.
    static constexpr const char* HeadingClips[4] { "jog", "jog_strafe_left", "jog_backward", "jog_strafe_right" };
    // (01/11 19h, ele: "por que temos um exercicio de andar, sendo que nao usamos andar no jogo? apenas trote, corrida e
    // sprint": os clipes de andar sairam do estilo e dos passos; ficam os giros parados.)
    static constexpr const char* LocomotionStyleClips[2] { "turn_left", "turn_right" };
    // (04/11 09h50) O giro parado do rastreio: o do modo padrao, do idle alerta (os passos do StandTurnFeet3D).
    static constexpr const char* TrackedTurnClips[2] { "giro_idle_esquerda", "giro_idle_direita" };
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
    // (05/11 03h15, ele: a corrida de costas "mais semelhante com o trote costas... o clipe atual deixa a base muito
    // baixa": run_backward_trote no lugar do sprint_backward.)
    // (04/11, ele: "adicionar os treinos de CORRIDA nas 8 direcoes" - no rastreio, a faixa da corrida (~4,1-4,4 m/s): a
    // frente, a esquerda, de costas e a direita; as diagonais pela cintura, como no trote. So rastreados: fora do juiz
    // de estilo da recuperacao.)
    static constexpr const char* TrackedRunClips[4] { "run_forward_medium", "sprint_strafe_left", "run_backward_trote",
        "sprint_strafe_right" };
    // (08/11, a velocidade: a rede corria na velocidade do clipe dos motores - o trote de frente ~3,5 m/s com o jog de 3,0; a
    // corrida ~4,3 com a run_forward_medium de 4,33 - contra metas de 4,5 e 6,0) Os ciclos nas velocidades-meta
    // (tools/animation/clips/ciclos_meta.py, os nossos regerados na meta): trote frente, esquerda, costas, direita; corrida
    // frente, esquerda, costas, direita. Com setGaitCycles, no lugar dos passos do trote e da corrida.
    static constexpr const char* CycleClips[8] { "ciclo_trote_frente", "ciclo_trote_esquerda", "ciclo_trote_costas",
        "ciclo_trote_direita", "ciclo_corrida_frente", "ciclo_corrida_esquerda", "ciclo_corrida_costas", "ciclo_corrida_direita" };
    static constexpr const char* SprintClips[8] { "sprint_forward", "mixamo_sprint", "sprint", "sprint_maximo",
        "sprint_backward", "sprint_backward_long", "sprint_strafe_left", "sprint_strafe_right" };
    // As entradas do comando, depois das do corpo: o rumo do deslocamento pedido em relacao ao corpo (cos, sen), a
    // distancia (/10 m, ate 1; nos rumos, 1), a velocidade pedida (/5 m/s; 0: ficar onde esta) e o rumo pedido do corpo
    // em relacao a ele (cos, sen: no levantar em movimento, o alvo; nos rumos, o do comeco; no giro, o que gira).
    static constexpr std::size_t CommandSize = 6;
    // Depois do comando, o corpo contra o chao (o contrato v3, TrackingGetUpContract3D::observeGround).
    static constexpr std::size_t GroundObservationSize = TrackingGetUpContract3D::GroundObservationSize;
    // Depois do chao, a fase do clipe do passo (sen, cos de 2 pi t/T; zeros sem clipe): a entrada de fase do DeepMimic,
    // sem a qual a imitacao no tempo do clipe nao tem como sincronizar (01/11 18h, o layout levantar-v4).
    static constexpr std::size_t PhaseSize = 2;
    static constexpr std::size_t SkillModeSize = 1;
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
    // O rastreio da ultima decisao de cada mundo (8 por mundo; zeros fora da locomocao): a nota, os erros da pelve, das
    // pernas (pior), do tronco, dos bracos, dos pes (pior), da velocidade e a velocidade no rumo pedido (m/s).
    // (04/11 16h30, mais 9: os pilares da firmeza, a margem de Hof (m), o momento angular alem do clipe, a firmeza, o
    // escorregao no ponto de contato (m/s) e o tropeco.)
    // (08/11 23h55, mais 1: o portao da velocidade daquela nota - 0 quando a nota andando nao foi calculada.)
    // (09/11 01h20, mais 1: o joelho para dentro alem do clipe, m - 0 quando nao medido.)
    static constexpr std::size_t TrackingFeedbackSize = 36;
    void locomotionFeedback(std::span<float> out) const;
    // O alvo do clipe do passo (a acao que reproduz o quadro do fim da decisao; zeros sem clipe): o teste de mergulho.
    void locomotionTargets(std::span<float> out) const;
    // (05/11, a perda de simetria) O espelho de cada mundo, 2 por mundo: se o estado de agora tem espelho (1/0: a locomocao
    // dirigindo um rumo reto sem manobra, num clipe periodico com espelho, fora da arrancada e das transicoes) e o
    // deslocamento da fase do clipe no espelho (pi nos clipes que servem aos dois lados; 0 nos laterais, cujo par e o
    // espelho exato no mesmo instante). Ler junto com as observacoes (antes do step).
    static constexpr std::size_t MirrorInfoSize = 3;
    void mirrorInfo(std::span<float> out) const;
    // A rede da locomocao parceira (vazia: sem). Troca so entre os steps.
    void setLocomotionPartner(std::shared_ptr<const LocomotionPartner3D> partner);
    // A professora da locomocao (begin/tick; nula: sem ela) e a fatia dos rumos com ela no volante (o beta, mudavel no
    // treino).
    void setLocomotionTeacher(std::shared_ptr<const LocomotionPartner3D> teacher);
    void setTeacherShare(float share);
    // A forca auxiliar da locomocao (TrackingGetUpAmpSettings3D::locomotionAssist), mudada pelo treinador.
    void setLocomotionAssist(float assist);
    void setBalanceFeet(bool on);
    void setTrackingReward(int version);
    void setPhaseSync(bool on);
    void setImitationLoosen(float factor);
    void setFacingGate(float gate);
    void setArmClipReach(float reach);
    void setLinearHand(bool on);
    void setUpperHand(bool on);
    void setBodyHand(bool on);
    void setIdle(bool afterStand, bool onLocomotion, float residualScale = 1.0f);
    void setIdleImitation(float weight);
    void setIdleHandoff(float blendSeconds, float blendReach);
    void setRiseHandoff(float headHeight);
    void setRecoveryNatural(float weight);
    void setLateralHandPush(bool push);
    void setLateralRunLegReach(float scale);
    void setGaitSpeeds(std::array<float, 8> trot, std::array<float, 8> run);
    void setBackStartLean(float radians);
    void setCourseLimit(float meters);
    void setCurveLean(bool lean);
    void setFootWidth(float slackMeters);
    void setRunTurnBrake(bool brake);
    void setRunTurnAlign(bool align);
    void setTurnSettle(bool settle);
    void setTurnLaunch(bool launch);
    void setRunLaunchEase(float scale, float easeSeconds, float acceleration);
    void setCourseTail(float weight);
    void setSpeedRange(float low);
    void setIdleQuiet(float weight);
    void setManeuverAcceleration(float acceleration);
    void setRiseOvershoot(float weight);
    void setRollTail(float weight);
    void setCliffFree(bool on);
    void setCliffParts(bool curveRoll, bool footCross, bool forearm, bool hop);
    void setLaunchRise(float spanMeters);
    void setLateralStart(float acceleration);
    void setRiseStiffness(float scale);
    // (06/11) A velocidade-alvo dos motores pelo proprio alvo da cabeca da locomocao, sem clipe (freeVelocityFeedforward).
    void setFreeVelocityFeedforward(bool on);
    void setFreeDampingScale(float scale);
    // (06/11 17h40) A velocidade-alvo na acao (velocityActions); muda o tamanho e os nomes da acao - chamar antes de usa-los.
    void setVelocityActions(bool on);
    void setRecoveryVelocity(bool on);
    void setManeuverWeights(std::array<float, 7> weights);
    static constexpr float VelocityActionLimit = 12.0f;
    void setTurnSettleLateral(float speed);
    void setTurnSettleFront(float speed);
    void setRunTurnTimeScale(float scale);
    void setStopGaitAll(bool on);
    void setIdlePoseGate(float gate);
    void setStopClipsFront(bool on);
    void setStopClipsCommand(bool on);
    void setStopGateFloor(float floor);
    void setFreeStop(bool on);
    void setStopPosture(float gate);
    void setStopIdlePose(float weight);
    void setIdleUnified(bool on);
    void setStopToIdle(bool on);
    void setIdleAfterStop(bool on);
    void setParkedOnLocomotion(bool on);
    // (12/11 16h30, o fogo da locomocao - ver TrackingGetUpAmpSettings3D::locoFireShare.)
    void setLocomotionFire(float share, std::array<float, 6> levels, float relief);
    void setLocomotionFireGaps(float scale);
    void setCrookedNoFire(bool on);
    void setCrookedFixSpan(float seconds);
    void setLocomotionFireExam(bool on, float level);
    void setCrookedStart(float share, float bonus, float close);
    void setStarRework(float pathWeight, float pathWidth, float curveShare, float curveMin, float curveMax);
    void setCommandGetUp(bool on, float runShare = 0.5f);
    void setFallToCommand(bool on);
    void setRunHandoffAtStand(bool on, float afterSeconds);
    void setGetUpGo(float share, float minMeters, float maxMeters);
    void setTurnVariety(bool on, float spineReward);
    void setStillCost(float weight);
    void setIdleStatic(bool on);
    void setIdleCosts(float motion, float pose);
    void setIdleReward(float pose, float still, float slide);
    void setIdleStillBase(float gate);
    void setTurnedGrace(float seconds);
    void setStopArmsReach(float reach);
    void setGetUpImitation(int mask, float paceSeconds);
    void setGetUpImitationPaceRun(float seconds);
    // (15/11) settings.getUpViewReward e, para as sondas, de um mundo: {a vista menos o peito (graus), a altura da cabeca (m), 1 se a vista vale
    // no levantar agora, 1 se ja trocou, 1 se e o levantar e ir, 1 se na locomocao}; e settings.getUpProneTimeScale.
    void setGetUpView(float weight);
    [[nodiscard]] std::vector<float> getUpViewNow(std::size_t world) const;
    void setGetUpProneTimeScale(float scale);
    void setHandBehind(float weight);
    void setGetUpStandBonusScale(float scale);
    void setGetUpImitationReward(float weight);
    void setGetUpImitationFar(float threshold, float seconds);
    void setGetUpImitationAnchor(bool enabled);
    void setClipUnderPelvis(bool enabled);
    void setIdleClipAnchor(bool enabled);
    void setLegAlign(float weight, float widthDegrees);
    void setIdleHold(float weight, float widthMeters);
    void setReviewFixes(bool enabled);
    void setLaunchSideMargin(bool enabled);
    void setRunWidth(float weight);
    void setIdleWeightCenter(float weight);
    void setSpeedWidthFraction(float fraction);
    void setLaunchArmsCalm(float weight);
    void setLegAlignCommand(bool enabled);
    void setLocomotionScenery(float fraction);
    void setReferenceSideFix(bool enabled);
    void setPhaseSyncBothFeet(bool enabled);
    void setHeadingBias(float weight, float width);
    void setContactForceObservation(bool enabled);
    void setStraightCommand(bool enabled);
    void setLateralBias(float weight, float width);
    void setImitationGate(float gate, float full);
    // (17/11, D2) A fatia dos mundos com o chao de grama (o do campo); os outros ficam no concreto (o do laboratorio). Recria o chao.
    void setGrassGroundFraction(float fraction);
    void setClipSideNow(bool enabled);
    void setClipTrackMask(int mask);
    void setUpperRigor(float factor);
    void setUnrealStarts(bool on);
    void setUnrealStartScale(float scale);
    void setLiveLaunch(bool on);
    void setLaunchStepReward(float weight);
    void setLaunchClipShare(float share);
    void setLaunchHand(float level);
    void setLaunchInPlacePenalty(float weight);
    void setLaunchCalmArms(bool on);
    void setSpineVelocityFeedforward(bool on);
    void setBasePosture(float weight);
    void setMotorTone(float tone);
    void setPostureTail(float share);
    void setSpineTone(float tone);
    void setArmTone(float tone);
    void setClipVelocityRotation(bool enabled);
    void setImitationVelocityTargets(bool enabled);
    void setProjectiles(float rate, float minimumImpulse, float maximumImpulse, float seconds);
    void setJamCost(float weight);
    void setFootSlipCost(float weight);
    void setMotorGravity(bool on);
    void setStopClipsBase(bool on);
    void setStopFrontVariants(bool on, int variant);
    void setStopFrontVariantMask(int mask);
    void setStopEntry(float blendSeconds, float windowSeconds);
    void setFreeTroca(bool on);
    void setFreeStandGate(float gate);
    // shortcut: sem efeito desde a limpeza de 10/11; so o Workbench (TrackingViewer3D.cpp) ainda chama. Sair junto com ele.
    void setCommandChestGate(float) {}
    void setFreeIdle(bool on);
    void setPoseBoost(float gate);
    void setFreePush(bool on);
    void setFreeIdleMatch(float weight);
    void setFreeStepHeight(float meters);
    void setIdleBentEnd(bool on);
    void setFinalRow(bool on);
    void setIdleLikeStop(bool on);
    // shortcut: sem efeito desde a limpeza de 10/11; so o Workbench (TrackingViewer3D.cpp) ainda chama. Sair junto com ele.
    void setUpperImitation(bool) {}
    void setIdleWide(bool on);
    void setMirrorCycles(bool on);
    void setTurnSettleBack(float speed);
    void setTrocaFixedChest(bool on);
    void setTrocaPractice(bool practice, std::array<float, 8> headingWeights);
    void setFootCross(float slackMeters);
    void setIdleReanchor(bool reanchor);
    void setAccelLean(float radians);
    void setFrontLean(float radians, float startRadians);
    void setRunLean(float radians, float legs);
    void setRunBackLean(float radians, float legs);
    void setGetUpTracking(bool on);
    void setSprint(float speed, float leanRadians, float legs);
    // (04/11 15h) O gerador de passos no idle e o idle seguindo nos empurroes; o equilibrio parado (a fatia e o nivel).
    void setBalanceSteps(bool steps, bool throughPushes);
    void setBalanceDrill(float fraction, float pushLevel);
    // (08/11, a fase de fogo) Os empurroes em todos os exercicios, de pe e em movimento: a fatia dos episodios do treino com
    // eles, as pancadas (N, 0,1-0,2 s), os continuos (N), a duracao deles (s) e a fatia dos continuos.
    void setFire(float episodeShare, std::array<float, 2> hitNewtons, std::array<float, 2> holdNewtons,
        std::array<float, 2> holdSeconds, float holdFraction);
    // (08/11) A estrela de 8 pontas: a fatia dos episodios de rumo, os pesos e as fases dos 6 e a duracao do episodio (s).
    void setStar(float fraction, std::array<float, 6> weights, std::array<int, 6> phases, float seconds);
    // (08/11) O comando: a fatia dos episodios de rumo, a fase (1-4) e a duracao (s).
    void setCommand(float fraction, int phase, float seconds);
    // (08/11) A parada no fim dos retos: a fatia, a janela (s), a freada (m/s^2) e se vale na avaliacao.
    void setFinalStop(float fraction, float earliest, float latest, float deceleration, bool inEvaluation);
    void setStopClips(bool on);
    void setFireShown(bool on);
    void setDiagonalChest(float gate);
    void setCourseSmoothing(float seconds);
    void setCourseAngleLimit(float radians);
    void setLocomotionShaping(bool on);
    void setSpeedGate(float floor);
    void setKneeInward(float slackMeters);
    void setIdleHeight(float slackMeters);
    void setStopWaitBody(bool on);
    // (08/11) Os passos do trote e da corrida pelos ciclos nas velocidades-meta (CycleClips) ou pelos clipes de antes.
    void setGaitCycles(bool on);
    void setPushVariety(bool on);
    void setFireGroups(const std::vector<float>& bands);
    void setLateralLegStrict(float factor);
    void setPushImitationRelief(bool on);
    void setRiseFlight(float penalty);
    void setSlowLimit(float fraction);
    void setSlowLimits(std::vector<float> fractions);
    void setHeadClipReach(float reach);
    void setStepTuning(float loadLimit, float loadWeight, bool swingFromBody);
    void setGetUpArrival(float bonus, float flightPenalty, float riseBalance = 0.0f, float calmHandoff = 0.0f);
    void setAvoidOnIdle(bool on);
    void setIdleThreatReach(float reach);
    // O tamanho do estado do equilibrio na observacao (0 sem balanceObservation) e dos canais do passo (0 sem stepChannels).
    [[nodiscard]] std::size_t balanceObservationSize() const;
    [[nodiscard]] std::size_t stepChannelCount() const;
    // Os eixos dos motores (o comeco da acao; depois vem os canais do passo e, com velocityActions, as velocidades).
    [[nodiscard]] std::size_t axisCount() const;
    void setRunAssist(float assist);
    void setManeuvers(float fraction, int evaluationManeuver);
    void setManeuverAssist(float assist);
    void setDrillAssist(std::span<const float> levels);
    // A aluna-oraculo (o teste da exatidao): 0 a rede; 1 o rotulo da professora em sombra nos motores da aluna no tick 0
    // de cada decisao, segurado (como a rede a 30 Hz); 2 a cada tick.
    void setTeacherOracle(int mode);
    void setTeacherLead(float minimum, float maximum);
    void setTeacherShadow(bool shadow);
    // O parado ensinado: 0 a B21 inteira, 1 o tronco no IDLE ALERTA, 2 todas as juntas nele com o equilibrio dela.
    void setTeacherStillClip(int mode);
    // O musculo da cabeca da locomocao (03/11: o do rastreador - o perfil a 1x, sem gravidade - nao reproduz a B21 nem
    // com o rotulo perfeito): as escalas por eixo da acao (rigidez, amortecimento e teto, as do backend) e a compensacao
    // de gravidade (torque das juntas, como na B21 a 0%). Vazio: o do rastreador.
    void setStudentActuator(std::span<const float> stiffness, std::span<const float> damping,
        std::span<const float> maximumTorque, bool gravity);
    // As escalas medias dos drives da professora por eixo da acao: [eixo][rigidez, amortecimento, teto, gravidade,
    // |antecipacao| N.m].
    [[nodiscard]] std::vector<float> teacherDriveStats() const;
    // Os pedidos por rumo (a ela e a aluna, m/s) e a intervencao (teacherTakeoverTilt, ...TiltRate, ...Chest, ...Travel,
    // ...Seconds) - os campos teacher* dos ajustes; valem nos proximos comecos.
    void setTeacherSettings(std::array<float, 8> speeds, std::array<float, 8> studentSpeeds, std::array<float, 5> takeover);
    // O rotulo da ultima decisao de cada mundo (teacherTargetWidth() por mundo): o alvo da aluna que daria o torque da
    // professora (actionSize; a media dos 4 ticks), o alvo dela cru (actionSize, a media), se vale, se ela estava no
    // volante e se retomou nesta decisao. (Guardado fora do recomeco: vale para a decisao que acabou de rodar.)
    [[nodiscard]] std::size_t teacherTargetWidth() const;
    void teacherTargets(std::span<float> out) const;
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
    // So os pares destes clipes (vazio: todos os da tarefa).
    [[nodiscard]] std::vector<float> referencePairsOf(const std::vector<std::string>& clips) const;
    // Carrega clipes so para o juiz (ids da pasta de clipes, com subpasta), no fim da lista; devolve quantos entraram.
    std::size_t addStyleClips(const std::vector<std::string>& names);
    // Os comecos deitados (07/11): o comeco no clipe (1) no quadro 0 de um destes (na avaliacao 9 o primeiro, 10 o segundo:
    // de costas, de brucos) e o levantar em movimento (3) no quadro 0 de um destes; carregados antes (addStyleClips).
    void setLyingStarts(const std::vector<std::string>& standClips, const std::vector<std::string>& runClips);
    // (11/11 14h) O levantar pelo clipe: os quatro clipes, ja carregados (addStyleClips) - de costas, de brucos, correndo de costas,
    // correndo de brucos - e as opcoes (ver clipGetUpFraction). clipGetUpEnds: de cada um, a duracao, a velocidade do centro de
    // massa e o rumo da raiz no fim (no quadro do clipe).
    void setClipGetUp(const std::vector<std::string>& clips, float fraction, float midStartFraction, float afterSeconds,
        float startNoise, float softness, float comVelocityWidth);
    [[nodiscard]] std::vector<float> clipGetUpEnds() const;
    void setClipGetUpWeights(const std::vector<float>& weights);
    // (18h05) A variante de cada exercicio na entrada levantar.variante (clipVariantObservation; vazio: zero).
    void setClipGetUpVariants(const std::vector<float>& variants);
    // (12/11) O ponto de entrega de cada exercicio (s do clipe; <= 0: no fim dele): ali a estatistica conta a troca e o clipe segue.
    void setClipGetUpDeliver(const std::vector<float>& seconds);
    // (12/11) O reforco da pose do tronco e dos bracos por exercicio (TrackingClipGetUp3D::Widths::emphasis; 0 nada).
    void setClipGetUpEmphasis(const std::vector<float>& emphasis);
    // (12/11 03h20) O peso da agilidade por exercicio (TrackingClipGetUp3D::AgilityMaxRate; 0: o relogio do clipe so anda com o tempo).
    void setClipGetUpAgility(const std::vector<float>& agility);
    // (12/11 04h40) O levantar adaptativo (settings.clipGetUpAdaptive).
    void setClipGetUpAdaptive(bool on);
    void setAdaptiveSpeedBonus(float bonus);
    void setAdaptiveTiming(float timeMinimum, float timeMaximum, float runImpulse);
    void setAdaptiveTimingRun(float timeMinimum, float timeMaximum);
    void setAdaptiveTimeFall(float seconds);
    void setAdaptiveTimeFire(float slack);
    void setGetUpForm(float weight);
    void setGetUpPace(float weight, float seconds);
    void setGetUpPalm(float weight);
    // (14/11 20h10) A forma da perna esquerda (settings.getUpLegWeight) e, para as sondas, o erro dela agora num mundo (m; < 0: sem).
    void setGetUpLegForm(float weight, std::uint32_t mask);
    [[nodiscard]] float getUpLegErrorNow(std::size_t world) const;
    // (O corpo - cabeca, pelve, joelho e pe esquerdos -, a media do clipe comparada, n e o erro.)
    [[nodiscard]] std::vector<float> getUpLegFormNow(std::size_t world) const;
    // (14/11 20h40) O custo da inclinacao na largada (settings.launchLeanWeight) e, para as sondas, agora num mundo: a inclinacao do
    // corpo, a do clipe (graus, no rumo do movimento) e o peso da janela da largada.
    void setLaunchLean(float weight);
    void setLaunchSpeedShare(float share);
    // (15/11) settings.launchImitationReward e, para as sondas, a ultima nota da semelhanca com o clipe da largada de um mundo (-1: fora).
    void setLaunchImitation(float weight);
    [[nodiscard]] float launchImitationNow(std::size_t world) const;
    [[nodiscard]] std::vector<float> launchLeanNow(std::size_t world) const;
    void setIdlePelvisLevel(float weight);
    // (13/11 11h30) Para as sondas: a tabela da forma do clipe do exercicio (altura da cabeca, cabeca para a frente, de lado, torcao do
    // peito, peito contra a pelve, olhar para cima; 6 por amostra) e a forma de agora de um mundo (as mesmas 6 e a nota 0-1).
    [[nodiscard]] std::vector<float> getUpFormTable(std::uint32_t exercise) const;
    [[nodiscard]] std::vector<float> getUpFormNow(std::size_t world) const;
    void setSettleCalm(float weight);
    void setRunPostRise(float speedReward, float settleSeconds);
    void setGetUpPoseVariety(float fraction);
    void setCalmHandoff(float calmSpeed, float maxSeconds, float maxSecondsRun);
    void setAdaptiveSettle(float seconds, float release = 0.15f);
    void setGetUpFireReleaseAtFirm(bool on);
    void setAdaptiveRetry(bool on);
    void setFireRunHeadwindCap(float fraction);
    void setGetUpArena(float minimumMeters, float maximumMeters);
    void setGetUpFireEarly(float share, float window);
    void setGetUpFireExtras(int shoveSeries, float legShare, float wallShare, float wallMinimum, float wallMaximum, float turnShare,
        float turnSeconds, float afterStandShare = 0.0f, float slipShare = 0.0f, float grabShare = 0.0f, float grabFraction = 0.3f,
        float seriesGapMin = 0.4f, float seriesGapMax = 1.2f, float shoveSecondsMin = 0.1f, float shoveSecondsMax = 0.2f,
        float shoveNewtonsMin = 0.0f, float shoveNewtonsMax = 0.0f);
    // (12/11 04h40, a fase de fogo do levantar) As fatias dos cenarios (limpo, pancada, continuo, os dois; a quinta, se houver: a fracao
    // dos com fogo que vem de cima - 12/11 05h15), os niveis de cada exercicio
    // (4 por exercicio: o continuo em fracao do peso, a pancada do comeco, do meio e do quase fim em m/s ao centro de massa) e a prova
    // (o cenario fixo pelo mundo, no topo do nivel - as avaliacoes).
    void setGetUpFire(const std::vector<float>& shares, const std::vector<float>& levels, bool exam);
    // (18h22) O vies dos comecos no meio do clipe por exercicio: o expoente da fracao sorteada (1 uniforme; 2 metade antes de 1/4).
    void setClipGetUpStartBias(const std::vector<float>& bias);
    // (11/11 17h45) A nota do levantar pelo clipe por partes do corpo (TrackingClipGetUp3D::Widths::parts).
    void setClipGetUpParts(bool on);
    // (11/11 14h) A sonda da nota do levantar pelo clipe (a calibracao): o corpo no quadro do clipe em bodyTime (com a coluna e as
    // pernas variadas, erguido lift, a velocidade vertical somada, o deslocamento desde o comeco na escala travelScale e a
    // velocidade no plano em speedScale) contra a referencia em referenceTime: pose, velocidade, maos e pes, centro de massa,
    // velocidade dele, total e longe (1/0).
    [[nodiscard]] std::vector<float> clipGetUpProbe(std::uint32_t exercise, float referenceTime, float bodyTime, float lift,
        float verticalSpeed, float twist, float pitch, float legs, float travelScale, float speedScale, float armOffset = 0.0f) const;
    void setMovingHandoff(bool handoff);
    // (07/11) O juiz com o Motion Matching (vault 04_locomocao; o conjunto denso do Game Animation Sample): os grupos (nome,
    // esquema "default"/"stop"/"idle"/"unreal", os clipes - carregados antes -, os vieses base/ciclo/continuar e a categoria
    // da escolha: 0 parado, 1 giro no lugar, 2-6 partidas/ciclos/pivos/paradas/giro de 180 do trote, 7-10
    // partidas/ciclos/pivos/paradas da corrida) e o grupo principal (o contexto do juiz). Devolve o numero de quadros.
    std::size_t buildMotionMatching(const std::vector<std::string>& names, const std::vector<std::string>& schemas,
        const std::vector<std::vector<std::string>>& clips, const std::vector<std::array<float, 3>>& biases,
        const std::vector<int>& categories, std::size_t mainGroup);
    // (07/11, so para ver no laboratorio) O corpo dos exercicios de locomocao e do parado posto no quadro que o Motion
    // Matching escolhe a cada decisao, sem fisica (precisa de buildMotionMatching).
    void setKinematicMotion(bool on);
    // (07/11, so para ver no laboratorio) A sequencia de clipes da Unreal de cada exercicio, por regra, sem fisica (os clipes
    // carregados antes por addStyleClips).
    void setClipSequence(bool on);
    // (07/11, so para ver) A estrela de 8 pontas na sequencia de clipes: 0 desligada, 1 peito fixo, 2 peito livre.
    void setStarDrill(int mode);
    [[nodiscard]] std::size_t kinematicJumps(std::size_t world) const;
    [[nodiscard]] std::string kinematicClip(std::size_t world) const;
    // A previsao da trajetoria (m/s^2, 1/s, rad/s; rotacao negativa = na hora).
    void setMotionMovement(float maxAcceleration, float brakingDeceleration, float groundFriction, float brakingFriction,
        float rotationRadiansPerSecond);
    [[nodiscard]] std::size_t motionGroupCount() const;
    [[nodiscard]] std::size_t motionGroupEntries(std::size_t group) const;
    [[nodiscard]] std::size_t motionFeatureSize(std::size_t group) const;
    [[nodiscard]] std::size_t motionQueryOffset(std::size_t group) const;
    [[nodiscard]] std::size_t motionQueryWidth() const;
    [[nodiscard]] std::size_t motionMainGroup() const;
    // Os quadros de um grupo ja ponderados (quadros x tamanho), os pares do juiz deles (quadros x 2 ampSize) e, por quadro, o
    // viés, a fonte (o indice do clipe no grupo) e o instante.
    void motionGroupFeatures(std::size_t group, std::span<float> out) const;
    void motionGroupPairs(std::size_t group, std::span<float> out) const;
    void motionGroupEntryInfo(std::size_t group, std::span<float> addends, std::span<float> sources, std::span<float> times) const;
    // Por mundo, a consulta da ultima decisao (todos os grupos, ponderada; motionQueryOffset), os grupos permitidos (os bits,
    // como numero) e se vale: mundos x (motionQueryWidth + 2).
    void motionQueries(std::span<float> out) const;
    [[nodiscard]] std::vector<TrackingGetUpAmpEpisode3D> takeEpisodes();

    [[nodiscard]] const RagdollState3D& body(std::size_t world) const;
    // Os alvos de junta de agora de um mundo (os da ultima acao).
    [[nodiscard]] std::span<const RagdollDriveTarget3D> driveTargets(std::size_t world) const;
    [[nodiscard]] const RagdollProfile3D& profile() const;
    [[nodiscard]] std::string frameLines(std::size_t world) const;
    // (05/11 06h50, as ferramentas de inspecao - tools/rl/inspecao): a referencia seguida agora no mundo - o quadro do clipe
    // posto sob a pelve do corpo, como a nota o compara -, nas mesmas linhas "C"/"B" do frameLines, e uma linha "Q" com o
    // clipe, o instante dele, a forca auxiliar, quem dirige (1 a cabeca da locomocao), o pedido (m/s) e o rumo dele (rad).
    // Vazio sem clipe seguido.
    [[nodiscard]] std::string referenceLines(std::size_t world) const;
    // (11/11, ele: "o clipes do exercicio deveria mostrar o que o treino segue") A pose alvo que o treino segue agora no mundo (o
    // quadro de referencia posto no corpo, o mesmo da nota); false sem clipe seguido.
    [[nodiscard]] bool referencePose(std::size_t world, std::vector<PhysicsBodyState3D>& out) const;
    [[nodiscard]] TrackingGetUpAmpWorldView3D worldView(std::size_t world) const;
    // O cenario do mundo em arame (no mundo): as arestas das caixas (a rampa cortada em 2,5 m em volta do meio).
    void scenerySegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const;
    // (12/11 09h15) As paredes do levantar (settings.getUpArenaMax) em grade: o contorno inteiro, um poste a cada 2 m e tres linhas.
    void arenaSegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const;
    // (12h50) A placa da grama molhada em grade no chao (settings.fireSlipShare).
    void slipSegments(std::size_t world, std::vector<std::pair<Vec3, Vec3>>& out) const;
    // Os corpos soltos do mundo (do mesmo perfil do corpo; validos ate o proximo passo).
    void sceneryBodies(std::size_t world, std::vector<const RagdollState3D*>& out) const;

private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace MatterEngine
