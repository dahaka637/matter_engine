#pragma once

#include "Engine/Math/Quaternion.hpp"
#include "Engine/Math/Vec3.hpp"

#include <array>
#include <cstdint>
#include <span>
#include <vector>

// A rede de fios (a rede do gol): nos ligados por fios. O fio estica pouco e
// nao resiste a compressao (dobra e fica frouxo), entao cada fio e uma
// restricao de distancia de um lado so (nao passa do comprimento), com a
// complacencia do fio trancado. Os nos presos na trave nao se movem.
//
// Dinamica de posicoes estendida (XPBD; Macklin, Mueller e Chentanez 2016, e
// "Small Steps in Physics Simulation", 2019): muitos subpassos com uma
// iteracao cada. Cada no solto tem ainda uma amarra ate o no preso mais perto
// pelo caminho dos fios (Long Range Attachments; Kim, Chentanez e Mueller
// 2012), uma mola com a rigidez desse caminho (EA / comprimento) que so puxa
// quando o caminho ja esta esticado: com uma iteracao, a forca da trave chega
// a qualquer no no mesmo subpasso.
//
// Os corpos entram como proxies de um NetBody3D. A bola (esfera) e as
// capsulas (os membros dos jogadores) batem nas CELULAS (os triangulos de
// nos): a malha da fisica pode ser mais grossa que a de verdade, com a celula
// maior que a bola ou o braco; o centro que passaria por uma celula desde o
// comeco do tick volta para o lado de onde veio. A caixa bate nos pontos da malha de
// verdade sobre ela (os nos virtuais: a correcao vai para os nos pelos
// pesos). O contato e em tres tempos:
// o corpo empurra os nos (pela massa: o fio leve cede); os nos que ele tocou
// passam a andar com ele (a massa somada) enquanto os fios e as amarras puxam,
// e assim o corpo sente a rede com a rigidez de verdade dos fios (frouxa, so
// arrasta os nos; esticada, freia pela elasticidade); por fim os nos que
// sobraram dentro dele saem. Atrito de Coulomb na esfera (a bola perde o giro
// na rede). O ar freia cada no pelo fio que ele carrega, e um atrito interno
// pequeno segura o movimento lento. A rede parada dorme e, dormindo, segura o
// que esta apoiado nela como um corpo parado; acorda quando algo chega rapido.
//
// Puro (sem o motor de fisica): o backend acopla nos subpassos da bola, e os
// testes rodam sozinhos.
namespace MatterEngine {

struct NetSegment3D {
    std::uint32_t a = 0;
    std::uint32_t b = 0;
    float restLength = 0.0f;
    // Rigidez axial (N: EA). A mola deste pedaco e EA / comprimento. (Fio
    // trancado de PP de 4 mm: rompe com ~2,8 kN a ~20% de alongamento.)
    float axialStiffness = 10000.0f;
    // Um cabo (o tensor do alto; ver ropeIterations).
    bool rope = false;
};

// Um ponto da rede entre os nos: a media de ate quatro nos (o no da malha de
// verdade sobre a malha grossa da fisica). As capsulas e as caixas tocam a
// rede por eles.
struct NetSample3D {
    std::array<std::uint32_t, 4> knots {};
    std::array<float, 4> weights {};
};

// Uma peca parada que a rede nao atravessa (um tubo da trave).
struct NetCapsule3D {
    Vec3 a;
    Vec3 b;
    float radius = 0.0f;
};

struct NetSettings3D {
    Vec3 gravity { 0.0f, 0.0f, -9.81f };
    // O raio do fio (colisao); o diametro (2x) entra no arrasto.
    float twineRadius = 0.002f;
    float airDensity = 1.2f;
    // Cilindro em fluxo cruzado (Re ~ 10^3).
    float dragCoefficient = 1.2f;
    // O chao (plano z = groundHeight) e o atrito do fio nele.
    bool ground = true;
    float groundHeight = 0.0f;
    float groundFriction = 0.6f;
    // O amortecimento dos fios esticados, viscoso: a forca e EA x este tempo
    // (s) x a taxa de alongamento (a mesma em qualquer malha e em qualquer
    // passo). Uma fracao fixa do movimento relativo por fio amortecia 4x mais
    // a malha de 24 cm que a de 12 (o efeito cresce com o quadrado do fio), e
    // o teto andava ~40% menos. (8,5e-7 s: no fio de 12 cm entre nos de ~1,8 g,
    // os 8% de antes a cada 1/960 s.)
    float stretchDampingTime = 8.5e-7f;
    // O atrito interno (o fio trancado, as presilhas) e o ar devagar (1/s):
    // o arrasto do ar cai com o quadrado da velocidade e sozinho deixava a
    // rede balancando devagar por mais de 8 s. (5 desde 26/10: com a bola nas
    // massas de verdade o chute mexe mais a rede, e com 1,5 o teto ficava
    // balancando +-5 mm sem dormir; com 5 ela assenta como antes, ~3,7 s
    // depois do chute, e o rebote da bola muda pouco: 2,6 -> 2,2 m/s.)
    float linearDamping = 5.0f;
    // O atrito interno que segura o fio parado (m/s^2): tira esta velocidade
    // por segundo de cada no, so pesa no movimento lento. Sem ele a projecao
    // deixava a rede parada escorregando devagar (cm/s) para sempre.
    float restFriction = 0.3f;
    // A amarra comeca a puxar com o caminho dos fios mais este alongamento
    // (os fios esticam um pouco com o peso: na conta exata ela brigava com
    // eles e a rede parada tremia).
    float tetherStretch = 0.01f;
    // No contato com a caixa e a capsula o no conta como esta vezes mais pesado
    // (a escala de massa de colisao dos panos com corpos rigidos). A bola usa
    // as massas de verdade (26/10): a escala tirava 3/4 da reacao dos nos e
    // era a freada artificial dela; com o contato por celulas, o limite de
    // alongamento e o teste do caminho do centro, ela nao atravessa (0 de 768
    // na grade forte). Historico (quando valia para a bola): com a massa de
    // verdade (2 g contra a bola de 430 g) a bola levava so 0,4% de cada
    // correcao, abria a malha e passava. Maior = a rede segura antes, com
    // menos estufada. (Na trave, o chute de 25 m/s no meio do fundo, na malha
    // de 12 cm: com 10 a rede estufava 61 cm e com 6, 69 cm, a bola parando
    // dentro do gol; com 4, 77 cm, mas a bola voltava e saia do gol.) O 6 e o
    // da malha de 12 cm; a trave escolhe pela malha da fisica (6, 4 e 3: os
    // nos grossos ja sao mais pesados). Na de 2 malhas, numa grade de 27
    // chutes: com 4, a rede estufa 0,71 m na media (como a de 12 cm) e 4
    // bolas voltam para fora (6 na de 12 cm); com 6, 0,67 m e nenhuma volta,
    // mas o alto do fundo e o teto andam ~40% menos nos chutes altos.
    float contactMassScale = 6.0f;
    // As rodadas do acoplamento fio-bola por subpasso (so com a bola
    // encostando): os fios e o contato da bola de novo, ate a tensao chegar a
    // moldura com as massas de verdade.
    std::uint32_t ballContactRounds = 2;
    // A perda no contato da caixa e da capsula (a bola nao: a freada dela e a
    // da rede, com as massas de verdade e o atrito): uma desaceleracao (m/s^2)
    // do corpo ao longo da normal media dos contatos, constante.
    float contactBrake = 6.0f;
    // O alongamento maximo de um fio: dali em diante ele nao cede mais (perto
    // do rompimento). Garante que a malha nao abre a ponto de a bola passar
    // (precisaria esticar ~40%).
    float maximumStrain = 0.15f;
    // Os cabos (os pedacos rope: o tensor do alto) levam ropeIterations
    // passadas a mais por subpasso, e cada no deles e puxado para a reta entre
    // as pontas presas pela tracao. Uma corrente rigida presa so nas pontas nao
    // fica esticada com uma passada so: a tracao nao chega ao meio (o cabo de
    // 7,4 m afundava 62 cm). Com a reta, o cabo parado afunda o mesmo com 0 a
    // 4 passadas (11 mm na malha de 2), e a grade de chutes da o mesmo com 0,
    // 1 ou 2 (sem nenhuma, a rede demorava um pouco mais a dormir).
    std::uint32_t ropeIterations = 1;
    // Dormir: nenhum no sai mais de sleepDistance (m) de onde estava por
    // sleepSeconds, com os corpos que tocam a rede abaixo de sleepSpeed (m/s).
    // (Pelo deslocamento e nao pela velocidade de cada subpasso: a projecao
    // deixa um tremor de decimos de milimetro com velocidade de cm/s.) Acorda
    // com um corpo chegando mais rapido que wakeSpeed.
    float sleepDistance = 0.003f;
    // (Com 0,05 a bola parando devagar encostada na rede a mantinha acordada
    // 2,4 s a mais depois de o lance acabar: dormindo, a rede ja segura o que
    // esta apoiado nela, e acorda com algo chegando mais rapido que wakeSpeed.)
    float sleepSpeed = 0.2f;
    // (3 mm em 0,3 s: dorme com o balanco que sobra abaixo de ~7 mm; com 0,6
    // s a rede ficava ~0,7 s a mais acordada so com o tremor invisivel.)
    float sleepSeconds = 0.3f;
    float wakeSpeed = 0.25f;
};

struct NetDescription3D {
    std::vector<Vec3> positions;
    std::vector<float> masses;
    std::vector<std::uint8_t> pinned;
    std::vector<NetSegment3D> segments;
    // As celulas em triangulos (a esfera toca a rede por elas).
    std::vector<std::array<std::uint32_t, 3>> faces;
    // Vazio, os proprios nos.
    std::vector<NetSample3D> samples;
    // A area de cada no contra o ar (m^2: o diametro x o fio que ele carrega);
    // vazia, a metade de cada fio dele.
    std::vector<float> dragAreas;
    std::vector<NetCapsule3D> colliders;
    NetSettings3D settings;
};

enum class NetProxyShape3D : std::uint8_t { Sphere, Capsule, Box };

// Um corpo que a rede pode tocar neste tick.
struct NetBody3D {
    // O centro de massa e a orientacao agora.
    Vec3 position;
    Quaternion orientation;
    Vec3 velocity;
    Vec3 angularVelocity;
    // 0: nao cede (parado, cinematico).
    float inverseMass = 0.0f;
    // O giro pelo atrito (so os contatos da esfera usam).
    float inverseInertia = 0.0f;
    // Anda e gira pelas velocidades a cada subpasso (os corpos do motor, cuja
    // pose so muda no passo seguinte); a bola e integrada por fora.
    bool advance = false;
    // Guiado pela animacao (o boneco no modo padrao): a rede sai inteira do
    // caminho dele e so relata o impulso que ele sentiria solto, com a massa
    // dele (pequeno contra a rede frouxa; contra a esticada, o de parar o
    // membro). Quem chama decide o que fazer com isso.
    bool guided = false;
    // Saida (acumula ate quem chama zerar): o impulso que a rede deu nele
    // (N s), o quanto ela o empurrou (m: quem integra o corpo por fora move
    // ele junto, ou ele fica dentro da rede ate o passo seguinte), a soma dos
    // pontos pesada e o peso, e quantos contatos teve.
    Vec3 impulse;
    Vec3 shift;
    Vec3 impulsePointSum;
    float impulseWeight = 0.0f;
    std::uint32_t contacts = 0;
};

struct NetProxy3D {
    NetProxyShape3D shape = NetProxyShape3D::Sphere;
    std::uint32_t body = 0;
    // No referencial do corpo.
    Vec3 localPosition;
    Quaternion localOrientation;
    // Caixa: as meias-medidas. Esfera e capsula: radius; capsula: o meio
    // comprimento do cilindro ao longo do z local.
    Vec3 halfExtents;
    float radius = 0.0f;
    float halfHeight = 0.0f;
    float friction = 0.5f;
};

struct NetTickStats3D {
    std::uint32_t substeps = 0;
    std::uint32_t contacts = 0;
    float maximumSpeed = 0.0f;
    // O fio mais esticado (alongamento sobre o comprimento de repouso, ou
    // meio fio comum nos pedacos curtos da borda).
    float maximumStrain = 0.0f;
};

class NetSolver3D {
public:
    explicit NetSolver3D(const NetDescription3D& description);

    // No comeco do tick: acorda se um corpo chega rapido e monta as listas do
    // tick (os nos perto dos tubos e dos corpos, os fios perto das esferas).
    void beginTick(float deltaTime, std::span<const NetBody3D> bodies, std::span<const NetProxy3D> proxies);
    // Um subpasso de h segundos. Os corpos com massa recebem as correcoes
    // (posicao, velocidade, giro) e acumulam o impulso.
    void substep(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies);
    // No fim do tick: as medidas e o sono.
    void endTick(float deltaTime, std::span<const NetBody3D> bodies);

    // Sem corpo nenhum ate dormir (a rede pendurada na posicao de repouso) ou
    // ate maximumSeconds; devolve o tempo usado.
    float settle(float maximumSeconds, float tick = 1.0f / 120.0f, std::uint32_t substeps = 8);

    [[nodiscard]] bool sleeping() const { return m_sleeping; }
    void wake();
    // Se a rede tem o que fazer nos subpassos deste tick (dormindo e sem nada
    // apoiado, nao).
    [[nodiscard]] bool needsSubsteps() const { return !m_sleeping || m_touching; }
    [[nodiscard]] std::span<const Vec3> positions() const { return m_positions; }
    [[nodiscard]] std::span<const Vec3> velocities() const { return m_velocities; }
    [[nodiscard]] std::span<const NetSegment3D> segments() const { return m_segments; }
    [[nodiscard]] std::span<const std::uint8_t> pinned() const { return m_pinned; }
    [[nodiscard]] const NetSettings3D& settings() const { return m_settings; }
    [[nodiscard]] Vec3 boundsMinimum() const { return m_boundsMinimum; }
    [[nodiscard]] Vec3 boundsMaximum() const { return m_boundsMaximum; }
    [[nodiscard]] const NetTickStats3D& stats() const { return m_stats; }
    [[nodiscard]] float totalMass() const { return m_totalMass; }
    // A amarra de cada no (o no preso e o comprimento; o proprio no se preso).
    [[nodiscard]] std::uint32_t tetherAnchor(std::uint32_t knot) const { return m_tetherAnchor[knot]; }
    [[nodiscard]] float tetherLength(std::uint32_t knot) const { return m_tetherLength[knot]; }

private:
    struct SphereContact {
        std::uint32_t proxy = 0;
        std::uint32_t face = 0;
        std::array<float, 3> weights {};
        Vec3 normal;
        // A correcao normal (impulso x h) que o atrito pode usar.
        float lambda = 0.0f;
    };
    // Share: corpo e nos dividem a correcao (o no com a escala de massa).
    // StaticNet: a rede dormindo, so o corpo sai.
    // Final: o mesmo, mas contra caixa e capsula o no com um fio esticado
    // (acima de StiffStrain) nao cede (a rede esticada esta presa na trave: o
    // corpo sai inteiro).
    enum class ContactPass : std::uint8_t { Share, Final, StaticNet };

    void computeTethers();
    void computeBounds();
    void collideFrame();
    void collideGround();
    void collideBodies(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies, ContactPass pass, bool ballsOnly = false);
    void applyFriction(float h, std::span<NetBody3D> bodies, std::span<const NetProxy3D> proxies, bool staticNet);

    NetSettings3D m_settings;
    std::vector<Vec3> m_positions;
    // A referencia da velocidade (o comeco do subpasso, menos a recuperacao das
    // penetracoes, que nao vira velocidade) e a posicao geometrica do comeco do
    // subpasso (o caminho que os tubos e o chao conferem).
    std::vector<Vec3> m_previous;
    std::vector<Vec3> m_start;
    std::vector<Vec3> m_velocities;
    std::vector<float> m_inverseMasses;
    std::vector<float> m_masses;
    // O arrasto do ar no no: 1/2 rho Cd A / m (1/m; A: diametro x fio
    // carregado).
    std::vector<float> m_dragFactors;
    std::vector<std::uint8_t> m_pinned;
    // Os nos soltos (os presos nao andam: ficam fora dos lacos por no).
    std::vector<std::uint32_t> m_free;
    std::vector<NetSegment3D> m_segments;
    std::vector<std::array<std::uint32_t, 3>> m_faces;
    std::vector<NetSample3D> m_samples;
    std::vector<float> m_compliances;
    // O laco quente dos fios: as pontas, o comprimento solto, a complacencia
    // e o peso de cada ponta (todos e os cabos).
    struct Link {
        std::uint32_t a = 0;
        std::uint32_t b = 0;
        float rest = 0.0f;
        float compliance = 0.0f;
        float wa = 0.0f;
        float wb = 0.0f;
        float inverseWeights = 0.0f;
        float inverseSoft = 0.0f;
        // A taxa do amortecimento (1/s: tempo x EA/L x (wa + wb)) e a fracao
        // dela neste passo.
        float dampingRate = 0.0f;
        float damping = 0.0f;
        // O pedaco (m_segments), comum as ordens de visita: o multiplicador dele em m_linkLambda.
        std::uint32_t slot = 0;
    };
    [[nodiscard]] static std::vector<Link> scheduleLinks(const std::vector<Link>& order, std::size_t knotCount);
    std::vector<Link> m_links;
    std::vector<Link> m_linksBackward;
    std::vector<Link> m_ropeLinks;
    std::vector<Link> m_ropeLinksBackward;
    // O multiplicador XPBD elastico de cada fio acumulado no subpasso (zerado no comeco dele): as
    // passadas a mais (as do cabo, as rodadas da bola) continuam a mesma solucao em vez de encolher o fio
    // de novo (sem ele, cada passada repetida endurecia o fio).
    std::vector<float> m_linkLambda;
    float m_linkStep = 0.0f;
    // Um no de cabo entre as duas pontas presas: a tracao do cabo o puxa para
    // a reta entre as pontas com a rigidez lateral de uma corda esticada,
    // T (1/a + 1/b) (a e b: as distancias as pontas; T: EA x o alongamento
    // que o cabo tem esticado entre elas). A complacencia e o inverso.
    struct RopeSpan {
        std::uint32_t knot = 0;
        std::uint32_t first = 0;
        std::uint32_t second = 0;
        float compliance = 0.0f;
    };
    std::vector<RopeSpan> m_ropeSpans;
    // Os nos com algum fio esticado demais neste subpasso (e a lista deles).
    std::vector<std::uint8_t> m_stiff;
    std::vector<std::uint32_t> m_stiffKnots;
    // O no encostado num tubo neste subpasso e a direcao para fora dele (a bola
    // nao o empurra contra o tubo: a moldura esta atras dele).
    std::vector<Vec3> m_onFrame;
    std::vector<std::uint32_t> m_onFrameKnots;
    std::vector<std::uint32_t> m_tetherAnchor;
    std::vector<float> m_tetherLength;
    std::vector<float> m_tetherCompliance;
    std::vector<NetCapsule3D> m_colliders;
    float m_totalMass = 0.0f;
    float m_strainLength = 0.0f;

    // Os pares (no, tubo) que podem se tocar e os que estao perto neste tick.
    std::vector<std::pair<std::uint32_t, std::uint32_t>> m_framePairs;
    std::vector<std::pair<std::uint32_t, std::uint32_t>> m_frameNear;
    // Por proxy: o intervalo em m_candidates (celulas da esfera, nos
    // virtuais dos outros).
    std::vector<std::pair<std::uint32_t, std::uint32_t>> m_proxyRanges;
    std::vector<std::uint32_t> m_candidates;
    // O centro de cada proxy no comeco do tick.
    std::vector<Vec3> m_proxyStarts;
    // Os nos no comeco do tick: o lado de onde o corpo veio e medido na celula
    // de entao (a rede voltando mais rapido que a bola passava por cima dela).
    std::vector<Vec3> m_tickStart;
    // A caixa de cada celula no comeco do tick, as celulas e os nos virtuais
    // perto dos proxies (a lista curta do tick) e o tamanho maximo de uma
    // celula (m).
    std::vector<Vec3> m_faceLow;
    std::vector<Vec3> m_faceHigh;
    bool m_faceBoundsStale = true;
    std::vector<std::uint8_t> m_near;
    std::vector<std::uint32_t> m_localFaces;
    std::vector<std::uint32_t> m_localSamples;
    float m_cellReach = 0.0f;
    std::vector<SphereContact> m_sphereContacts;
    // O impulso normal que cada corpo recebeu neste subpasso (o atrito do
    // giro da esfera).
    std::vector<float> m_contactImpulse;
    // A soma das normais dos contatos de cada corpo (pesadas pelo impulso).
    std::vector<Vec3> m_contactNormal;

    Vec3 m_boundsMinimum;
    Vec3 m_boundsMaximum;
    bool m_sleeping = false;
    // Dormindo, algum corpo apoiado (a rede o segura parada).
    bool m_touching = false;
    // A maior velocidade de um corpo em contato neste tick.
    float m_touchSpeed = 0.0f;
    float m_stillSeconds = 0.0f;
    // O tick de agora (s).
    float m_tickTime = 1.0f / 120.0f;
    // O no mais rapido no fim do tick, o quanto ele pode ter andado desde a
    // lista dos tubos e se ela precisa ser refeita (a rede acordou).
    float m_fastestKnot = 0.0f;
    float m_frameTravel = 0.0f;
    bool m_frameStale = true;
    // Onde os nos estavam quando a rede parou de andar (o teste do sono).
    std::vector<Vec3> m_stillReference;
    NetTickStats3D m_stats;
};

} // namespace MatterEngine
