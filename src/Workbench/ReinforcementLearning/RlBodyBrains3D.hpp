#pragma once

#include "Engine/Athlete/AthleteSettings3D.hpp"
#include "Engine/Athlete/AthleteState3D.hpp"
#include "Engine/Character/CharacterLocomotion3D.hpp"
#include "Engine/Character/DefaultLocomotion3D.hpp"
#include "Engine/Physique/PlayerPhysique3D.hpp"
#include "Engine/Physics/PhysicsScene3D.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include "Workbench/ReinforcementLearning/TrackingViewer3D.hpp"

#include <cstdint>
#include <filesystem>
#include <functional>
#include <future>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

namespace MatterEngine {

// (16/11) O cerebro unico do RL nos corpos do jogo, com a fisica do jogo: um cerebro por boneco (a mesma rede, uma instancia por corpo).
// O controlador do treino le o ragdoll NA CENA DO JOGO e poe os alvos dos motores dele - o chao, a bola, os outros, as paredes e as
// pancadas sao os do jogo. Usado pelo laboratorio in-game (WorkbenchApp) e pela sonda do atleta (o mesmo codigo nos dois).

// Um corpo que o cerebro pode dirigir, preenchido a cada tick pelo dono do corpo.
struct RlBrainBody3D {
    std::uint64_t entityId = 0;
    // O ragdoll (o cerebro o recria com o perfil do treino quando o corpo e outro) e o estado dele do ultimo passo.
    RagdollHandle3D* handle = nullptr;
    RagdollState3D* state = nullptr;
    // Os estados de antes (para o desenho), atualizados quando o corpo e recriado; opcionais.
    RagdollState3D* previousState = nullptr;
    RagdollState3D* simulationPreviousState = nullptr;
    AthleteState3D* athlete = nullptr;
    bool usable = false;
    // O jogador controlado (o registro detalhado e so dele).
    bool controlled = false;
    // Este corpo roda a locomocao padrao (o jogador no Padrao; os outros com "bonecos no padrao").
    bool inDefault = false;
    // A entrada: o rumo pedido no mundo (zero: parado), a vista (rad) e o shift. Sem entrada (commanded falso), o boneco fica parado
    // olhando para onde estava quando o cerebro assumiu.
    Vec3 direction;
    float viewYaw = 0.0f;
    bool sprint = false;
    bool commanded = false;
    // A velocidade pedida no mundo (a da tabela do padrao) e o olhar para cima/baixo: a sombra do padrao na volta.
    Vec3 requestedVelocity;
    float lookPitch = 0.0f;
};

struct RlBrainSettings3D {
    std::filesystem::path assetsDirectory;
    std::filesystem::path runDirectory;
    // O Fisico com RL (o cerebro sempre) e o Padrao com o fisico pelo RL (enquanto o padrao daquele corpo esta no fisico; e sempre nos
    // corpos fora do padrao).
    bool full = false;
    bool transitions = false;
    // O corpo do jogo nao e o do treino (outro corpo, a rigidez do painel): o cerebro o recria com o perfil do treino.
    bool recreateBody = false;
    // As reguas da ajuda auxiliar (0..1): parado, andando/correndo, levantando.
    float assistStanding = 0.0f, assistWalking = 0.0f, assistGetUp = 0.0f;
    // A tensao dos limites do jogo (devolvida ao corpo quando o cerebro o larga) e o perfil do padrao (o recomeco dele na volta).
    bool gameLimitTension = false;
    const RagdollProfile3D* gameProfile = nullptr;
    // (A rigidez do painel: o corpo do jogo recriado de volta, na mesma pose, quando o cerebro o larga.)
    float gameRigidityPercent = 0.0f;
    // Carregar os cerebros em segundo plano (o jogo); a sonda carrega na hora.
    bool loadAsync = true;
    // (16/11, BodyNavigation3D) A capsula do corpo: enquanto o cerebro dirige, a navegacao de cada corpo - a capsula do jogador, a
    // virtual do boneco - fica nele a cada tick (o padrao volta de onde o corpo esta).
    const CharacterMotorSettings3D* avatar = nullptr;
    // (16/11, a volta harmonica ao padrao) Os clipes do padrao, os ajustes do runtime e o corpo: a sombra do padrao que o cerebro imita
    // antes de devolver. Sem eles, a volta e a de antes (direto ao guiado pela mola).
    const CharacterLocomotionAnimations3D* defaultClips = nullptr;
    const AthleteRuntimeSettings3D* runtimeSettings = nullptr;
    PlayerBody3D body;
    // A mistura: o tempo da subida de 0 a 100% dos musculos na pose do padrao (s), o limite para encaixar (s), o encaixe so com as juntas
    // a menos de commitJointError (rad, rms) e os pes e a pelve a menos de commitDistance (m) da sombra, e a ajuda auxiliar que sobe junto.
    float blendSeconds = 0.5f;
    float blendMaxSeconds = 1.5f;
    float commitJointError = 0.10f;
    float commitDistance = 0.05f;
    float blendAssist = 0.4f;
    // A mola da volta (meia-vida e duracao, s): depois da mistura - a diferenca ja pequena - curta, para a perna nao seguir com atraso o
    // passo do padrao (com 0,30 s, o giro dele depois do encaixe ficava com o joelho pendurado 0,3 s); na volta direta (andando), media.
    float blendCommitHalfLife = 0.12f, blendCommitSeconds = 0.6f;
    float directHalfLife = 0.22f, directSeconds = 1.0f;
    // (16/11 19h55, ele: "entre na locomocao default assim que ja levanta, sem demorar tanto como e agora, CASO nao haja contatos") Depois
    // de um levantar do cerebro, sem tecla: a volta direta assim que ele fica de pe - pelve, peito e cabeca no alto, nada encostando alem
    // do chao - por getUpReturnCalmSeconds, com o centro de massa ate getUpReturnSpeed (o fim do levantar ainda balanca), sem esperar o
    // giro dele para a vista (o rumo vai a camera depois, pela mola do padrao) nem a mistura de 0,5 s. Medido antes (sonda queda_levanta):
    // ~2,1 s de pe ate devolver - 0,8 s esperando parar abaixo de 0,3 m/s, 0,4 s de calma e 0,6 s de mistura. false: como antes.
    bool quickGetUpReturn = true;
    float getUpReturnSpeed = 0.8f, getUpReturnCalmSeconds = 0.1f;
    // (A volta rapida parado: pela mistura harmonica - true - ou direta.)
    bool getUpReturnBlend = false;
};

class RlBodyBrains3D {
public:
    // O que o dono do corpo faz depois do update: os corpos que o cerebro largou sem a volta ao padrao (o atleta deles recomecou do
    // corpo como esta, no fisico - nada recriado nem posto de pe: o dono so zera o que e dele, como a capsula seguindo o corpo).
    struct Events {
        std::vector<std::uint64_t> releasedFromFull;
    };
    ~RlBodyBrains3D();
    // Antes da simulacao: a entrada, a decisao da rede a cada 4 ticks (30 Hz, como no treino), os alvos dos motores (e a mao do treino,
    // se o painel pede), a volta ao padrao.
    void update(PhysicsScene3D& scene, std::span<RlBrainBody3D> bodies, const RlBrainSettings3D& settings, float deltaTime,
        Events& events);
    // Depois da simulacao (os estados dos corpos ja os deste passo): o resto da decisao no 4o tick e a navegacao de cada corpo nele.
    void finish(PhysicsScene3D& scene, std::span<RlBrainBody3D> bodies, const RlBrainSettings3D& settings);
    [[nodiscard]] bool drives(std::uint64_t entityId) const;
    // Tira todos (os cerebros recarregam no proximo update).
    void clear();
    [[nodiscard]] const std::string& label() const { return m_label; }
    // O registro (opcional): quem assume, o estado, a volta, as trocas do modo do padrao.
    std::function<void(const std::string&)> log;
    // (A volta ao padrao, para quem mede: o tick em que o cerebro devolveu cada corpo, desde o ultimo pedido.)
    [[nodiscard]] std::vector<std::uint64_t> takeReturned();

private:
    struct Slot {
        std::unique_ptr<TrackingViewer3D> brain;
        std::future<std::unique_ptr<TrackingViewer3D>> loading;
        bool failed = false;
        RagdollHandle3D body;
        bool active = false, full = false;
        int tick = 0;
        float standingSeconds = 0.0f, seconds = 0.0f, logSeconds = 0.0f, idleView = 0.0f;
        LegacyBodyMode3D legacyMode = LegacyBodyMode3D::Animated;
        // A volta harmonica: a sombra do padrao (rodando do corpo, sem mexer nele), se esta misturando, ha quanto, e as diferencas de
        // agora (juntas rms, rad; pes e pelve, m).
        DefaultLocomotion3D shadow;
        bool blending = false;
        float blendTime = 0.0f, jointError = 1.0e9f, distance = 1.0e9f, feetGap = 1.0e9f;
        Vec3 pelvisGap;
        // (Parado: o rumo da sombra - o do corpo no comeco da mistura - e a raiz dela, fixa: o chao sob a pelve no comeco. Seguindo a
        // pelve a cada tick, a pose alvo andava junto com o corpo que ia ate ela e a mistura nunca encaixava - sempre no limite, 1,5 s.)
        float shadowFacing = 0.0f;
        Vec3 shadowRoot;
        // (16/11) O peso de agora da mistura, a diferenca de rumo da pelve (rad, sombra - corpo) e a velocidade do centro de massa; e a
        // saida em rampa: desfeita de uma vez, os alvos saltavam da pose do padrao para a do cerebro num tick e ele caia.
        float blendWeight = 0.0f, yawGap = 0.0f, comSpeed = 0.0f;
        // (O que falta para assentar, para o registro.)
        std::string calmMissing;
        // (16/11, ele: "debrucado em alguma estrutura... outro em cima ou em contato") De pe encostado numa parede, caixa ou noutro corpo,
        // sem tecla: o passo para longe dele - o rumo (mundo), ha quanto encostado e o pulso da tecla (s).
        Vec3 awayFrom;
        float leaningSeconds = 0.0f, stepAwayLeft = 0.0f, stepAwayRest = 0.0f;
        bool turning = false;
        // (19h55, quickGetUpReturn) O cerebro levantou o corpo nesta posse (passou por fora do de pe): a volta rapida vale.
        bool fromGetUp = false;
        // (De pe e sem tecla, no fisico do padrao: o cerebro segura o rumo da pelve - o giro ate a camera e do padrao depois da volta. O
        // giro parado do cerebro e lento, ~0,6 rad/s, e arrasta os pes: perto de uma caixa ele girava sem parar e nunca assentava.)
        bool uprightStill = false, holdingView = false;
        float heldView = 0.0f;
        bool fading = false;
        float fadeFrom = 0.0f, fadeTime = 0.0f, fadeSeconds = 0.3f;
        std::vector<Vec3> fadeCoordinates;
    };
    void beginBlend(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings, float view);
    // Um tick da sombra (o padrao rodando sob o corpo, com a entrada); devolve se ha pose.
    bool stepShadow(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings, float view, float deltaTime);
    bool adopt(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings);
    // A navegacao do corpo nele (a capsula do jogador, a virtual do boneco).
    void anchorNavigation(PhysicsScene3D& scene, const Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings) const;
    // O atleta do corpo do zero, no fisico, do corpo como esta (o rumo da guia: a vista de quem comanda, senao o da pelve).
    void restartAthlete(PhysicsScene3D& scene, const Slot& slot, RlBrainBody3D& body, const RlBrainSettings3D& settings) const;
    void release(PhysicsScene3D& scene, Slot& slot, RlBrainBody3D* body, const RlBrainSettings3D& settings, const char* why, Events& events);
    void write(const std::string& line) const {
        if (log) log(line);
    }
    std::unordered_map<std::uint64_t, Slot> m_slots;
    std::vector<std::uint64_t> m_returned;
    std::string m_label;
};

} // namespace MatterEngine
