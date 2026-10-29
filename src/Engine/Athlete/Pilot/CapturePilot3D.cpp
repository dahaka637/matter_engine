#include "Engine/Athlete/Pilot/CapturePilot3D.hpp"

#include "Engine/Athlete/AthleteTelemetry3D.hpp"
#include "Engine/Athlete/Physical/PhysicalCorrections3D.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine {
namespace {

constexpr float Pi = 3.14159265358979323846f;
// As oito teclas: W, W+A, A, S+A, S, S+D, D, W+D (a partir do olhar, no sentido anti-horario).
constexpr float KeyStep = Pi / 4.0f;

Vec3 planar(Vec3 v) { return { v.x, v.y, 0.0f }; }
Vec3 heading(float yaw) { return { std::cos(yaw), std::sin(yaw), 0.0f }; }
float yawOf(Vec3 v) { return std::atan2(v.y, v.x); }
float wrap(float angle) { return std::remainder(angle, 2.0f * Pi); }
Vec3 unit(Vec3 v, Vec3 fallback) {
    const float length = v.length();
    return length > 1e-4f ? v * (1.0f / length) : fallback;
}

} // namespace

void capturePilotSelf3D(CapturePilotView3D& view, const RagdollProfile3D& profile, const AthleteState3D& athlete,
    const RagdollState3D& body) {
    if (body.links.empty()) return;
    const PhysicsBodyState3D& pelvis = body.links.front();
    const AthleteTelemetry3D telemetry = athleteTelemetry3D(athlete);
    view.pelvisGround = { pelvis.position.x, pelvis.position.y, athlete.groundHeight };
    view.velocity = planar(pelvis.linearVelocity);
    view.bodyYaw = locomotionFrameYaw3D(profile, pelvis.orientation);
    view.animated = telemetry.mode != BodyModeKind3D::Physical;
    view.down = telemetry.fallen || telemetry.gettingUp || telemetry.tackling;
    view.terrainSpeedLimit = telemetry.terrainSpeedLimit;
}

float CapturePilot3D::random() {
    m_rng ^= m_rng << 13;
    m_rng ^= m_rng >> 17;
    m_rng ^= m_rng << 5;
    return static_cast<float>(m_rng % 10000u) / 10000.0f;
}

int CapturePilot3D::keysFor(Vec3 desired) const {
    if (desired.lengthSquared() < 1e-4f) return -1;
    const float relative = wrap(yawOf(desired) - m_look);
    // (Com folga: a combinacao de agora fica ate o rumo sair 30 graus dela.)
    if (m_keys >= 0 && std::abs(wrap(relative - static_cast<float>(m_keys) * KeyStep)) < 0.52f) return m_keys;
    const int nearest = static_cast<int>(std::lround(relative / KeyStep));
    return (nearest % 8 + 8) % 8;
}

AthleteCommand3D CapturePilot3D::update(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot,
    const AthleteRuntimeSettings3D& settings, const CharacterGaitSpeeds3D& gaitSpeeds, const GestureReach3D& reach,
    float deltaTime) {
    const float dt = std::max(0.0f, deltaTime);
    if (!m_started) {
        m_started = true;
        m_rng = static_cast<std::uint32_t>(((pilot.seed != 0 ? pilot.seed : view.self) + 1u) * 2654435761u) | 1u;
        for (int i = 0; i < 4; ++i) random();
        m_reaction = 0.18f + 0.14f * random();
        m_mouseRate = 4.5f + 3.0f * random();
        m_aggression = 0.35f + 0.55f * random();
        m_rhythm = 0.45f + 0.3f * random();
        m_sprintLove = 0.4f + 0.5f * random();
        m_side = random() < 0.5f ? -1.0f : 1.0f;
        m_look = view.bodyYaw;
        m_lastOwner = view.ballOwner;
    }
    m_decisionLeft -= dt;
    m_sinceTouch += dt;
    m_sinceTackle += dt;
    m_ctrlLeft -= dt;
    m_feintLeft -= dt;
    m_lungeLeft -= dt;
    m_recoverLeft -= dt;
    m_keysAge += dt;
    const Vec3 me = planar(view.pelvisGround);
    BallActionKind3D request = BallActionKind3D::None;
    float requestPower = 0.0f;
    Vec3 desired;
    // A bola no referencial do corpo e no alcance dos pes (o mesmo do jogador).
    const Vec3 ballOffset = planar(view.ballPosition) - me;
    const float ballDistance = ballOffset.length();
    bool reachable = false;
    if (view.ball) {
        const Vec3 forward = heading(view.bodyYaw);
        const Vec3 left { -forward.y, forward.x, 0.0f };
        reachable = inGestureReach3D(
            { dot(ballOffset, forward), dot(ballOffset, left), view.ballPosition.z - view.pelvisGround.z }, reach);
    }
    // (A bola dele fora do alcance: vai nela de frente, olhando para ela, ate
    // toca-la - parando no meio do caminho, ele ficava a 1,1 m dela sem tocar.)
    bool chasingOwn = false;
    if (view.down) {
        // Caido, levantando ou no carrinho: as maos soltas (como o laboratorio
        // faz com o jogador); de pe de novo, um respiro para se achar.
        m_wasDown = true;
        m_holding = false;
        m_ctrlLeft = 0.0f;
        m_lungeLeft = 0.0f;
    } else {
        if (m_wasDown) {
            m_wasDown = false;
            m_recoverLeft = 0.3f + 0.4f * random();
            m_decisionLeft = 0.0f;
        }
        // A posse mudou: ele percebe com atraso; quem perdeu a bola respira.
        if (view.ball && view.ballOwner != m_lastOwner) {
            if (m_lastOwner == view.self) m_recoverLeft = std::max(m_recoverLeft, 0.25f + 0.3f * random());
            m_decisionLeft = std::max(m_decisionLeft, 0.1f + 0.15f * random());
            m_lastOwner = view.ballOwner;
            m_lungeLeft = 0.0f;
        }
        if (m_decisionLeft <= 0.0f) {
            decide(view, pilot);
            m_decisionLeft = m_reaction * (0.8f + 0.45f * random());
        }
        // Para onde ir agora: o ponto da ultima decisao (com folga na chegada,
        // sem tremer em cima dele) ou o rumo.
        if (m_toPoint) {
            const Vec3 to = planar(m_target) - me;
            const float distance = to.length();
            if (distance < m_stopRadius) m_arrived = true;
            else if (distance > m_stopRadius + 0.45f) m_arrived = false;
            if (!m_arrived && distance > 1e-3f) desired = to * (1.0f / distance);
        } else if (m_plan == Plan::Keep && view.ball) {
            // Com a bola: no rumo dele com ela no pe; solta a frente (ou para
            // tras, depois de um corte), vai nela - de frente, olhando para ela
            // (de lado, na corrida lateral, ele nao a alcancava).
            chasingOwn = !reachable;
            desired = chasingOwn ? unit(planar(view.ballPosition + view.ballVelocity * 0.2f) - me, m_heading)
                                 : m_heading;
        } else {
            desired = m_heading;
        }
    }
    // O olhar gira como a mao no mouse: ate m_mouseRate (mais no comeco de um
    // giro grande), com o embalo da mao; segue a bola, o ponto ou o rumo.
    {
        float goal = m_look;
        if (!view.down) {
            const Vec3 toBall = planar(view.ballPosition) - me;
            const Vec3 toTarget = planar(m_target) - me;
            if (chasingOwn) goal = yawOf(toBall);
            else if (m_lookAt == Look::Heading && m_heading.lengthSquared() > 0.0f) goal = yawOf(m_heading);
            else if (m_lookAt == Look::Ball && view.ball && toBall.length() > 0.3f) goal = yawOf(toBall);
            else if (m_lookAt == Look::Target && toTarget.length() > 0.5f) goal = yawOf(toTarget);
        }
        const float error = wrap(goal - m_look);
        const float limit = m_mouseRate * (1.0f + 0.6f * std::min(1.0f, std::abs(error) / Pi));
        const float wanted = std::clamp(error / 0.14f, -limit, limit);
        m_lookRate += (wanted - m_lookRate) * (1.0f - std::exp(-dt / 0.05f));
        m_look = wrap(m_look + m_lookRate * dt);
    }
    // O botao do drible: comeca a segurar com a bola chegando (a barra enche com
    // o tempo: a forca e o tempo segurado) e solta com ela no alcance; sem ela
    // no alcance a tempo, solta assim mesmo (o pedido se perde, como o do
    // jogador). No soltar, as teclas apontam o toque.
    bool touchKeys = false;
    if (!view.down && view.ball) {
        const float distance = ballDistance;
        // (Segura so com a bola no alcance: segurar esperando por ela enchia a
        // barra - um toque de 0,9 mandou a bola a 10 m/s.)
        if (!m_holding) {
            bool start = false;
            if (m_plan == Plan::Keep) {
                // (Com ela no pe ja indo no rumo dele, no ritmo dos toques; fora
                // do rumo, o toque de corrigir.)
                const Vec3 rolling = planar(view.ballVelocity);
                const bool along = rolling.length() > 0.5f && dot(unit(rolling, m_heading), m_heading) > 0.7f;
                start = reachable && m_sinceTouch > 0.25f && (m_sinceTouch > m_nextTouch || !along);
            } else if (m_plan == Plan::Chase) {
                start = reachable && m_sinceTouch > 0.4f;
            } else if (m_plan == Plan::Press) {
                start = (reachable || (m_lungeLeft > 0.0f && distance < 1.0f)) && m_sinceTouch > 0.4f;
            }
            if (start) {
                m_holding = true;
                m_held = 0.0f;
                // (Conduzindo, leve - mais fugindo no Shift; no bote, um cutucao
                // para longe de quem a tinha.)
                const float base = m_plan == Plan::Keep ? (m_sprint ? 0.22f : 0.14f)
                    : m_plan == Plan::Press ? 0.36f : 0.13f;
                m_holdPower = std::clamp(base + (random() - 0.5f) * 0.1f, 0.05f, 1.0f);
            }
        }
        if (m_holding) {
            m_held += dt;
            const float charge = std::max(0.05f, pilot.touchChargeSeconds);
            const float hold = m_holdPower * charge;
            if (m_held >= hold && (reachable || m_held > hold + 0.1f)) {
                m_holding = false;
                request = BallActionKind3D::Dribble;
                requestPower = std::min(1.0f, m_held / charge);
                m_sinceTouch = 0.0f;
                m_nextTouch = m_rhythm * (0.8f + 0.45f * random());
                // Conduzindo ou pegando a bola solta, no rumo da faixa; no bote,
                // para longe de quem a tinha (somado ao embalo).
                const bool loose = view.ballOwner == 0 || view.ballOwner == view.self;
                if (m_plan == Plan::Keep || (m_plan == Plan::Chase && loose && m_heading.lengthSquared() > 0.0f)) {
                    desired = m_heading;
                    // (No maximo 90 graus do rumo em que ele corre: o corte de
                    // costas freava a capsula e a bola fugia a 4,7 m/s; o resto
                    // da virada vem no proximo toque.)
                    if (view.velocity.length() > 1.5f) {
                        const float run = yawOf(view.velocity);
                        desired = heading(run + std::clamp(wrap(yawOf(desired) - run), -0.5f * Pi, 0.5f * Pi));
                    }
                } else {
                    const Vec3 rival = view.ownerSeen && view.ballOwner != view.self ? planar(view.ownerPelvis)
                        : view.player ? planar(view.playerPelvis) : me - heading(m_look);
                    desired = unit(unit(planar(view.ballPosition) - rival, heading(m_look))
                        + unit(view.velocity, {}) * 0.6f, heading(m_look));
                }
                touchKeys = true;
            }
        }
    }
    // As teclas: a combinacao mais perto do rumo, com folga, e a troca dos
    // dedos (~0,06 s) - menos no toque, que ele ja aperta junto.
    {
        const int keys = keysFor(desired);
        const bool fingers = !touchKeys && keys >= 0 && m_keys >= 0 && m_keysAge < 0.06f;
        if (keys != m_keys && !fingers) {
            m_keys = keys;
            m_keysAge = 0.0f;
        }
    }
    const Vec3 move = m_keys >= 0 ? heading(m_look + static_cast<float>(m_keys) * KeyStep) : Vec3 {};
    AthleteStickInput3D stick;
    stick.moveWorld = move;
    stick.cameraYawRadians = m_look;
    stick.lookPitchRadians = -0.2f;
    // (Alcancando a bola dele, o Shift so de longe: perto ele trota e o toque
    // sai na velocidade do trote - no Shift, a bola saia a 8 m/s.)
    // (Com folga: comecou a correr atras dela, segura o Shift ate alcanca-la -
    // ligando e desligando no limiar, ele ficava preso a 1,2 m dela.)
    if (!chasingOwn)
        m_catching = false;
    else if (ballDistance > 1.2f && (ballDistance > 2.0f || planar(view.ballVelocity).length() > 3.8f))
        m_catching = true;
    stick.sprint = (m_sprint || m_catching) && m_keys >= 0 && !view.down;
    stick.crouch = m_ctrlLeft > 0.0f;
    AthleteCommand3D command =
        athleteStickCommand3D(stick, settings, gaitSpeeds, view.animated ? 1.0e9f : view.terrainSpeedLimit);
    // (No padrao, a tabela dele, como o laboratorio faz com o jogador.)
    if (view.animated && m_keys >= 0)
        command.intent.requestedVelocityWorld = move
            * athleteDefaultTravelSpeed3D(settings, wrap(static_cast<float>(m_keys) * KeyStep), stick.sprint);
    command.pilot = true;
    command.ballRequest = request;
    command.ballRequestPower = requestPower;
    command.ballRequestAimYaw = m_look;
    command.ballDribbleHeld = m_holding;
    return command;
}

void CapturePilot3D::decide(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot) {
    const Vec3 me = planar(view.pelvisGround);
    m_toPoint = true;
    m_stopRadius = 0.0f;
    if (!view.ball) {
        // Sem bola: fica na faixa do jogador, como se o marcasse.
        if (view.player) {
            bandHeading(view, pilot);
            m_toPoint = false;
            m_lookAt = Look::Heading;
            m_sprint = false;
        } else {
            m_target = me;
            m_stopRadius = 1.0f;
            m_sprint = false;
            m_lookAt = Look::Target;
        }
        return;
    }
    const Vec3 ball = planar(view.ballPosition);
    const Vec3 toBall = ball - me;
    const float distance = toBall.length();
    const bool mine = view.ballOwner == view.self;
    const bool carried = view.ballOwner != 0 && !mine && view.ownerSeen
        && (ball - planar(view.ownerPelvis)).length() < 1.6f;
    if (m_recoverLeft > 0.0f) {
        // O respiro: volta a uns passos da bola olhando para ela, sem pressa.
        m_plan = Plan::Recover;
        const float keep = 2.4f + 0.8f * random();
        m_target = distance > keep ? me : ball - unit(toBall, heading(m_look)) * keep;
        m_stopRadius = 0.35f;
        m_sprint = false;
        m_lookAt = Look::Ball;
        return;
    }
    // (A bola dele continua dele ate uns metros: tocada a frente, ele vai nela
    // no rumo da faixa - perseguindo-a como bola solta, ele a tocava para longe
    // do jogador e fugia 30 m.)
    if (mine && distance < 4.0f) {
        m_plan = Plan::Keep;
        m_toPoint = false;
        m_lookAt = Look::Heading;
        m_sprint = bandHeading(view, pilot);
        return;
    }
    if (carried) {
        m_plan = Plan::Press;
        press(view, pilot);
        return;
    }
    // A bola solta (ou longe de quem a tinha): vai nela, mirando um pouco a
    // frente dela - nem sempre o ponto certo.
    m_plan = Plan::Chase;
    const float lead = std::clamp(distance / 7.0f, 0.0f, 0.7f) * (0.7f + 0.5f * random());
    m_target = ball + planar(view.ballVelocity) * lead;
    // (A bola fugindo dele, o Shift: trotando, ele ficava 3 s atras dela.)
    const float fleeing = distance > 1e-3f ? dot(planar(view.ballVelocity), toBall * (1.0f / distance)) : 0.0f;
    m_sprint = distance > 2.5f + 3.0f * (1.0f - m_sprintLove) || (distance > 1.5f && fleeing > 2.5f);
    m_lookAt = Look::Target;
    // (Chegando na bola solta, o toque de pega-la sai no rumo da faixa.)
    if (distance < 3.0f && (view.ballOwner == 0 || mine)) bandHeading(view, pilot);
}

bool CapturePilot3D::bandHeading(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot) {
    const Vec3 me = planar(view.pelvisGround);
    bool sprint = false;
    if (m_feintLeft <= 0.0f) {
        // A finta: de tempos em tempos troca o lado de rodear (o corte).
        m_feintLeft = 1.2f + 1.8f * random();
        if (random() < 0.6f) m_side = -m_side;
    }
    Vec3 wanted;
    if (view.player) {
        const float nearMeters = std::max(0.5f, pilot.nearMeters);
        const float farMeters = std::max(nearMeters + 0.5f, pilot.farMeters);
        const Vec3 radial = unit(me - planar(view.playerPelvis), heading(m_look));
        const float distance = (me - planar(view.playerPelvis)).length();
        const Vec3 tangent { -radial.y * m_side, radial.x * m_side, 0.0f };
        const float middle = 0.5f * (nearMeters + farMeters);
        // (O quanto o jogador vem nele, m/s.)
        const float closing = -dot(planar(view.playerVelocity), radial);
        if (distance < nearMeters || (distance < middle && closing > 3.0f)) {
            wanted = radial + tangent * (0.4f + 0.5f * random());
            sprint = true;
        } else if (distance > farMeters) {
            wanted = radial * -1.0f + tangent * 0.4f;
        } else {
            wanted = tangent + radial * (std::clamp((middle - distance) / (farMeters - nearMeters), -1.0f, 1.0f) * 2.0f);
            sprint = closing > 2.0f && random() < m_sprintLove;
        }
    } else {
        wanted = heading(m_look + (random() - 0.5f) * 0.9f);
    }
    m_heading = unit(wanted, heading(m_look));
    return sprint;
}

void CapturePilot3D::press(const CapturePilotView3D& view, const CapturePilotSettings3D& pilot) {
    const Vec3 me = planar(view.pelvisGround);
    const Vec3 ball = planar(view.ballPosition);
    const Vec3 toBall = ball - me;
    const float distance = toBall.length();
    const float speed = planar(view.velocity).length();
    m_lookAt = Look::Ball;
    // O carrinho: so no bote, embalado, com a bola a frente do olhar e do rumo
    // (o Ctrl curto, como o do jogador correndo).
    const auto tackle = [&]() {
        const float offLook = std::abs(wrap(yawOf(toBall) - m_look));
        const float offRun = speed > 0.5f ? std::abs(wrap(yawOf(view.velocity) - yawOf(toBall))) : Pi;
        if (pilot.tackles && m_sinceTackle > 6.0f && speed > 4.0f && distance > 1.3f && distance < 3.0f
            && offLook < 0.3f && offRun < 0.4f && random() < 0.4f * m_aggression) {
            m_ctrlLeft = 0.12f;
            m_sinceTackle = 0.0f;
        }
    };
    if (m_lungeLeft > 0.0f) {
        m_target = ball + planar(view.ballVelocity) * 0.2f;
        m_sprint = true;
        tackle();
        return;
    }
    if (distance > 4.0f) {
        // Longe: corre ate quem tem a bola, mirando onde ele vai estar.
        m_target = ball + planar(view.ownerVelocity) * std::clamp(distance / 8.0f, 0.2f, 0.6f);
        m_sprint = true;
        return;
    }
    // Perto: espreita a uns passos, no caminho dele, e da o bote quando a bola
    // se solta do pe (ou quando da vontade).
    const float loose = (ball - planar(view.ownerPelvis)).length();
    if (random() < m_aggression * (loose > 0.75f ? 0.7f : 0.22f)) {
        m_lungeLeft = 0.45f + 0.35f * random();
        m_target = ball + planar(view.ballVelocity) * 0.2f;
        m_sprint = true;
        tackle();
        return;
    }
    const float keep = 2.3f - 0.8f * m_aggression + 0.3f * random();
    m_target = ball - unit(toBall, heading(m_look)) * keep + planar(view.ownerVelocity) * 0.35f;
    m_stopRadius = 0.3f;
    m_sprint = distance > keep + 1.5f;
}

} // namespace MatterEngine
