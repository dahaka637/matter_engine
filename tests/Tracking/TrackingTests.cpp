// O rastreador fisico (vault 04_locomocao/referencia/pesquisas/12_especificacao_rastreador.md):
// o contrato (21 saidas, 106 entradas, as 39 juntas, a composicao), a nota do DReCon
// e o lote (o comeco sobreposto ao lider, o fim pela cabeca a 1 m).
#include "Engine/Athlete/AthleteAssets3D.hpp"
#include "Engine/RL/Body/RLBody3D.hpp"
#include "Engine/Tracking/TrackingContract3D.hpp"
#include "Engine/Tracking/TrackingLeader3D.hpp"
#include "Engine/Tracking/TrackingReward3D.hpp"
#include "Engine/Tracking/TrackingTraining3D.hpp"

#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

int checks = 0;

void expect(bool condition, const std::string& what) {
    ++checks;
    if (!condition) throw std::runtime_error(what);
}

const std::string Assets = MATTERENGINE_ASSETS_DIR;

float component(Vec3 v, std::size_t axis) { return axis == 0 ? v.x : axis == 1 ? v.y : v.z; }

TrackingLeader3D warmLeader(const AthleteBundle3D& bundle, float speed) {
    TrackingLeader3D leader(bundle.profile(), *bundle.assets().defaultClips, AthleteRuntimeSettings3D {});
    TrackingCommand3D command;
    command.requestedVelocityWorld = { speed, 0.0f, 0.0f };
    for (int k = 0; k < 240; ++k) leader.step(command, 1.0f / 120.0f);
    return leader;
}

// O corpo exatamente sobre o lider (o RagdollState3D de um corpo igual).
RagdollState3D bodyAt(const TrackingLeaderState3D& leader, const RagdollProfile3D& profile) {
    RagdollState3D body;
    body.links = leader.links;
    body.joints.resize(profile.links.size());
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        const Vec3 c = leader.coordinates[i];
        body.joints[i].positionRadians = { c.x, c.y, c.z };
    }
    return body;
}

void contractTests(const AthleteBundle3D& bundle) {
    const RagdollProfile3D& profile = bundle.profile();
    const TrackingContract3D contract(profile);
    expect(contract.actionNames().size() == 21, "a acao tem 21 canais (DReCon 25, sem dedo e com o pe de 2 eixos)");
    expect(contract.observationNames().size() == 106, "a observacao tem 106 entradas");
    std::size_t axes = 0;
    for (std::size_t i = 1; i < profile.links.size(); ++i)
        for (const RagdollAxisDefinition3D& axis : profile.links[i].inboundJoint.axes) axes += axis.enabled ? 1 : 0;
    expect(axes == 39, "o corpo do jogo tem 39 eixos");

    const TrackingLeader3D leader = warmLeader(bundle, 3.0f);
    const TrackingLeaderState3D& s = leader.state();
    expect(s.valid, "o lider da pose");
    const RagdollState3D body = bodyAt(s, profile);
    std::vector<float> action(21, 0.0f);
    std::vector<RagdollDriveTarget3D> drives;
    contract.driveTargets(s, body, action, drives);
    expect(drives.size() == 39, "as 39 juntas recebem alvo (sem alvo, o motor desliga)");
    for (const RagdollDriveTarget3D& d : drives)
        expect(std::abs(d.positionRadians - component(s.coordinates[d.linkIndex], static_cast<std::size_t>(d.axis)))
                < 1e-5f,
            "com a correcao zero, o alvo e o lider");

    // A composicao: coxa esquerda (3 eixos) pela esquerda; canela (1 eixo) somada.
    std::size_t thigh = 0, shin = 0;
    for (std::size_t i = 0; i < profile.links.size(); ++i) {
        if (profile.links[i].id == "LeftThigh") thigh = i;
        if (profile.links[i].id == "LeftShin") shin = i;
    }
    action[0] = 0.1f;   // LeftThigh.twist
    action[1] = -0.05f; // LeftThigh.swing1
    action[3] = 0.2f;   // LeftShin.twist
    contract.driveTargets(s, body, action, drives);
    const Vec3 composed = rlQuaternionLog3D(
        (rlQuaternionExp3D({ 0.1f, -0.05f, 0.0f }) * rlQuaternionExp3D(s.coordinates[thigh])).normalized());
    for (const RagdollDriveTarget3D& d : drives) {
        if (d.linkIndex == thigh)
            expect(std::abs(d.positionRadians - component(composed, static_cast<std::size_t>(d.axis))) < 1e-5f,
                "a coxa: R(alvo) = R(corr) R(lider)");
        if (d.linkIndex == shin)
            expect(std::abs(d.positionRadians - (s.coordinates[shin].x + 0.2f)) < 1e-5f, "a canela: somada");
    }

    std::vector<float> observation(106, -9.0f);
    contract.observe(s, s.links, s.centerOfMass, s.centerOfMassVelocity, std::vector<float>(21, 0.0f), observation);
    for (std::size_t k = 0; k < 3; ++k)
        expect(std::abs(observation[6 + k]) < 1e-6f, "o corpo sobre o lider: v_sim - v_kin = 0");
    for (std::size_t k = 49; k < 85; ++k)
        expect(std::abs(observation[k]) < 1e-5f, "o corpo sobre o lider: s_sim - s_kin = 0");
}

void rewardTests(const AthleteBundle3D& bundle) {
    const RagdollProfile3D& profile = bundle.profile();
    TrackingReward3D reward(profile);
    const TrackingLeader3D leader = warmLeader(bundle, 3.0f);
    const TrackingLeaderState3D& s = leader.state();
    const TrackingRewardTerms3D same = reward.evaluate(s, s.links, s.centerOfMass, s.centerOfMassVelocity);
    expect(std::abs(same.total - 4.0f) < 1e-4f && !same.terminate, "o corpo igual ao lider: as 4 parcelas inteiras");
    std::vector<PhysicsBodyState3D> moved = s.links;
    for (PhysicsBodyState3D& link : moved) link.position += Vec3 { 1.1f, 0.0f, 0.0f };
    const Vec3 com = rlBodyCenterOfMass3D(profile, moved);
    const TrackingRewardTerms3D far = reward.evaluate(s, moved, com, s.centerOfMassVelocity);
    expect(far.terminate && far.total == 0.0f, "a cabeca a mais de 1 m: acaba, com nota zero");
    expect(std::abs(far.position - 1.0f) < 1e-4f, "a posicao conta no referencial do centro de massa (so a pose)");
}

void batchTests() {
    TrackingTrainingSettings3D settings;
    settings.assetsDirectory = Assets;
    settings.environments = 2;
    settings.threads = 2;
    settings.evaluation = true;
    settings.evaluationSpeed = 3.0f;
    TrackingTraining3D batch(settings);
    for (std::size_t w = 0; w < 2; ++w) {
        const auto& leader = batch.leader(w).links;
        const auto& body = batch.body(w).links;
        for (std::size_t i = 0; i < leader.size(); ++i)
            expect((leader[i].position - body[i].position).length() < 1e-4f, "o comeco: o corpo sobre o lider");
    }
    std::vector<float> actions(2 * 21, 0.0f), rewards(2);
    std::vector<std::uint8_t> terminated(2), truncated(2);
    bool fell = false;
    for (int step = 0; step < 120 && !fell; ++step) {
        batch.step(actions, rewards, terminated, truncated);
        fell = terminated[0] != 0;
    }
    expect(fell, "sem rede (so o lider) o corpo cai: a malha aberta e instavel");
    const auto episodes = batch.takeEpisodes();
    expect(!episodes.empty() && episodes.front().fell, "o episodio da queda e registrado");
}

// A estrela: o piloto anda no trote pedido (nunca acima dele), o peito fixo nao gira, o girando gira e
// o livre acompanha o rumo; o exercicio do episodio e o da estrela.
void starTests() {
    const TrackingDrill3D drills[3] = { TrackingDrill3D::Star, TrackingDrill3D::StarFixed, TrackingDrill3D::StarTurning };
    for (const TrackingDrill3D drill : drills) {
        TrackingTrainingSettings3D settings;
        settings.assetsDirectory = Assets;
        settings.environments = 1;
        settings.threads = 1;
        settings.starSpeed = 3.0f;
        settings.starShare = drill == TrackingDrill3D::Star ? 1.0f : 0.0f;
        settings.starFixedShare = drill == TrackingDrill3D::StarFixed ? 1.0f : 0.0f;
        settings.starTurningShare = drill == TrackingDrill3D::StarTurning ? 1.0f : 0.0f;
        TrackingTraining3D batch(settings);
        std::vector<float> actions(21, 0.0f), rewards(1);
        std::vector<std::uint8_t> terminated(1), truncated(1);
        float maxSpeed = 0.0f, yawTravel = 0.0f, last = batch.lookYaw(0);
        Vec3 first = batch.leader(0).links[0].position;
        float farthest = 0.0f;
        for (int step = 0; step < 240; ++step) {
            batch.step(actions, rewards, terminated, truncated);
            const Vec3 v = batch.leader(0).requestedVelocityWorld;
            maxSpeed = std::max(maxSpeed, std::hypot(v.x, v.y));
            yawTravel += std::abs(batch.lookYaw(0) - last);
            last = batch.lookYaw(0);
            farthest = std::max(farthest, (batch.leader(0).links[0].position - first).length());
        }
        expect(maxSpeed <= 3.0f + 1e-3f && maxSpeed > 1.0f, "a estrela pede o trote, nunca acima dele");
        expect(farthest > 2.0f, "a estrela leva o lider para longe do comeco");
        if (drill == TrackingDrill3D::StarFixed) expect(yawTravel < 1e-4f, "estrela de peito fixo: o olhar nao gira");
        else expect(yawTravel > 0.3f, "estrela de peito livre ou girando: o olhar gira");
        for (const TrackingEpisode3D& episode : batch.takeEpisodes())
            expect(episode.drill == static_cast<std::uint8_t>(drill), "o episodio registra a estrela");
    }
}

} // namespace

int main() {
    try {
        const auto bundle = AthleteBundle3D::shared(Assets);
        contractTests(*bundle);
        rewardTests(*bundle);
        batchTests();
        starTests();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "rastreador: FALHOU: %s (%d checagens antes)\n", error.what(), checks);
        return 1;
    }
    std::printf("rastreador: %d checagens, tudo certo\n", checks);
    return 0;
}
