// As regras do trote (G4): o pouso pelo ponto de captura (parado, a frente com v_des, a volta a
// casa so parado), a decolagem pela velocidade ou pelo pico com a perna estendida, a absorcao
// e a escolha do andar com histerese.
#include "Engine/BodyControl/Planning/RunningGait3D.hpp"

#include <cmath>
#include <cstdio>
#include <string>

using namespace MatterEngine;

namespace {
int failures = 0;
void check(bool condition, const std::string& message) {
    if (!condition) {
        ++failures;
        std::printf("FALHOU: %s\n", message.c_str());
    }
}
} // namespace

int main() {
    RunningLandingInput3D in;
    in.omega = 3.0;
    // Parado, sem velocidade: o pe pousa sob o CoM, meia largura para o lado dele.
    check((runningLandingPoint3D(in) - Eigen::Vector2d(0.0, 0.04)).norm() < 1e-12, "parado: meia largura para o lado");
    in.side = -1.0;
    check((runningLandingPoint3D(in) - Eigen::Vector2d(0.0, -0.04)).norm() < 1e-12, "parado: o outro lado");
    // Com velocidade igual a pedida: o neutro de Raibert, v T/2 a frente do CoM previsto no toque.
    in.side = 1.0;
    in.velocity = in.desiredVelocity = Eigen::Vector2d(1.0, 0.0);
    in.remaining = 0.1;
    check((runningLandingPoint3D(in) - Eigen::Vector2d(0.1 + 0.125, 0.04)).norm() < 1e-12, "em cruzeiro: o neutro");
    // Mais rapido que o pedido: o pe pousa mais a frente (freia), em (v - v_des)/omega.
    in.velocity = Eigen::Vector2d(1.3, 0.0);
    check(std::abs(runningLandingPoint3D(in).x() - (0.13 + 0.125 + 0.1)) < 1e-12, "rapido demais: o pe vai a frente");
    // A volta a casa so parado.
    in.homeError = Eigen::Vector2d(1.0, 0.0);
    check(std::abs(runningLandingPoint3D(in).x() - (0.13 + 0.125 + 0.1)) < 1e-12, "andando: a casa nao pesa");
    in.velocity = in.desiredVelocity = Eigen::Vector2d::Zero();
    in.remaining = 0.0;
    check(std::abs(runningLandingPoint3D(in).x() + 0.05) < 1e-12, "parado: atras do CoM, rumo a casa");
    // Nao cruza: empurrado para o lado do pe de apoio, o pe esquerdo fica a pelo menos 5 cm a esquerda dele.
    RunningLandingInput3D cross;
    cross.velocity = Eigen::Vector2d(0.0, -1.0);
    cross.omega = 3.0;
    cross.hasStance = true;
    cross.stance = Eigen::Vector2d(0.0, -0.04);
    check(std::abs(runningLandingPoint3D(cross).y() - 0.01) < 1e-12, "nao cruza o pe de apoio");
    cross.velocity = Eigen::Vector2d::Zero();
    check(std::abs(runningLandingPoint3D(cross).y() - 0.04) < 1e-12, "sem empurrao, a trava nao age");
    // A decolagem.
    check(!runningTakeoffReady3D(0.5, 0.4, 0.3, 0.6, 0.52), "joelho dobrado: nao decola");
    check(runningTakeoffReady3D(0.31, 0.29, 0.3, 0.4, 0.52), "velocidade atingida: decola");
    check(runningTakeoffReady3D(0.2, 0.22, 0.3, 0.4, 0.52), "pico acima de metade: decola");
    check(!runningTakeoffReady3D(0.12, 0.14, 0.3, 0.4, 0.52), "pico baixo demais: nao decola");
    check(!runningTakeoffReady3D(0.2, 0.18, 0.3, 0.4, 0.52), "ainda subindo abaixo de v0: espera");
    // A absorcao: parado na altura, zero; acima, desce; descendo, freia.
    check(std::abs(runningAbsorbAcceleration3D(0.9, 0.0, 0.9, 10.0)) < 1e-12, "na altura e parado: zero");
    check(runningAbsorbAcceleration3D(0.95, 0.0, 0.9, 10.0) < 0.0, "alto: desce");
    check(runningAbsorbAcceleration3D(0.9, -0.5, 0.9, 10.0) > 0.0, "descendo: freia");
    // O andar com histerese.
    check(!runningGaitSelectTrot3D(0.55, false, 0.6, 0.15), "abaixo do limiar: marcha");
    check(runningGaitSelectTrot3D(0.6, false, 0.6, 0.15), "no limiar: trote");
    check(runningGaitSelectTrot3D(0.5, true, 0.6, 0.15), "trotando, na histerese: segue trote");
    check(!runningGaitSelectTrot3D(0.44, true, 0.6, 0.15), "abaixo da histerese: volta a marcha");
    std::printf(failures ? "trote: %d falha(s)\n" : "trote: ok\n", failures);
    return failures ? 1 : 0;
}
