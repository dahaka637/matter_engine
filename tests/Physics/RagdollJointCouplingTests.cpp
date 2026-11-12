// O acoplamento da coluna (RagdollJointCoupling3D, 10/11): o perfil do jogador
// declara a torcao de Abdomen, Chest e UpperChest como uma coordenada nas
// parcelas 7:14:11; juntas pedidas em sentidos opostos saem no mesmo sentido, com
// a mesma soma, e a soma fica presa a amplitude do grupo.
#include "Engine/Physics/RagdollProfile3D.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

std::uint32_t linkOf(const RagdollProfile3D& profile, const std::string& id) {
    for (std::size_t i = 0; i < profile.links.size(); ++i)
        if (profile.links[i].id == id) return static_cast<std::uint32_t>(i);
    throw std::runtime_error("sem o link " + id);
}

} // namespace

int main() {
    try {
        const RagdollProfile3D profile = loadRagdollProfile3D(
            std::string(MATTERENGINE_TEST_ASSETS_DIR) + "/characters/football_player/FootballPlayerV1.ragdoll.json");
        require(profile.jointCouplings.size() == 1, "o perfil declara um acoplamento");
        const RagdollJointCoupling3D& twist = profile.jointCouplings.front();
        const std::uint32_t abdomen = linkOf(profile, "Abdomen"), chest = linkOf(profile, "Chest"),
                            upper = linkOf(profile, "UpperChest");
        require(twist.axis == RagdollAxis3D::Twist && twist.links == std::vector<std::uint32_t> { abdomen, chest, upper },
            "a torcao de Abdomen, Chest e UpperChest");
        require(std::abs(twist.shares[0] - 7.0f / 32.0f) < 1e-5f && std::abs(twist.shares[1] - 14.0f / 32.0f) < 1e-5f,
            "parcelas 7:14:11");

        const float degree = 3.14159265f / 180.0f;
        std::vector<Vec3> coordinates(profile.links.size());
        coordinates[abdomen].x = 10.0f * degree;
        coordinates[chest].x = -10.0f * degree;
        coordinates[upper].x = 10.0f * degree;
        coordinates[chest].y = 0.3f; // os outros eixos ficam como estao
        coupleRagdollJointCoordinates3D(profile, coordinates);
        require(coordinates[abdomen].x > 0.0f && coordinates[chest].x > 0.0f && coordinates[upper].x > 0.0f,
            "o zigue-zague sai no mesmo sentido");
        require(std::abs(coordinates[abdomen].x + coordinates[chest].x + coordinates[upper].x - 10.0f * degree) < 1e-5f,
            "a soma (o giro do peito sobre a pelve) fica");
        require(std::abs(coordinates[chest].x - 2.0f * coordinates[abdomen].x) < 1e-5f, "nas parcelas");
        require(coordinates[chest].y == 0.3f, "so o eixo acoplado muda");

        coordinates[abdomen].x = coordinates[chest].x = coordinates[upper].x = 30.0f * degree;
        coupleRagdollJointCoordinates3D(profile, coordinates);
        require(std::abs(coordinates[abdomen].x + coordinates[chest].x + coordinates[upper].x - 32.0f * degree) < 1e-4f,
            "a soma presa a amplitude anatomica (7 + 14 + 11 graus)");
        std::cout << "acoplamento da coluna: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FALHOU: " << error.what() << '\n';
        return 1;
    }
}
