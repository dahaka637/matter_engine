// Testes da ponte com o jogo (MatterModelingRig) e da tela de inspecao sem
// janela: o modelo do jogador carregado do manifesto, o esqueleto do jogo, as
// poses, o jogador exportado para o jogo e a tela "Inspecao de jogador"
// rodando num contexto ImGui sem GPU (os erros que o ImGui recupera viram
// falha).

#include "Engine/Modeling/Analysis/DeformationCheck.hpp"
#include "Engine/Modeling/Assembly/FacePaint.hpp"
#include "Engine/Modeling/Assembly/PlayerAssembly.hpp"
#include "Engine/Modeling/Assembly/PlayerPaintSpaces.hpp"
#include "Engine/Modeling/Paint/PaintDesign.hpp"
#include "Engine/Modeling/Import/GltfImport.hpp"
#include "Engine/Modeling/Analysis/ModelValidator.hpp"
#include "Engine/Modeling/Inspection/ViewFraming.hpp"
#include "Engine/Modeling/Raster/ImageIO.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Mesh/TriangleBvh.hpp"
#include "Engine/Modeling/Rig/GameCharacterExport.hpp"
#include "Engine/Modeling/Rig/PlayerModelAsset.hpp"
#include "Support/HeadlessUi.hpp"
#include "Workbench/PlayerInspector/PlayerInspectorScreen.hpp"

#include "imgui.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <random>
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using namespace MatterEngine;
using namespace MatterEngine::Modeling;

namespace {

int checks = 0;

void require(bool condition, const std::string& message) {
    ++checks;
    if (!condition) throw std::runtime_error(message);
}

void testPlayerModel() {
    std::string error;
    const std::optional<LoadedPlayerModel> loaded = loadPlayerModel(defaultPlayerModelManifest(), {}, &error);
    require(loaded.has_value(), "modelo do jogador carregado: " + error);
    const InspectionModel& model = *loaded->model;
    const Skeleton& skeleton = model.skeleton;
    // O jogador montado das malhas base: o corpo (com as maos), a cabeca, a
    // camisa, o calcao e as chuteiras (cabedal, sola, travas e lingua; o
    // cadarco e pintado).
    {
        std::vector<std::string> names;
        for (const MeshPart& part : model.mesh.parts) names.push_back(part.name);
        const std::vector<std::string> expected { "corpo", "cabeca", "chuteira_cabedal_direita", "chuteira_sola_direita",
            "chuteira_travas_direita", "chuteira_lingua_direita", "camisa", "calcao", "chuteira_cabedal_esquerda",
            "chuteira_sola_esquerda", "chuteira_travas_esquerda", "chuteira_lingua_esquerda" };
        require(names == expected, "as partes do jogador montado");
        // Leve: o calcao subdividido (a dobra do quadril em curva, 10/10/2026:
        // o usuario liberou mais poligonos no uniforme) passou de 4,7 para
        // 6,1 mil.
        // Camisa e calcao em quads densos (22/10: a malha grossa nao dobrava):
        // 22 jogadores dao ~290 mil triangulos, folgado ate numa GTX 750.
        require(model.mesh.triangles.size() < 13000, "leve: menos de 13000 triangulos (" + std::to_string(model.mesh.triangles.size()) + ")");
        require(std::abs(model.mesh.bounds().maximum.z - model.mesh.bounds().minimum.z - 1.8f) < 0.005f, "1,80 m do chao ao alto");
    }
    const Report procedural = validateModel(model.mesh, &skeleton, &model.regions, model.jointRegion);
    require(procedural.count(Severity::Error) == 0, procedural.text());
    for (const auto& finding : procedural.findings())
        require(finding.severity == Severity::Info, procedural.text());
    // Uma superficie so (tudo costurado) e fechada: a cabeca fecha em cima.
    std::size_t openings = 0;
    for (const auto& finding : procedural.findings()) {
        if (finding.code == "abertura_declarada") ++openings;
        require(finding.code != "peca_solta" && finding.code != "borda_aberta", procedural.text());
    }
    require(openings == 0, "nada aberto: a cabeca fecha o corpo");
    for (const auto& [name, value] : procedural.statistics())
        if (name == "superficies (pedacos ligados)") require(value == "1", "uma superficie so: " + value);
    // Simetria exata (os numeros sao pintura, nao malha).
    for (Vec3 p : model.mesh.positions) {
        bool mirrored = false;
        for (Vec3 q : model.mesh.positions)
            if (distance(Vec3 { p.x, -p.y, p.z }, q) < 1e-7f) { mirrored = true; break; }
        require(mirrored, "simetria exata do modelo");
    }
    // A textura: um atlas para todos os materiais, cada triangulo mapeado; a
    // pintura no lugar (lida no atlas pelo ponto 3D da superficie). O atlas tem
    // a cabeca na metade direita (4096 x 2048).
    {
        const Mesh& mesh = model.mesh;
        require(mesh.hasUv() && mesh.textures.size() == 1 && mesh.textures.front().image.width() == 4096
                && mesh.textures.front().image.height() == 2048, "um atlas de 4096 x 2048");
        for (const MeshMaterial& material : mesh.materials) require(material.texture == 0, "material com a textura: " + material.name);
        // A cor no atlas no ponto da peca mais perto de "point" na direcao "facing".
        const auto paintAt = [&](std::string_view part, Vec3 point, Vec3 facing) {
            // O raio de "point" contra "facing" (vindo de fora): o primeiro triangulo
            // da peca que ele acerta e a cor do atlas no ponto.
            const int index = mesh.findPart(part);
            const Vec3 direction = -safeNormalized(facing, { 1.0f, 0.0f, 0.0f });
            float best = std::numeric_limits<float>::max();
            Color color {};
            for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
                if (mesh.partOf(t) != index) continue;
                const Vec3 a = mesh.positions[mesh.triangles[t][0]], b = mesh.positions[mesh.triangles[t][1]], c = mesh.positions[mesh.triangles[t][2]];
                const std::optional<float> hit = rayTriangle(point, direction, a, b, c);
                if (!hit || *hit >= best) continue;
                best = *hit;
                const Vec3 q = point + direction * *hit;
                const Vec3 n = cross(b - a, c - a);
                const float area = dot(n, n);
                const float wa = dot(cross(b - q, c - q), n) / area, wb = dot(cross(c - q, a - q), n) / area, wc = 1.0f - wa - wb;
                const std::array<Uv, 3>& uv = mesh.triangleUv[t];
                color = mesh.textures.front().image.sample({ uv[0].u * wa + uv[1].u * wb + uv[2].u * wc, uv[0].v * wa + uv[1].v * wb + uv[2].v * wc });
            }
            return color;
        };
        const float ground = skeleton.groundHeight;
        const auto dark = [](Color c) { return c.r + c.g + c.b < 0.6f; };
        const auto light = [](Color c) { return c.r + c.g + c.b > 2.4f; };
        // O "1" do peito (a haste a esquerda do centro, vista de frente) e a
        // camisa branca acima do numero.
        require(dark(paintAt("camisa", { 0.3f, -0.026f, ground + 1.3175f }, { 1.0f, 0.0f, 0.0f })), "o 1 do numero no peito");
        require(light(paintAt("camisa", { 0.3f, 0.0f, ground + 1.2f }, { 1.0f, 0.0f, 0.0f })), "a camisa branca abaixo do numero");
        require(light(paintAt("camisa", { 0.3f, -0.0086f, ground + 1.3175f }, { 1.0f, 0.0f, 0.0f })), "o vao entre o 1 e o 0");
        require(dark(paintAt("camisa", { -0.4f, 0.045f, ground + 1.31f }, { -1.0f, 0.0f, 0.0f })), "o 1 do numero nas costas (do lado esquerdo do jogador)");
        // O meiao: a faixa preta no alto (abaixo da dobra do joelho), branco
        // embaixo; o joelho e pele.
        require(dark(paintAt("corpo", { 0.3f, -0.13f, ground + 0.41f }, { 1.0f, 0.0f, 0.0f })), "a faixa do meiao");
        require(light(paintAt("corpo", { 0.3f, -0.13f, ground + 0.3f }, { 1.0f, 0.0f, 0.0f })), "o meiao branco");
        const Color knee = paintAt("corpo", { 0.3f, -0.13f, ground + 0.5f }, { 1.0f, 0.0f, 0.0f });
        require(knee.r > knee.b + 0.15f, "o joelho e pele");
        // O rosto pintado: a pupila escura no olho, a pele no meio da testa.
        require(dark(paintAt("cabeca", { 0.3f, -0.033f, ground + 1.6795f }, { 1.0f, 0.0f, 0.0f })), "a pupila do olho direito");
        require(dark(paintAt("cabeca", { 0.3f, 0.033f, ground + 1.6795f }, { 1.0f, 0.0f, 0.0f })), "a pupila do olho esquerdo");
        const Color forehead = paintAt("cabeca", { 0.3f, 0.0f, ground + 1.72f }, { 1.0f, 0.0f, 0.0f });
        require(forehead.r > forehead.b + 0.15f, "a testa e pele");
        // A orelha (a pintura no espaco da placa medida na malha): o ponto mais
        // escuro da placa fica na metade da frente (o canal nao e no meio), bem
        // mais escuro que a beira de cima, no mesmo lugar nos dois lados. (A
        // orelha da cabeca low-poly e uma placa de 3 triangulos: 5 pontos.)
        const FacePaintShape faceShape;
        const EarPlate plate = earPlate(mesh, mesh.findPart("cabeca"), faceShape.earOutside, ground);
        require(plate.outline.size() >= 5 && plate.major > 0.02f && plate.major < 0.06f && plate.minor > 0.01f && plate.minor < 0.04f
                && plate.center[1] > 1.6f && plate.center[1] < 1.75f,
            "a placa da orelha medida (" + std::to_string(plate.major) + " x " + std::to_string(plate.minor) + " m)");
        float minX = 1.0f, maxX = -1.0f, minH = 9.0f, maxH = -9.0f;
        for (const auto& q : plate.outline)
            minX = std::min(minX, q[0]), maxX = std::max(maxX, q[0]), minH = std::min(minH, q[1]), maxH = std::max(maxH, q[1]);
        const auto sum = [](Color c) { return c.r + c.g + c.b; };
        // So a grade dentro do contorno: a caixa da placa de 5 pontos pega o cabelo atras da orelha.
        const auto onPlate = [&](float x, float h) {
            bool inside = false;
            for (std::size_t i = 0, j = plate.outline.size() - 1; i < plate.outline.size(); j = i++) {
                const auto& a = plate.outline[i];
                const auto& b = plate.outline[j];
                if ((a[1] > h) != (b[1] > h) && x < (b[0] - a[0]) * (h - a[1]) / (b[1] - a[1]) + a[0]) inside = !inside;
            }
            return inside;
        };
        const auto darkest = [&](float side) {
            float best = 9.0f, bestU = 0.0f, bestV = 0.0f;
            for (int i = 1; i < 20; ++i)
                for (int j = 1; j < 20; ++j) {
                    const float u = static_cast<float>(i) / 20.0f, v = static_cast<float>(j) / 20.0f;
                    if (!onPlate(minX + u * (maxX - minX), minH + v * (maxH - minH))) continue;
                    const float value = sum(paintAt("cabeca", { minX + u * (maxX - minX), side * 0.3f, ground + minH + v * (maxH - minH) }, { 0.0f, side, 0.0f }));
                    if (value < best) {
                        best = value;
                        bestU = u;
                        bestV = v;
                    }
                }
            return std::array<float, 3> { best, bestU, bestV };
        };
        const std::array<float, 3> right = darkest(-1.0f), left = darkest(1.0f);
        const float rim = sum(paintAt("cabeca", { minX + 0.5f * (maxX - minX), -0.3f, ground + minH + 0.9f * (maxH - minH) }, { 0.0f, -1.0f, 0.0f }));
        require(right[1] > 0.5f && right[0] < 0.75f * rim, "o canal da orelha escuro e na frente (u " + std::to_string(right[1]) + ")");
        require(std::abs(left[1] - right[1]) < 0.06f && std::abs(left[2] - right[2]) < 0.06f && std::abs(left[0] - right[0]) < 0.08f,
            "a orelha esquerda e o espelho da direita");
        // Sem as faixas pretas do lado (23/10/2026, pedido do usuario: o contraste com o fundo): o lado
        // do calcao e da camisa e branco.
        require(light(paintAt("calcao", { -0.03f, -0.3f, ground + 0.75f }, { 0.0f, -1.0f, 0.0f })), "o lado do calcao branco (sem faixa)");
        require(light(paintAt("camisa", { -0.03f, -0.3f, ground + 1.05f }, { 0.0f, -1.0f, 0.0f })), "o lado da camisa branco (sem faixa)");
    }
    require(skeleton.validate().empty(), "esqueleto sem problemas de estrutura");
    require(model.regions.regions.size() == 18, "18 partes no menu");
    require(model.regions.validate(skeleton).empty(), "regioes batem com o esqueleto");
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
        require(model.jointRegion[j] >= 0, "junta sem regiao: " + skeleton.joints[j].name);
    std::size_t body = 0, fingers = 0, toes = 0, tips = 0, helpers = 0;
    for (const Joint& joint : skeleton.joints) {
        body += joint.kind == JointKind::Body ? 1 : 0;
        fingers += joint.kind == JointKind::Finger ? 1 : 0;
        toes += joint.kind == JointKind::Toe ? 1 : 0;
        tips += joint.kind == JointKind::Tip ? 1 : 0;
        helpers += joint.kind == JointKind::Helper ? 1 : 0;
    }
    require(body == 18 && fingers == 30 && toes == 2, "18 ossos fisicos, 30 dedos e 2 dedos do pe");
    require(tips == 13, "pontas: cabeca, 10 dedos e 2 pes");
    // No quadril, tres (1/4, 1/2 e 3/4 do giro: os aneis do calcao giram
    // inteiros em volta da junta) e a da dobra da frente (so a flexao).
    require(helpers == 26 && skeleton.find("RightClavicleHelper") >= 0 && skeleton.find("LeftClavicleHelper") >= 0 && skeleton.find("RightKneeHelper") >= 0 && skeleton.find("LeftHipHelper") >= 0
            && skeleton.find("RightHipHelper25") >= 0 && skeleton.find("LeftHipHelper75") >= 0
            && skeleton.find("RightHipLift") >= 0 && skeleton.find("LeftHipLift") >= 0
            && skeleton.find("RightGluteHelper") >= 0 && skeleton.find("LeftGluteHelper") >= 0
            && skeleton.find("RightAnkleHelper") >= 0 && skeleton.find("LeftShoulderHelper") >= 0
            && skeleton.find("RightArmpitHelper") >= 0 && skeleton.find("LeftArmpitHelper") >= 0
            && skeleton.find("RightArmpitFill") >= 0 && skeleton.find("LeftArmpitFill") >= 0
            && skeleton.find("RightElbowHelper") >= 0 && skeleton.find("LeftWristHelper") >= 0,
        "auxiliares: joelhos, quadris (cinco), tornozelos, ombros, axilas, claviculas, cotovelos e pulsos");
    // Os dedos da esquerda sao o espelho exato dos da direita (a malha e
    // espelhada; no jogo eles diferem uns milimetros).
    for (const Joint& joint : skeleton.joints) {
        const bool finger = joint.kind == JointKind::Finger
            || (joint.kind == JointKind::Tip && joint.parent >= 0
                && skeleton.joints[static_cast<std::size_t>(joint.parent)].kind == JointKind::Finger);
        if (!finger || joint.name.rfind("Left", 0) != 0) continue;
        const int right = skeleton.find("Right" + joint.name.substr(4));
        if (right < 0) continue;
        const Transform& other = skeleton.joints[static_cast<std::size_t>(right)].bind;
        const Vec3 p = other.position;
        require(distance(joint.bind.position, { p.x, -p.y, p.z }) < 1e-4f, "dedo espelhado: " + joint.name);
    }
    // A auxiliar do joelho gira a metade da canela em relacao a coxa.
    {
        SkeletonPose bent = bindPose(skeleton);
        const int shin = skeleton.find("RightShin"), helper = skeleton.find("RightKneeHelper");
        const Quaternion turn = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, 1.2f);
        const Vec3 pivot = bent.joints[static_cast<std::size_t>(shin)].position;
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
            int k = static_cast<int>(j);
            while (k >= 0 && k != shin) k = skeleton.joints[static_cast<std::size_t>(k)].parent;
            if (k != shin || skeleton.joints[j].kind == JointKind::Helper) continue;
            bent.joints[j].position = pivot + turn.rotate(bent.joints[j].position - pivot);
            bent.joints[j].orientation = (turn * bent.joints[j].orientation).normalized();
        }
        updateDerivedJoints(skeleton, bent);
        const Transform& half = bent.joints[static_cast<std::size_t>(helper)];
        require(distance(half.position, pivot) < 1e-5f, "a auxiliar fica no joelho");
        require(std::abs(angleBetween(half.orientation, skeleton.joints[static_cast<std::size_t>(helper)].bind.orientation) - 0.6f)
                < 1e-3f,
            "a auxiliar gira a metade");
    }
    // A da dobra da frente gira so a fracao da flexao (em volta do eixo de
    // lado): a coxa que abre de lado nao mexe nela; levantada e aberta, so a
    // parte levantada conta.
    {
        const int thigh = skeleton.find("RightThigh"), lift = skeleton.find("RightHipLift");
        const Joint& helper = skeleton.joints[static_cast<std::size_t>(lift)];
        const auto turned = [&](Quaternion turn) {
            SkeletonPose pose = bindPose(skeleton);
            const Vec3 pivot = pose.joints[static_cast<std::size_t>(thigh)].position;
            for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
                int k = static_cast<int>(j);
                while (k >= 0 && k != thigh) k = skeleton.joints[static_cast<std::size_t>(k)].parent;
                if (k != thigh || skeleton.joints[j].kind == JointKind::Helper) continue;
                pose.joints[j].position = pivot + turn.rotate(pose.joints[j].position - pivot);
                pose.joints[j].orientation = (turn * pose.joints[j].orientation).normalized();
            }
            updateDerivedJoints(skeleton, pose);
            return angleBetween(pose.joints[static_cast<std::size_t>(lift)].orientation, helper.bind.orientation);
        };
        // A clavicula leva o braco: erguido (girar em volta de y para a frente), o
        // ombro sobe; baixado (encostar no corpo), nada muda.
        {
            const int arm = skeleton.find("RightUpperArm"), hand = skeleton.find("RightHand");
            const auto raised = [&](Quaternion turn) {
                SkeletonPose pose = bindPose(skeleton);
                const Vec3 pivot = pose.joints[static_cast<std::size_t>(arm)].position;
                for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
                    int k = static_cast<int>(j);
                    while (k >= 0 && k != arm) k = skeleton.joints[static_cast<std::size_t>(k)].parent;
                    if (k != arm || skeleton.joints[j].kind == JointKind::Helper) continue;
                    pose.joints[j].position = pivot + turn.rotate(pose.joints[j].position - pivot);
                    pose.joints[j].orientation = (turn * pose.joints[j].orientation).normalized();
                }
                const Vec3 handBefore = pose.joints[static_cast<std::size_t>(hand)].position;
                const Quaternion armBefore = pose.joints[static_cast<std::size_t>(arm)].orientation;
                updateDerivedJoints(skeleton, pose);
                // (A clavicula leva o ombro sem girar o braco: a direcao e a pedida, 25/10.)
                require(angleBetween(pose.joints[static_cast<std::size_t>(arm)].orientation, armBefore) < 1e-4f,
                    "a clavicula leva o ombro sem girar a direcao do braco");
                const int clavicle = skeleton.find("RightClavicleHelper");
                return std::tuple { pose.joints[static_cast<std::size_t>(arm)].position.z - pivot.z,
                    (pose.joints[static_cast<std::size_t>(hand)].position - handBefore).z,
                    angleBetween(pose.joints[static_cast<std::size_t>(clavicle)].orientation, skeleton.joints[static_cast<std::size_t>(clavicle)].bind.orientation) };
            };
            const auto [up, handUp, lift] = raised(Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -2.6f));
            const auto [down, handDown, rest] = raised(Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 0.6f));
            require(up > 0.02f && handUp > 0.02f, "a clavicula sobe o ombro e leva o braco com o braco erguido");
            require(std::abs(down) < 1e-5f && std::abs(handDown) < 1e-5f && rest < 1e-4f, "com o braco baixado a clavicula fica");
            // O teto (25/10): erguido, ela nao passa de 28 graus (sem ele, ate 44) e ainda sobe bastante.
            const float cap = skeleton.joints[static_cast<std::size_t>(skeleton.find("RightClavicleHelper"))].derivedCap;
            require(cap > 0.0f && lift <= cap + 1e-3f && lift > 0.5f * cap, "a subida da clavicula fica no teto com o braco erguido");
            // A meia-junta do ombro (25/10): 50% do giro com o braco pouco erguido, perto de 65% na vertical
            // (a fracao pela diferenca entre o giro dela e o do braco: a clavicula leva os dois igual).
            const int shoulder = skeleton.find("RightShoulderHelper"), armpit = skeleton.find("RightArmpitHelper");
            const auto raisedPose = [&](float degrees) {
                SkeletonPose pose = bindPose(skeleton);
                const Quaternion turn = Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, -degrees * DegreesToRadians);
                const Vec3 pivot = pose.joints[static_cast<std::size_t>(arm)].position;
                for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
                    int k = static_cast<int>(j);
                    while (k >= 0 && k != arm) k = skeleton.joints[static_cast<std::size_t>(k)].parent;
                    if (k != arm || skeleton.joints[j].kind == JointKind::Helper) continue;
                    pose.joints[j].position = pivot + turn.rotate(pose.joints[j].position - pivot);
                    pose.joints[j].orientation = (turn * pose.joints[j].orientation).normalized();
                }
                updateDerivedJoints(skeleton, pose);
                return pose;
            };
            const auto shareAt = [&](int helper, float degrees) {
                const SkeletonPose pose = raisedPose(degrees);
                const auto skin = [&](int j) {
                    return (pose.joints[static_cast<std::size_t>(j)].orientation * skeleton.joints[static_cast<std::size_t>(j)].bind.orientation.conjugate()).normalized();
                };
                return 1.0f - angleBetween(skin(helper), skin(arm)) / (degrees * DegreesToRadians);
            };
            require(std::abs(shareAt(shoulder, 40.0f) - 0.5f) < 0.01f && std::abs(shareAt(shoulder, 146.0f) - 0.65f) < 0.02f,
                "a meia-junta do ombro gira metade com o braco pouco erguido e 65% na vertical");
            // A da axila (26/10): igual a meia-junta com o braco pouco erguido, o braco inteiro na vertical.
            require(armpit >= 0 && std::abs(shareAt(armpit, 40.0f) - 0.5f) < 0.01f && shareAt(armpit, 146.0f) > 0.98f,
                "a junta da axila gira metade com o braco pouco erguido e com o braco inteiro na vertical");
            // A do volume (26/10): fica com o peito e so anda (para fora) com o braco erguido.
            const int fill = skeleton.find("RightArmpitFill"), chest = skeleton.find("UpperChest");
            const auto fillShift = [&](float degrees) {
                const SkeletonPose pose = raisedPose(degrees);
                const Transform chestSkin = pose.joints[static_cast<std::size_t>(chest)] * skeleton.joints[static_cast<std::size_t>(chest)].bind.inverse();
                return pose.joints[static_cast<std::size_t>(fill)].position - chestSkin.apply(skeleton.joints[static_cast<std::size_t>(fill)].bind.position);
            };
            const Vec3 low = fillShift(40.0f), high = fillShift(146.0f);
            require(fill >= 0 && low.length() < 1e-5f && high.y < -0.03f,
                "a junta do volume fica com o peito com o braco pouco erguido e anda para fora com ele na vertical");
        }
        const Quaternion flexion = Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, -1.4f);
        const Quaternion abduction = Quaternion::fromAxisAngle({ 1.0f, 0.0f, 0.0f }, 0.5f);
        require(std::abs(turned(flexion) - helper.derivedFraction * 1.4f) < 1e-3f, "a da dobra gira a fracao da flexao");
        require(turned(abduction) < 1e-3f, "abrir a coxa de lado nao mexe na da dobra");
        require(turned(Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, 0.7f)) < 1e-3f, "a coxa atras nao mexe na da dobra");
        // A do gluteo e o contrario: so a extensao.
        const int glute = skeleton.find("RightGluteHelper");
        const auto gluteTurned = [&](Quaternion turn) {
            SkeletonPose pose = bindPose(skeleton);
            const Vec3 pivot = pose.joints[static_cast<std::size_t>(thigh)].position;
            for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
                int k = static_cast<int>(j);
                while (k >= 0 && k != thigh) k = skeleton.joints[static_cast<std::size_t>(k)].parent;
                if (k != thigh || skeleton.joints[j].kind == JointKind::Helper) continue;
                pose.joints[j].position = pivot + turn.rotate(pose.joints[j].position - pivot);
                pose.joints[j].orientation = (turn * pose.joints[j].orientation).normalized();
            }
            updateDerivedJoints(skeleton, pose);
            return angleBetween(pose.joints[static_cast<std::size_t>(glute)].orientation,
                skeleton.joints[static_cast<std::size_t>(glute)].bind.orientation);
        };
        require(glute >= 0 && gluteTurned(flexion) < 1e-3f, "a coxa a frente nao mexe na do gluteo");
        require(std::abs(gluteTurned(Quaternion::fromAxisAngle({ 0.0f, 1.0f, 0.0f }, 0.7f))
                    - skeleton.joints[static_cast<std::size_t>(glute)].derivedFraction * 0.7f) < 1e-3f,
            "a do gluteo gira a fracao da extensao");
        const Quaternion both = (abduction * flexion).normalized();
        const float twist = angleBetween(twistAbout(both, { 0.0f, 1.0f, 0.0f }), {});
        require(twist > 1.0f && std::abs(turned(both) - helper.derivedFraction * twist) < 1e-3f, "levantada e aberta: so a parte da flexao");
    }
    const int head = skeleton.find("Head"), top = skeleton.find("HeadTop");
    require(head >= 0 && top >= 0 && skeleton.joints[static_cast<std::size_t>(top)].bind.position.z
            > skeleton.joints[static_cast<std::size_t>(head)].bind.position.z + 0.1f,
        "alto da cabeca acima da junta da cabeca");
    // O chao do espaco do modelo e o do model.json (fixo); o repouso (a pose T
    // do jogo) fica em pe nele com a pelve na altura de pe do perfil (o jogo
    // tem a pelve na origem: o chao dele e outro).
    require(loaded->manifest.skeleton.ground && std::abs(skeleton.groundHeight - *loaded->manifest.skeleton.ground) < 1e-6f,
        "chao do espaco do modelo (model.json)");
    {
        const SkeletonPose rest = sampleGameRigPose(*loaded->character, *loaded->rig, nullptr, 0.0f, RigHandPose::Bind, true);
        const float pelvis = rest.joints[static_cast<std::size_t>(skeleton.find("Pelvis"))].position.z - skeleton.groundHeight;
        require(std::abs(pelvis - loaded->character->profile.standingRootHeightMeters) < 1e-3f,
            "a pelve do repouso na altura de pe do perfil");
    }
    require(skeleton.height() > 1.6f && skeleton.height() < 2.1f, "altura plausivel");
    for (const BodyRegion& region : model.regions.regions)
        require(regionBounds({ &skeleton, nullptr, nullptr, {}, &model.regions, model.jointRegion }, model.regions.find(region.id))
                    .valid(),
            "regiao com caixa: " + region.id);

    // Pose de uma animacao do jogo.
    const std::optional<AnimationClip3D> clip = loadCharacterClip("jog", &error);
    require(clip.has_value(), "animacao jog: " + error);
    const SkeletonPose pose = sampleGameRigPose(*loaded->character, *loaded->rig, &*clip, 0.25f);
    require(pose.joints.size() == skeleton.joints.size(), "pose com todas as juntas");
    // Nenhuma junta sai de onde o pai a poe (com o braco deslocado no modelo,
    // o cotovelo dobrado saia 4,5 cm do lugar).
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
        const int parent = skeleton.joints[j].parent;
        if (parent < 0 || skeleton.joints[j].kind == JointKind::Helper) continue;
        const Transform local = skeleton.joints[static_cast<std::size_t>(parent)].bind.inverse() * skeleton.joints[j].bind;
        Transform placed = pose.joints[static_cast<std::size_t>(parent)] * local;
        // (O braco e filho da clavicula: ela o leva.)
        for (std::size_t h = 0; h < skeleton.joints.size(); ++h)
            if (skeleton.joints[h].derivedCarry && skeleton.joints[h].derivedTo == static_cast<int>(j)) {
                const Transform chest = pose.joints[static_cast<std::size_t>(parent)] * skeleton.joints[static_cast<std::size_t>(parent)].bind.inverse();
                placed = pose.joints[h] * skeleton.joints[h].bind.inverse() * chest.inverse() * placed;
            }
        require(distance(placed.position, pose.joints[j].position) < 1e-4f, "junta no lugar na pose: " + skeleton.joints[j].name);
    }
    const int foot = skeleton.find("LeftFoot");
    require(distance(pose.joints[static_cast<std::size_t>(foot)].position,
                skeleton.joints[static_cast<std::size_t>(foot)].bind.position)
            > 0.02f,
        "a pose move o pe");

    // A deformacao nas poses do jogo, dobrando a coluna, o ombro, o
    // cotovelo, o pulso e os dedos, com a mao aberta e fechada: nenhum
    // triangulo virado a vista.
    {
        std::vector<PoseSample> samples;
        for (const char* id : { "natural_idle", "jog", "sprint", "alert_idle" }) {
            const std::optional<AnimationClip3D> animation = loadCharacterClip(id, &error);
            require(animation.has_value(), std::string("animacao ") + id + ": " + error);
            samples.push_back({ id, sampleGameRigPose(*loaded->character, *loaded->rig, &*animation, 0.25f) });
        }
        const auto bend = [&](const char* joint, Vec3 axis, float degrees) {
            const JointBend spine { skeleton.find(joint), axis, degrees * DegreesToRadians };
            samples.push_back({ std::string(joint) + std::to_string(static_cast<int>(degrees)),
                bendPose(skeleton, std::span<const JointBend>(&spine, 1)) });
        };
        bend("Abdomen", { 0.0f, 1.0f, 0.0f }, 30.0f);
        bend("Abdomen", { 0.0f, 1.0f, 0.0f }, -20.0f);
        bend("Abdomen", { 0.0f, 0.0f, 1.0f }, 25.0f);
        bend("Chest", { 0.0f, 1.0f, 0.0f }, 25.0f);
        bend("RightUpperArm", { 1.0f, 0.0f, 0.0f }, 45.0f);
        bend("RightUpperArm", { 1.0f, 0.0f, 0.0f }, -50.0f);
        bend("RightUpperArm", { 0.0f, 0.0f, 1.0f }, 70.0f);
        bend("RightForearm", { 0.0f, 0.0f, 1.0f }, 90.0f);
        bend("RightHand", { 1.0f, 0.0f, 0.0f }, 70.0f);
        bend("RightHand", { 1.0f, 0.0f, 0.0f }, -60.0f);
        bend("RightHand", { 0.0f, 0.0f, 1.0f }, 25.0f);
        // O pescoco e a cabeca: para a frente, para tras, de lado e girando.
        bend("Neck", { 0.0f, 1.0f, 0.0f }, 30.0f);
        bend("Neck", { 0.0f, 1.0f, 0.0f }, -30.0f);
        bend("Neck", { 1.0f, 0.0f, 0.0f }, 30.0f);
        bend("Neck", { 0.0f, 0.0f, 1.0f }, 45.0f);
        bend("Head", { 0.0f, 1.0f, 0.0f }, 35.0f);
        bend("Head", { 0.0f, 1.0f, 0.0f }, -35.0f);
        {
            // O pescoco e a cabeca girando juntos ate o fim dos controles da
            // tela (95 graus): o giro repartido pela faixa do pescoco.
            const std::array<JointBend, 2> twist { JointBend { skeleton.find("Neck"), { 0.0f, 0.0f, 1.0f }, 50.0f * DegreesToRadians },
                JointBend { skeleton.find("Head"), { 0.0f, 0.0f, 1.0f }, 45.0f * DegreesToRadians } };
            samples.push_back({ "pescoco e cabeca girando", bendPose(skeleton, twist) });
        }
        for (const RigHandPose hand : { RigHandPose::Open, RigHandPose::Fist })
            samples.push_back({ hand == RigHandPose::Open ? "mao aberta" : "punho",
                sampleGameRigPose(*loaded->character, *loaded->rig, nullptr, 0.0f, hand) });
        {
            // As dobras fortes como na tela (a dobradica de cada junta, presa
            // ao pai): o cotovelo e o joelho fechados, o pulso dos dois lados,
            // o braco levantado com o cotovelo dobrado e a mao fechando da
            // solta ate o punho.
            const SkeletonPose bind = bindPose(skeleton);
            const auto hinge = [&](const char* joint, float degrees) {
                const int index = skeleton.find(joint);
                return JointBend { index, jointHingeAxis(skeleton, bind, index), degrees * DegreesToRadians, true };
            };
            const auto severe = [&](const std::string& label, std::vector<JointBend> bends) {
                samples.push_back({ label, bendPose(skeleton, bends) });
            };
            severe("cotovelo 145", { hinge("RightForearm", 145.0f) });
            severe("joelho 150", { hinge("RightShin", 150.0f) });
            severe("pulso 70", { hinge("RightHand", 70.0f) });
            severe("pulso -70", { hinge("RightHand", -70.0f) });
            severe("braco para cima e cotovelo 120",
                { JointBend { skeleton.find("RightUpperArm"), { 1.0f, 0.0f, 0.0f }, -146.0f * DegreesToRadians, true },
                    hinge("RightForearm", 120.0f) });
            severe("coxa a frente e joelho 130",
                { JointBend { skeleton.find("RightThigh"), { 0.0f, 1.0f, 0.0f }, -90.0f * DegreesToRadians, true }, hinge("RightShin", 130.0f) });
            const SkeletonPose relaxed = withGameHands(*loaded->character, *loaded->rig, bind, RigHandPose::Relaxed);
            const SkeletonPose fist = withGameHands(*loaded->character, *loaded->rig, bind, RigHandPose::Fist);
            samples.push_back({ "mao fechando da solta",
                bendPose(skeleton, relaxed, blendJointBends(skeleton, relaxed, fist, fingerJoints(skeleton, Side::Right, false), 1.0f)) });
            // O polegar contraido ate o fim do controle da tela (a base 32, o
            // meio 64 e a ponta 80 graus, na dobradica de cada junta) de cada
            // mao de partida, sozinho e com a mao fechando 5% alem do punho do
            // jogo (a folga: o punho e o mais fechado que o jogo e a tela
            // fazem): a pele de dentro das juntas nao vira do avesso.
            for (const RigHandPose hand : { RigHandPose::Bind, RigHandPose::Relaxed, RigHandPose::Open }) {
                const SkeletonPose start = withGameHands(*loaded->character, *loaded->rig, bind, hand);
                const std::string from = hand == RigHandPose::Bind ? " (reta)" : hand == RigHandPose::Relaxed ? " (solta)" : " (aberta)";
                std::vector<JointBend> thumb;
                for (int k = 1; k <= 3; ++k) {
                    const int index = skeleton.find("RightThumb" + std::to_string(k));
                    thumb.push_back({ index, jointHingeAxis(skeleton, start, index), (k == 1 ? 32.0f : k == 2 ? 64.0f : 80.0f) * DegreesToRadians,
                        true });
                }
                samples.push_back({ "polegar contraido" + from, bendPose(skeleton, start, thumb) });
                std::vector<JointBend> closing = blendJointBends(skeleton, start, fist, fingerJoints(skeleton, Side::Right, false), 1.05f);
                samples.push_back({ "mao fechando" + from, bendPose(skeleton, start, closing) });
                closing.insert(closing.end(), thumb.begin(), thumb.end());
                samples.push_back({ "punho e polegar contraido" + from, bendPose(skeleton, start, closing) });
            }
        }
        // (Os dedos a 85/100/70 graus por cima da curva que eles ja tem na
        // ligacao passavam do punho do jogo - o medio 17 a 24 graus, o minimo
        // ate 37, o polegar 85 na base -; a mao fechando alem do punho, acima,
        // e a folga que vale.)
        Report deformation;
        checkDeformation(model.mesh, skeleton, samples, deformation);
        for (const Finding& finding : deformation.findings()) {
            // Nada vira a vista em parte nenhuma (dentro das pregas da axila,
            // do cotovelo e do joelho dobrados e so observacao).
            std::string where;
            for (const std::uint32_t t : finding.triangles) where += " T" + std::to_string(t);
            if (finding.hasLocation)
                where += " em (" + std::to_string(finding.location.x) + "; " + std::to_string(finding.location.y) + "; "
                    + std::to_string(finding.location.z) + ")";
            require(finding.code != "virou_pose", "virado a vista: " + finding.message + " (" + finding.part + ")" + where);
        }
    }

    // A camisa so no tronco (22/10/2026, decisao do usuario: mexer a perna
    // nunca deforma a camisa; o calcao pode entrar na barra): nenhum vertice
    // da camisa com peso das juntas da coxa.
    {
        const Mesh& mesh = model.mesh;
        const int shirt = mesh.findPart("camisa");
        std::vector<bool> thigh(skeleton.joints.size(), false);
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j)
            for (const char* end : { "HipLift", "HipHelper", "HipHelper25", "HipHelper75", "Thigh", "GluteHelper" })
                thigh[j] = thigh[j] || skeleton.joints[j].name.ends_with(end);
        std::size_t touched = 0;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            if (mesh.partOf(t) != shirt) continue;
            for (const std::uint32_t v : mesh.triangles[t])
                for (std::size_t k = 0; k < 4; ++k)
                    touched += mesh.skin[v].weights[k] > 1e-4f && thigh[mesh.skin[v].joints[k]] ? 1 : 0;
        }
        require(shirt >= 0 && mesh.hasSkin() && touched == 0, "a camisa so no tronco (" + std::to_string(touched) + " pesos da coxa)");
    }

    // O braco so mexe a camisa perto dele (22/10/2026, pedido do usuario: erguer o
    // braco deformava o peito, a barriga e as costas): a mais de 14 cm do segmento
    // ombro-cotovelo dos dois lados, nenhum peso do braco nem da clavicula.
    {
        const Mesh& mesh = model.mesh;
        const int shirt = mesh.findPart("camisa");
        std::vector<bool> arm(skeleton.joints.size(), false);
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j) {
            const std::string& name = skeleton.joints[j].name;
            if (!name.starts_with("Left") && !name.starts_with("Right")) continue;
            for (const char* piece : { "UpperArm", "ShoulderHelper", "ArmpitHelper", "ClavicleHelper", "Forearm", "ElbowHelper", "Hand", "WristHelper", "Thumb",
                                       "Index", "Middle", "Ring", "Pinky" })
                arm[j] = arm[j] || name.find(piece) != std::string::npos;
        }
        const auto at = [&](const char* name) { return skeleton.joints[static_cast<std::size_t>(skeleton.find(name))].bind.position; };
        const auto toSegment = [](Vec3 p, Vec3 a, Vec3 b) {
            const float t = std::clamp(dot(p - a, b - a) / dot(b - a, b - a), 0.0f, 1.0f);
            return distance(p, a + (b - a) * t);
        };
        const Vec3 rightShoulder = at("RightUpperArm"), rightElbow = at("RightForearm"), leftShoulder = at("LeftUpperArm"), leftElbow = at("LeftForearm");
        std::vector<bool> seen(mesh.positions.size(), false);
        std::size_t touched = 0;
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
            if (mesh.partOf(t) != shirt) continue;
            for (const std::uint32_t v : mesh.triangles[t]) {
                if (seen[v]) continue;
                seen[v] = true;
                const Vec3 p = mesh.positions[v];
                if (std::min(toSegment(p, rightShoulder, rightElbow), toSegment(p, leftShoulder, leftElbow)) <= 0.14f) continue;
                for (std::size_t k = 0; k < 4; ++k)
                    touched += mesh.skin[v].weights[k] > 1e-4f && arm[mesh.skin[v].joints[k]] ? 1 : 0;
            }
        }
        require(shirt >= 0 && touched == 0, "o braco so mexe a camisa perto dele (" + std::to_string(touched) + " pesos do braco longe dele)");
    }

    // O calcao com as pernas abertas (23/10/2026, pedido do usuario: o meio do calcao virava cortina entre as coxas;
    // a forma parada fica - o gancho alto foi desfeito): a parede de dentro de cada perna, da barra ate 5 cm abaixo
    // do gancho, segue a coxa do lado (fora a dobra de baixo do gluteo, atras).
    {
        const Mesh& mesh = model.mesh;
        const int shorts = mesh.findPart("calcao");
        const int rightThigh = skeleton.find("RightThigh"), leftThigh = skeleton.find("LeftThigh");
        const float hip = skeleton.joints[static_cast<std::size_t>(rightThigh)].bind.position.z;
        std::vector<bool> inShorts(mesh.positions.size(), false);
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (mesh.partOf(t) == shorts)
                for (const std::uint32_t v : mesh.triangles[t]) inShorts[v] = true;
        float crotch = 1.0e9f;
        for (std::size_t v = 0; v < mesh.positions.size(); ++v)
            if (inShorts[v] && std::abs(mesh.positions[v].y) < 0.004f && mesh.positions[v].z > hip - 0.20f)
                crotch = std::min(crotch, mesh.positions[v].z);
        std::size_t inner = 0, loose = 0;
        for (std::size_t v = 0; v < mesh.positions.size(); ++v) {
            const Vec3 p = mesh.positions[v];
            if (!inShorts[v] || std::abs(p.y) < 0.025f || std::abs(p.y) > 0.055f || p.x < -0.08f || p.z > crotch - 0.05f
                || p.z < hip - 0.26f)
                continue;
            const int thigh = p.y < 0.0f ? rightThigh : leftThigh;
            float w = 0.0f;
            for (std::size_t k = 0; k < 4; ++k)
                if (mesh.skin[v].joints[k] == thigh) w += mesh.skin[v].weights[k];
            ++inner;
            loose += w < 0.95f ? 1 : 0;
        }
        require(shorts >= 0 && inner > 0 && loose == 0, "a parede de dentro da perna do calcao segue a coxa (" + std::to_string(loose) + " de "
            + std::to_string(inner) + " vertices soltos)");
    }

    // O polegar fecha atravessando a palma, rente a ela (o eixo de cada
    // junta, como os controles da tela), e as duas maos sao o espelho uma da
    // outra: com o polegar dobrado assim e nas maos do jogo (o polegar
    // esquerdo do punho saia para fora).
    {
        // As coordenadas da palma de uma junta: x do pulso ao medio, y do
        // minimo ao indicador, z para o dorso (em comprimentos da mao; iguais
        // nas duas maos se uma for o espelho da outra).
        const auto palm = [&](const SkeletonPose& pose, const std::string& side, const std::string& joint) {
            const auto at = [&](const std::string& name) {
                return pose.joints[static_cast<std::size_t>(skeleton.find(side + name))].position;
            };
            const Vec3 wrist = at("Hand"), along = at("Middle1") - wrist;
            const Vec3 x = safeNormalized(along, { 1.0f, 0.0f, 0.0f });
            const Vec3 across = at("Index1") - at("Pinky1");
            const Vec3 y = safeNormalized(across - x * dot(across, x), { 0.0f, 1.0f, 0.0f });
            const Vec3 z = cross(x, y) * (side == "Left" ? -1.0f : 1.0f);
            const Vec3 d = at(joint) - wrist;
            const float length = std::max(1e-6f, along.length());
            return Vec3 { dot(d, x) / length, dot(d, y) / length, dot(d, z) / length };
        };
        std::vector<JointBend> thumbs;
        for (const char* side : { "Right", "Left" })
            for (int k = 1; k <= 3; ++k) {
                const int joint = skeleton.find(std::string(side) + "Thumb" + std::to_string(k));
                require(joint >= 0, std::string(side) + "Thumb" + std::to_string(k));
                thumbs.push_back({ joint,
                    fingerFlexionAxis(skeleton.joints[static_cast<std::size_t>(joint)].bind.orientation, true,
                        std::string_view(side) == "Left"),
                    (k == 1 ? 32.0f : k == 2 ? 64.0f : 80.0f) * DegreesToRadians });
            }
        const SkeletonPose closed = bendPose(skeleton, thumbs);
        const Vec3 open = palm(bindPose(skeleton), "Right", "ThumbTip"), shut = palm(closed, "Right", "ThumbTip");
        require(open.y > 0.3f && shut.y < 0.0f && shut.z < 0.0f && shut.x < 0.5f * open.x,
            "o polegar fecha atravessando a palma: ponta de (" + std::to_string(open.x) + ", " + std::to_string(open.y) + ", "
                + std::to_string(open.z) + ") para (" + std::to_string(shut.x) + ", " + std::to_string(shut.y) + ", "
                + std::to_string(shut.z) + ")");
        std::vector<std::pair<std::string, SkeletonPose>> poses { { "polegar dobrado", closed } };
        for (const RigHandPose hand : { RigHandPose::Relaxed, RigHandPose::Open, RigHandPose::Fist })
            poses.push_back({ hand == RigHandPose::Relaxed ? "mao solta" : hand == RigHandPose::Open ? "mao aberta" : "punho",
                sampleGameRigPose(*loaded->character, *loaded->rig, nullptr, 0.0f, hand, true) });
        for (const auto& [label, pose] : poses)
            for (const char* finger : { "Thumb", "Index", "Middle", "Ring", "Pinky" })
                for (const char* joint : { "1", "2", "3", "Tip" }) {
                    const std::string name = std::string(finger) + joint;
                    const float apart = distance(palm(pose, "Right", name), palm(pose, "Left", name));
                    require(apart < 0.05f, label + ": " + name + " nao e o espelho na outra mao (" + std::to_string(apart) + ")");
                }
    }

    // Os ossos dos dedos: cada falange dobra no plano em que o dedo curva na
    // malha (o indicador da malha base curva 26 graus fora do lado da mao; com
    // o lado, ele fechava torto).
    for (const char* side : { "Right", "Left" })
        for (const char* finger : { "Index", "Middle", "Ring", "Pinky" }) {
            const auto at = [&](const std::string& joint) {
                return skeleton.joints[static_cast<std::size_t>(skeleton.find(std::string(side) + finger + joint))].bind;
            };
            const Vec3 p1 = at("1").position, p2 = at("2").position, p3 = at("3").position, tip = at("Tip").position;
            const Vec3 plane = safeNormalized(cross(p2 - p1, p3 - p2) + cross(p3 - p2, tip - p3), {});
            for (const char* joint : { "1", "2", "3" }) {
                const float off = std::acos(std::min(1.0f, std::abs(dot(-axisY(at(joint).orientation), plane)))) / DegreesToRadians;
                require(off < 6.0f, std::string(side) + finger + joint + ": a dobradica " + std::to_string(off) + " graus fora do plano do dedo");
            }
        }

    // A mao do jogo como partida: so os dedos mudam (o corpo fica na
    // ligacao); fechando ate o punho, qualquer partida chega a mesma mao.
    {
        const SkeletonPose bind = bindPose(skeleton);
        const SkeletonPose fist = withGameHands(*loaded->character, *loaded->rig, bind, RigHandPose::Fist);
        const SkeletonPose game = sampleGameRigPose(*loaded->character, *loaded->rig, nullptr, 0.0f, RigHandPose::Fist, true);
        const auto local = [&](const SkeletonPose& pose, int joint) {
            const int parent = skeleton.joints[static_cast<std::size_t>(joint)].parent;
            return (pose.joints[static_cast<std::size_t>(parent)].orientation.conjugate() * pose.joints[static_cast<std::size_t>(joint)].orientation)
                .normalized();
        };
        const auto apart = [](Quaternion a, Quaternion b) {
            return 2.0f * std::acos(std::min(1.0f, std::abs(a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w))) / DegreesToRadians;
        };
        for (const char* body : { "RightUpperArm", "RightForearm", "RightHand", "LeftHand", "Head" }) {
            const int joint = skeleton.find(body);
            require(distance(fist.joints[static_cast<std::size_t>(joint)].position, bind.joints[static_cast<std::size_t>(joint)].position) < 1e-5f
                    && apart(fist.joints[static_cast<std::size_t>(joint)].orientation, bind.joints[static_cast<std::size_t>(joint)].orientation) < 0.01f,
                std::string("a mao de partida nao mexe no corpo: ") + body);
        }
        std::vector<int> fingers = fingerJoints(skeleton, Side::Right, true);
        const std::vector<int> left = fingerJoints(skeleton, Side::Left, true);
        fingers.insert(fingers.end(), left.begin(), left.end());
        require(fingers.size() == 30, "15 juntas de dedo em cada mao");
        for (const int joint : fingers)
            require(apart(local(fist, joint), local(game, joint)) < 0.5f, "os dedos tomam o giro do punho do jogo: " + skeleton.joints[static_cast<std::size_t>(joint)].name);
        for (const RigHandPose hand : { RigHandPose::Bind, RigHandPose::Relaxed, RigHandPose::Open }) {
            const SkeletonPose start = withGameHands(*loaded->character, *loaded->rig, bind, hand);
            const SkeletonPose closed
                = bendPose(skeleton, start, blendJointBends(skeleton, start, fist, fingerJoints(skeleton, Side::Right, false), 1.0f));
            for (const int joint : fingerJoints(skeleton, Side::Right, false))
                require(apart(local(closed, joint), local(fist, joint)) < 0.5f,
                    "fechando, toda partida chega ao punho: " + skeleton.joints[static_cast<std::size_t>(joint)].name + " "
                        + std::to_string(apart(local(closed, joint), local(fist, joint))));
        }
    }

    // Todas as animacoes do jogo (seis instantes de cada, a mao solta e o
    // punho): nenhum triangulo virado a vista em parte nenhuma (nem dobrado
    // do avesso contra os vizinhos, como a barra da camisa na largada
    // agachada).
    {
        std::size_t poses = 0;
        DeformationSettings settings;
        settings.intersections = false;
        for (const std::string& id : availableClipIds()) {
            const std::optional<AnimationClip3D> animation = loadCharacterClip(id, &error);
            require(animation.has_value(), "animacao " + id + ": " + error);
            std::vector<PoseSample> samples;
            for (int k = 0; k < 6; ++k)
                for (const RigHandPose hand : { RigHandPose::Relaxed, RigHandPose::Fist }) {
                    const float time = animation->durationSeconds * static_cast<float>(k) / 6.0f;
                    samples.push_back({ id + "@" + std::to_string(time) + (hand == RigHandPose::Fist ? " punho" : ""),
                        sampleGameRigPose(*loaded->character, *loaded->rig, &*animation, time, hand, true) });
                }
            poses += samples.size();
            Report report;
            checkDeformation(model.mesh, skeleton, samples, report, settings);
            for (const Finding& finding : report.findings()) {
                std::string where;
                for (const std::uint32_t t : finding.triangles) where += " T" + std::to_string(t);
                if (finding.hasLocation)
                    where += " em (" + std::to_string(finding.location.x) + "; " + std::to_string(finding.location.y) + "; "
                        + std::to_string(finding.location.z) + ")";
                require(finding.code != "virou_pose", "virado a vista: " + finding.message + " (" + finding.part + ")" + where);
            }
        }
        require(poses >= 400, "as animacoes do jogo carregadas: " + std::to_string(poses) + " poses");
    }

}

// A pele de um personagem do jogo (skin) contra a malha posada da modelagem:
// cada vertice da pele e os da modelagem na mesma posicao da ligacao (twins),
// o jogo com a pelve na origem (lift acima do modelo). As poses: o repouso,
// quatro animacoes e os bracos erguidos de lado e a frente (nenhuma animacao os
// ergue tanto: a rampa e o volume da axila e a clavicula). game e rig posam a
// modelagem; na pele os clipes andam a raiz x clipScale e abrem o braco em
// armOpening (bodyAnimationClip3D, como o jogo num corpo). O pior desvio de cada pose (m) e o mais baixo da pele
// no repouso (no espaco do jogo).
struct SkinParity {
    std::vector<std::pair<std::string, float>> worst;
    float lowest = std::numeric_limits<float>::max();
};

SkinParity skinParity(const RagdollCharacter3D& skin, const RagdollCharacter3D& game, const GameRig& rig, const Mesh& mesh,
    const std::vector<std::vector<std::size_t>>& twins, Vec3 lift, float clipScale, float armOpening = 0.0f) {
    SkinParity result;
    std::vector<Quaternion> visual(skin.visualBones.size());
    for (std::size_t v = 0; v < visual.size(); ++v) visual[v] = skin.visualBones[v].restRotation;
    const auto compare = [&](const std::string& name, const RagdollAnimationPose3D& links, const SkeletonPose& pose, bool atRest) {
        const std::vector<Mat4> palette = buildRagdollSkinMatrices3D(skin, links.linkPositions, links.linkOrientations, visual);
        const std::vector<Vec3> posed = mesh.posedPositions(skinningTransforms(rig.skeleton, pose));
        float worst = 0.0f;
        for (std::size_t g = 0; g < skin.mesh.vertices.size(); ++g) {
            const Vec3 at = skinVertexPosition3D(skin.mesh.vertices[g], palette) - lift;
            float best = std::numeric_limits<float>::max();
            for (const std::size_t twin : twins[g]) best = std::min(best, distance(at, posed[twin]));
            worst = std::max(worst, best);
            if (atRest) result.lowest = std::min(result.lowest, at.z + lift.z);
        }
        result.worst.push_back({ name, worst });
    };
    const struct {
        const char* clip;
        float time;
    } poses[] = { { "", 0.0f }, { "alert_idle", 0.4f }, { "sprint", 0.15f }, { "tackle_slide", 0.6f }, { "stand_up_back_kimodo", 1.2f } };
    for (const auto& entry : poses) {
        std::optional<AnimationClip3D> clip, scaled;
        if (entry.clip[0] != '\0') {
            std::string error;
            clip = loadCharacterClip(entry.clip, &error);
            require(clip.has_value(), error);
            scaled = bodyAnimationClip3D(*clip, skin.profile, clipScale, armOpening, 0.0f);
        }
        compare(entry.clip[0] ? entry.clip : "repouso", sampleRagdollAnimationPose3D(skin.profile, scaled ? &*scaled : nullptr, entry.time, true),
            sampleGameRigPose(game, rig, clip ? &*clip : nullptr, entry.time, RigHandPose::Relaxed, false), entry.clip[0] == '\0');
    }
    const auto link = [&](const std::string& id) {
        for (std::size_t i = 0; i < skin.profile.links.size(); ++i)
            if (skin.profile.links[i].id == id) return i;
        throw std::runtime_error("sem o link " + id);
    };
    const RagdollAnimationPose3D standing = sampleRagdollAnimationPose3D(skin.profile, nullptr, 0.0f, true);
    for (const auto& [name, axis, degrees] : { std::tuple { "bracos erguidos de lado", Vec3 { -1.0f, 0.0f, 0.0f }, 150.0f },
             std::tuple { "bracos erguidos a frente", Vec3 { 0.0f, -1.0f, 0.0f }, 120.0f } }) {
        RagdollAnimationPose3D links = standing;
        for (const std::string side : { "Right", "Left" }) {
            // O esquerdo e o espelho (o eixo de giro espelhado em y).
            const Quaternion turn = Quaternion::fromAxisAngle(side == "Right" ? axis : Vec3 { -axis.x, axis.y, -axis.z }, degrees * DegreesToRadians);
            const Vec3 pivot = skin.profile.links[link(side + "UpperArm")].inboundJoint.anchorModelPosition;
            for (const char* part : { "UpperArm", "Forearm", "Hand" }) {
                const std::size_t i = link(side + part);
                links.linkPositions[i] = pivot + turn.rotate(links.linkPositions[i] - pivot);
                links.linkOrientations[i] = (turn * links.linkOrientations[i]).normalized();
            }
        }
        compare(name, links, gameRigPoseFromLinks(game, rig, links, visual, false, true), false);
    }
    return result;
}

// O corpo variavel (26/10): em cada corpo a pele do jogo (o personagem do
// corpo) deforma igual a modelagem com o corpo (cada vertice da pele contra o
// da modelagem de onde ele saiu) nas poses da paridade, a sola fica no chao do
// perfil do corpo e a altura vai na escala. No corpo de hoje nada muda.
void testBodyVariants() {
    std::string error;
    const std::optional<LoadedPlayerModel> base = loadPlayerModel(defaultPlayerModelManifest(), {}, &error);
    require(base.has_value(), "modelo do jogador carregado: " + error);
    {
        LoadedPlayerModel same = *base;
        applyPlayerBody(same, PlayerBody3D {});
        // Os mesmos objetos: bit a bit.
        require(same.character == base->character && same.rig == base->rig && same.model == base->model, "o corpo de hoje nao muda nada");
    }
    const std::optional<AnimationClip3D> sprint = loadCharacterClip("sprint", &error);
    require(sprint.has_value(), error);
    const PlayerBody3D bodies[] = { { 1.70f, 60.0f }, { 1.70f, 100.0f }, { 2.00f, 60.0f }, { 2.00f, 100.0f }, { 1.87f, 72.0f }, { 1.80f, 60.0f } };
    for (const PlayerBody3D& body : bodies) {
        LoadedPlayerModel loaded = *base;
        applyPlayerBody(loaded, body);
        const float s = bodyScale3D(body);
        const RagdollCharacter3D& skin = *loaded.character;
        const GameRig& rig = *loaded.rig;
        const InspectionModel& model = *loaded.model;
        char label[48];
        std::snprintf(label, sizeof label, "%.2f m %.0f kg", static_cast<double>(body.heightMeters), static_cast<double>(body.massKg));
        require(skin.surfaceVertices.size() == skin.mesh.vertices.size() && model.mesh.positions.size() == base->model->mesh.positions.size(),
            std::string("a pele com a origem dos vertices: ") + label);
        std::vector<std::vector<std::size_t>> twins(skin.mesh.vertices.size());
        for (std::size_t g = 0; g < twins.size(); ++g) twins[g] = { skin.surfaceVertices[g].source };
        const SkeletonPose rest = sampleGameRigPose(skin, rig, nullptr, 0.0f, RigHandPose::Bind, true);
        const Vec3 lift = skin.profile.links.front().modelPosition - rest.joints[static_cast<std::size_t>(rig.skeleton.find("Pelvis"))].position;
        const SkinParity parity = skinParity(skin, skin, rig, model.mesh, twins, lift, s, bodyArmOpening3D(bodyTone3D(body)));
        std::string line = std::string("corpo ") + label + ":";
        for (const auto& [pose, worst] : parity.worst) line += " " + pose + " " + std::to_string(worst * 1000.0f) + " mm;";
        const float sole = parity.lowest + skin.profile.standingRootHeightMeters;
        const float skeletonHeight = model.skeleton.height(), meshHeight = model.mesh.bounds().maximum.z - model.skeleton.groundHeight;
        line += " sola " + std::to_string(sole * 1000.0f) + " mm; altura " + std::to_string(skeletonHeight) + " m (malha "
            + std::to_string(meshHeight) + ", esperado " + std::to_string(1.7995f * s) + ")";
        std::cout << line << '\n';
        for (const auto& [pose, worst] : parity.worst)
            require(worst < 0.0005f, std::string("a pele do corpo ") + label + " difere da modelagem em " + pose + ": " + std::to_string(worst * 1000.0f) + " mm");
        require(std::abs(sole) < 0.003f * s, std::string("a sola no chao do perfil do corpo ") + label);
        // O volume da malha e o da curva de Engine/Physique (medida nela) e,
        // entre os joelhos e nos baixos e medios, o do peso (nos altos, peso / s^2:
        // a forma e a do olho dele, mais encorpada que o peso).
        const float volume = model.mesh.signedVolume() / base->model->mesh.signedVolume();
        const float curve = s * s * s * bodyToneVolume3D(bodyTone3D(body));
        std::cout << "  volume " << volume << " (curva " << curve << ", peso/80 " << body.massKg / 80.0f << ")\n";
        require(std::abs(volume / curve - 1.0f) < 0.003f, std::string("o volume da malha na curva do tonus: ") + label);
        if (bodyTone3D(body) > -2.4f && bodyTone3D(body) < 0.9f && s <= 1.0f)
            require(std::abs(volume / (body.massKg / 80.0f) - 1.0f) < 0.003f, std::string("o volume da malha no peso do corpo ") + label);
        require(std::abs(skeletonHeight - 1.7995f * s) < 0.005f && std::abs(meshHeight - 1.7995f * s) < 0.005f,
            std::string("a altura da modelagem do corpo ") + label);
        // O chao da modelagem fica; o esqueleto e o de hoje em escala em volta dele, junta a junta (as
        // pontas e os auxiliares inclusive), no repouso e num clipe (a raiz dele x s).
        const Vec3 ground { 0.0f, 0.0f, model.skeleton.groundHeight };
        require(ground.z == base->model->skeleton.groundHeight && std::abs(model.mesh.bounds().minimum.z - ground.z) < 0.001f,
            std::string("o chao da modelagem no lugar: ") + label);
        // (O de hoje com o mesmo braco aberto do corpo, sem a escala.)
        GameRig opened = *base->rig;
        opened.bodyArmOpening = rig.bodyArmOpening;
        for (const AnimationClip3D* clip : { static_cast<const AnimationClip3D*>(nullptr), &*sprint }) {
            const SkeletonPose today = sampleGameRigPose(*base->character, opened, clip, 0.15f, RigHandPose::Relaxed, false);
            const SkeletonPose scaled = sampleGameRigPose(skin, rig, clip, 0.15f, RigHandPose::Relaxed, false);
            for (std::size_t j = 0; j < today.joints.size(); ++j)
                require(distance(scaled.joints[j].position, ground + (today.joints[j].position - ground) * s) < 1e-4f
                        && angleBetween(scaled.joints[j].orientation, today.joints[j].orientation) < 2e-3f,
                    std::string("o esqueleto do corpo ") + label + " em escala: " + rig.skeleton.joints[j].name + " ("
                        + std::to_string(distance(scaled.joints[j].position, ground + (today.joints[j].position - ground) * s) * 1000.0f)
                        + " mm, " + std::to_string(angleBetween(scaled.joints[j].orientation, today.joints[j].orientation)) + " rad)");
        }
    }
}

// O jogador levado ao jogo (exportar-jogo): o personagem do laboratorio em
// dia com o modelo (exportar de novo da o mesmo perfil), a pele do jogo
// deformando igual a modelagem nas animacoes (o mesmo esqueleto; o jogo tem a
// pelve na origem), o sombreamento e a sola no chao do perfil.
void testGameCharacterExport() {
    std::string error;
    const std::optional<LoadedPlayerModel> loaded = loadPlayerModel(defaultPlayerModelManifest(), {}, &error);
    require(loaded.has_value(), "modelo do jogador carregado: " + error);
    const std::filesystem::path out = std::filesystem::temp_directory_path()
        / ("matter_jogador_" + std::to_string(std::random_device {}()));
    GameCharacterExportReport report;
    require(exportGameCharacter(*loaded, out, {}, report, error), "exportar o jogador: " + error);
    require(report.bones == 76 && report.triangles == loaded->model->mesh.triangles.size(), "a pele com 76 ossos e os triangulos do modelo");
    const RagdollCharacter3D exported = loadRagdollCharacter3D((out / "character.json").string());
    require(exported.helperBones.size() == 26 && exported.visualBones.size() == 32, "os auxiliares e os dedos na pele");
    require(!exported.flatShaded && std::abs(exported.facet - 0.5f) < 1e-3f, "o sombreamento do jogo: facetado 0,5");

    // O do laboratorio e o de agora: o mesmo rig (senao o modelo mudou e o jogo
    // ficou para tras). Os colisores do perfil fisico instalado sao congelados
    // de proposito (a fisica e a prova do RL ficam iguais; a exportacao instala
    // so a pele, a superficie, o manifesto e o atlas): eles derivam dos pesos, e
    // o desvirar dos corpos variaveis (26/10) mexeu ~5 mm no comprimento do
    // UpperChest. A diferenca sai no log, sem falhar ate 1 cm.
    const RagdollCharacter3D& game = *loaded->character;
    const auto close = [](Vec3 a, Vec3 b) { return distance(a, b) < 2e-4f; };
    require(game.profile.links.size() == exported.profile.links.size(), "os mesmos links");
    for (std::size_t i = 0; i < game.profile.links.size(); ++i) {
        const RagdollLinkDefinition3D& a = game.profile.links[i];
        const RagdollLinkDefinition3D& b = exported.profile.links[i];
        require(close(a.modelPosition, b.modelPosition) && close(a.inboundJoint.anchorModelPosition, b.inboundJoint.anchorModelPosition),
            "o personagem do jogo esta desatualizado (" + a.id + "): MatterModelInspector exportar-jogo --destino assets/characters/football_player");
        const float drift = std::max({ distance(a.collider.localPosition, b.collider.localPosition),
            std::abs(a.collider.radiusMeters - b.collider.radiusMeters), std::abs(a.collider.lengthMeters - b.collider.lengthMeters),
            distance(a.collider.boxHalfExtents, b.collider.boxHalfExtents) });
        if (drift >= 2e-4f) std::cout << "  colisor congelado " << a.id << ": a exportacao difere " << drift * 1000.0f << " mm\n";
        require(drift < 0.01f, "o colisor congelado de " + a.id + " longe demais da exportacao: " + std::to_string(drift * 1000.0f) + " mm");
    }
    require(std::abs(game.profile.standingRootHeightMeters - exported.profile.standingRootHeightMeters) < 2e-4f, "a altura de pe em dia");

    // A pele do jogo contra a malha posada da modelagem (cada vertice do jogo
    // e um da modelagem na mesma posicao da ligacao).
    const GameRig& rig = *loaded->rig;
    const Mesh& mesh = loaded->model->mesh;
    const SkeletonPose rest = sampleGameRigPose(game, rig, nullptr, 0.0f, RigHandPose::Bind, true);
    const Vec3 lift = exported.profile.links.front().modelPosition - rest.joints[static_cast<std::size_t>(rig.skeleton.find("Pelvis"))].position;
    require(std::abs(lift.x) < 1e-4f && std::abs(lift.y) < 1e-4f && std::abs(rig.rootScale - 1.0f) < 1e-3f, "so a altura muda do modelo ao jogo; a raiz sem escala");
    std::multimap<std::tuple<long, long, long>, std::size_t> byPosition;
    const auto key = [](Vec3 p) { return std::make_tuple(std::lround(p.x * 2e4), std::lround(p.y * 2e4), std::lround(p.z * 2e4)); };
    for (std::size_t v = 0; v < mesh.positions.size(); ++v) byPosition.insert({ key(mesh.positions[v] + lift), v });
    std::vector<std::vector<std::size_t>> twins(exported.mesh.vertices.size());
    for (std::size_t g = 0; g < exported.mesh.vertices.size(); ++g) {
        const Vec3 p = exported.mesh.vertices[g].position;
        const auto [x, y, z] = key(p);
        for (long dx = -1; dx <= 1; ++dx)
            for (long dy = -1; dy <= 1; ++dy)
                for (long dz = -1; dz <= 1; ++dz) {
                    const auto range = byPosition.equal_range({ x + dx, y + dy, z + dz });
                    for (auto it = range.first; it != range.second; ++it)
                        if (distance(mesh.positions[it->second] + lift, p) < 2e-5f) twins[g].push_back(it->second);
                }
        require(!twins[g].empty(), "vertice do jogo sem par na modelagem");
    }
    const SkinParity parity = skinParity(exported, game, rig, mesh, twins, lift, 1.0f);
    for (const auto& [name, worst] : parity.worst)
        require(worst < 0.0005f, "a pele do jogo difere da modelagem em " + name + ": " + std::to_string(worst * 1000.0f) + " mm");
    require(std::abs(parity.lowest + exported.profile.standingRootHeightMeters) < 0.003f, "a sola no chao do perfil no repouso");

    // O template de superficie (o sistema de aparencia) chega a pele do jogo: em
    // cada canto de cada triangulo (a mesma ordem), a origem, a regiao, a zona, o
    // UV0 e o UV1 do template; e o template do jogo le igual ao da modelagem.
    const std::shared_ptr<const Appearance::SurfaceTemplate>& surface = loaded->model->surface;
    require(surface != nullptr, "o modelo com o template de superficie");
    require(exported.surface != nullptr && exported.surfaceVertices.size() == exported.mesh.vertices.size(), "a superficie no personagem exportado");
    const auto same = [](Vec2 a, Vec2 b) { return std::abs(a.x - b.x) < 2e-7f && std::abs(a.y - b.y) < 2e-7f; };
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (std::size_t k = 0; k < 3; ++k) {
            const RagdollCharacter3D::SurfaceVertex& vertex = exported.surfaceVertices[exported.mesh.indices[t * 3 + k]];
            require(vertex.source == mesh.triangles[t][k], "a origem do vertice da pele");
            require(vertex.region == surface->triangleRegion[t] && vertex.identity == surface->triangleIdentity[t], "a regiao e a zona na pele");
            if (surface->trianglePanel[t] >= 0) require(same(vertex.kitUv, surface->triangleKitUv[t][k]), "o UV0 na pele");
            if (surface->triangleIdentity[t] >= 0) require(same(vertex.identityUv, surface->triangleIdentityUv[t][k]), "o UV1 na pele");
        }
    require(exported.surface->trianglePanel == surface->trianglePanel && exported.surface->seams.size() == surface->seams.size()
            && exported.surface->storageLayout.texelsPerMeter == surface->storageLayout.texelsPerMeter,
        "o template do jogo igual ao da modelagem");

    // As recusas do carregador: uma pele que nao bate com o template para (o atributo
    // faltando, a classificacao trocada, o UV divergente, fora da faixa, sem o template).
    using Json = nlohmann::json;
    const auto readFile = [](const std::filesystem::path& path) {
        Json value;
        std::ifstream(path) >> value;
        return value;
    };
    const Json manifest = readFile(out / "character.json");
    const std::filesystem::path skinPath = out / manifest.at("skin").get<std::string>();
    const Json skin = readFile(skinPath);
    std::size_t withKit = 0, withIdentity = 0;
    while (!skin["vertices"][withKit].contains("kitUv")) ++withKit;
    while (!skin["vertices"][withIdentity].contains("identity")) ++withIdentity;
    const auto rejected = [&](const char* what, const std::string& fragment, const auto& change, bool inManifest = false) {
        Json broken = inManifest ? manifest : skin;
        change(broken);
        std::ofstream(inManifest ? out / "character.json" : skinPath) << broken;
        std::string message;
        try {
            (void)loadRagdollCharacter3D((out / "character.json").string());
        } catch (const std::exception& failure) {
            message = failure.what();
        }
        std::ofstream(inManifest ? out / "character.json" : skinPath) << (inManifest ? manifest : skin);
        require(!message.empty(), std::string("o carregador aceitou ") + what);
        require(message.find(fragment) != std::string::npos, std::string(what) + ": " + message);
    };
    rejected("o UV0 faltando", "disagrees", [&](Json& d) { d["vertices"][withKit].erase("kitUv"); });
    rejected("a regiao faltando", "region", [&](Json& d) { d["vertices"][withKit].erase("region"); });
    rejected("a regiao trocada", "disagrees", [&](Json& d) {
        d["vertices"][withKit]["region"] = (d["vertices"][withKit]["region"].get<int>() + 1) % static_cast<int>(surface->regions.size());
    });
    rejected("uma zona onde nao ha", "disagrees", [&](Json& d) {
        d["vertices"][withKit]["identity"] = 0;
        d["vertices"][withKit]["identityUv"] = Json::array({ 0.5, 0.5 });
    });
    rejected("o UV0 divergente", "disagrees", [&](Json& d) {
        d["vertices"][withKit]["kitUv"][0] = d["vertices"][withKit]["kitUv"][0].get<double>() + 1e-3;
    });
    rejected("o UV1 divergente", "disagrees", [&](Json& d) {
        d["vertices"][withIdentity]["identityUv"][1] = d["vertices"][withIdentity]["identityUv"][1].get<double>() + 1e-3;
    });
    rejected("a regiao fora da faixa", "Invalid skin surface region", [&](Json& d) { d["vertices"][withKit]["region"] = 70000; });
    rejected("o UV0 fora do atlas", "Invalid skin surface kitUv", [&](Json& d) { d["vertices"][withKit]["kitUv"][0] = 1.5; });
    rejected("a zona sem o UV1", "without its UV", [&](Json& d) { d["vertices"][withIdentity].erase("identityUv"); });
    rejected("a superficie sem o template", "without a surface template", [&](Json& d) { d.erase("surface"); }, true);
    (void)loadRagdollCharacter3D((out / "character.json").string());
    std::filesystem::remove_all(out);
}

using TestSupport::FakeCanvas;
using TestSupport::snapshot;

void frame(Workbench::PlayerInspectorScreen& screen, FakeCanvas& canvas) {
    ImGuiIO& io = ImGui::GetIO();
    io.DeltaTime = 1.0f / 60.0f;
    ImGui::NewFrame();
    screen.update(io.DeltaTime);
    screen.draw(canvas, 1.0f);
    ImGui::Render();
}

// O molde da cabeca de referencia: o alinhamento pelo cranio e o rosto (erro
// de poucos mm), a cabeca recuada sobre o pescoco e os vertices do queixo, da
// mandibula e do pescoco que vao inteiros para o molde em cima da superficie
// dele - igual a referencia, medido.
void testReferenceFit() {
    const std::filesystem::path recipePath = defaultPlayerModelManifest().parent_path() / "montagem.json";
    std::ifstream file(recipePath);
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error);
    require(recipe.has_value(), "a receita: " + error);
    require(!recipe->referenceFit.file.empty() && recipe->headShift < 0.0f, "a receita usa o molde e recua a cabeca");
    const std::optional<LoadedPlayerModel> loaded = loadPlayerModel(defaultPlayerModelManifest(), { "nenhuma" }, &error);
    require(loaded.has_value(), "o esqueleto: " + error);
    const AssemblyResult assembled = assemblePlayer(*recipe, recipePath.parent_path(), loaded->rig->skeleton);
    require(assembled.ok, "a montagem: " + assembled.error);
    const ReferenceFitReport& fit = assembled.referenceFit;
    require(fit.alignmentPoints >= 40 && fit.alignmentRms < 0.004f, "o alinhamento pelo cranio e o rosto (rms " + std::to_string(fit.alignmentRms) + ")");
    require(std::abs(fit.pitchDegrees) < 10.0f && fit.scale > 0.005f, "a referencia quase de pe");
    GltfImportSettings settings;
    settings.weldTolerance = 1e-5f;
    const GltfImportResult reference = importGltfMesh(recipePath.parent_path() / recipe->referenceFit.file, settings);
    require(reference.ok, "a referencia: " + reference.error);
    std::vector<Vec3> placed;
    for (const Vec3& p : reference.mesh.positions) placed.push_back(fit.transform.apply(p));
    const TriangleBvh surface(placed, reference.mesh.triangles);
    std::size_t whole = 0;
    for (const ReferenceFitReport::Move& move : fit.moves) {
        if (move.weight < 0.999f) continue;
        const std::optional<TriangleBvh::Nearest> nearest = surface.nearest(move.to, 0.01f);
        require(nearest && nearest->distance < 5e-4f, "um vertice inteiro no molde fora da superficie");
        ++whole;
    }
    require(whole >= 12, "o queixo, a mandibula e o pescoco no molde: " + std::to_string(whole) + " vertices inteiros");
    // A junta da cabeca anda com ela (o pivo no lugar).
    const Skeleton& skeleton = loaded->rig->skeleton;
    const Vec3 head = skeleton.joints[static_cast<std::size_t>(skeleton.find("Head"))].bind.position;
    require(std::abs(head.x - (-0.025f + recipe->headShift)) < 1e-4f, "a junta da cabeca recuada junto");
}

// O exemplo da pintura (exemplos/pintura_exemplo.json: o uniforme mais um
// escudo em SVG e um patrocinio em PNG) continua valendo nos espacos medidos
// no modelo de agora: as ancoras, os arquivos ao lado dele e o lugar do
// escudo (o peito esquerdo, abaixo da gola).
void testPaintExample() {
    const std::filesystem::path directory = defaultPlayerModelManifest().parent_path();
    std::ifstream file(directory / "montagem.json");
    std::stringstream text;
    text << file.rdbuf();
    std::string error;
    const std::optional<AssemblyRecipe> recipe = parseAssemblyRecipe(text.str(), &error);
    require(recipe.has_value(), "a receita: " + error);
    // A montagem (a malha do jogo vem do Blender desde 22/10; a montagem gera
    // a troca e a pintura).
    PlayerModelOptions assembled;
    assembled.meshSource = "montagem";
    const std::optional<LoadedPlayerModel> loaded = loadPlayerModel(defaultPlayerModelManifest(), assembled, &error);
    require(loaded.has_value(), "o modelo: " + error);
    // A carga diz o que leu (a tela recarrega quando a pintura muda) e quanto
    // levou cada etapa (o bake em paralelo: a montagem inteira em ~1 s).
    const auto read = [&](const std::string& name) {
        return std::any_of(loaded->sourceFiles.begin(), loaded->sourceFiles.end(), [&](const auto& path) { return path.filename() == name; });
    };
    require(read("pintura.json") && read("corpo_base.glb") && read("chuteira.json"), "a carga lista os arquivos lidos");
    double total = 0.0;
    bool baked = false;
    for (const auto& [stage, ms] : loaded->timings) {
        total += ms;
        baked = baked || stage == "pintura (bake)";
    }
    require(baked && total < 5000.0, "as etapas da carga com tempo (" + std::to_string(static_cast<int>(total)) + " ms)");
    const auto spaces = playerPaintSpaces(loaded->model->mesh, *recipe, loaded->model->skeleton);
    const std::optional<PaintDesign> uniform = loadPaintDesign(directory / recipe->paintFile, spaces, &error);
    const std::optional<PaintDesign> example = loadPaintDesign(directory / "exemplos" / "pintura_exemplo.json", spaces, &error);
    require(uniform && example, "o exemplo da pintura: " + error);
    require(example->layers.size() == uniform->layers.size() + 4, "o uniforme, as tres cores do escudo e o patrocinio");
    const float collar = spaces.at("frente").anchors.at("gola_frente");
    for (const PaintLayer& layer : example->layers) {
        if (layer.name.rfind("escudo", 0) != 0) continue;
        const Box2& box = layer.shape.bounds();
        require(layer.sides == Sides::Left && box.minimum.x > 0.04f && box.maximum.x < 0.13f && box.maximum.y < collar && box.minimum.y > collar - 0.13f,
            "o escudo no peito esquerdo: " + layer.name);
    }
}

void testScreenWithoutWindow() {
    TestSupport::HeadlessUi context;
    ImGuiIO& io = ImGui::GetIO();

    Workbench::PlayerInspectorScreen::Options options;
    options.manifest = defaultPlayerModelManifest();
    options.capturesDirectory = std::filesystem::temp_directory_path() / "matter_inspector_test";
    bool wentBack = false;
    Workbench::PlayerInspectorScreen screen(options, [&] { wentBack = true; });
    FakeCanvas canvas;
    frame(screen, canvas);
    // A montagem roda em segundo plano: o primeiro quadro sai sem esperar.
    require(screen.loading() || screen.loaded(), "a tela comeca a carregar no primeiro quadro");
    frame(screen, canvas);
    screen.waitUntilLoaded();
    require(screen.loaded() && screen.error().empty(), "a tela carrega o modelo: " + screen.error());
    {
        // O corpo (a secao "CORPO"): 2,00 m remonta o modelo x 2,00/1,80 (a
        // cabeca sobre o pe: a escala e em volta do chao).
        const auto headZ = [&] {
            const InspectionModel& model = *screen.session().model();
            const auto z = [&](const char* id) {
                return model.skeleton.joints[static_cast<std::size_t>(model.skeleton.find(id))].bind.position.z;
            };
            return z("Head") - z("LeftFoot");
        };
        const float base = headZ();
        screen.setBody(200.0f, 100.0f);
        screen.waitUntilLoaded();
        require(screen.error().empty() && std::abs(headZ() / base - 2.0f / 1.8f) < 0.01f,
            "a tela remonta o modelo no corpo de 2,00 m: " + screen.error());
        screen.setBody(180.0f, 80.0f);
        screen.waitUntilLoaded();
        require(std::abs(headZ() - base) < 1e-6f, "e volta ao corpo de hoje");
    }
    for (int i = 0; i < 30; ++i) frame(screen, canvas);
    snapshot(canvas, "tela_visao_geral.png");
    require(canvas.width > 800 && canvas.height > 500, "o viewport recebe a imagem no tamanho dele");
    const int uploadsBefore = canvas.uploads;
    frame(screen, canvas);
    require(canvas.uploads == uploadsBefore, "sem mudanca, a imagem nao e reenviada");

    // Arrastar no meio do viewport gira a camera.
    const float yaw = screen.session().camera().yaw;
    io.AddMousePosEvent(1200.0f, 600.0f);
    frame(screen, canvas);
    io.AddMouseButtonEvent(0, true);
    frame(screen, canvas);
    for (int i = 1; i <= 6; ++i) {
        io.AddMousePosEvent(1200.0f + 20.0f * static_cast<float>(i), 600.0f);
        frame(screen, canvas);
    }
    io.AddMouseButtonEvent(0, false);
    frame(screen, canvas);
    require(std::abs(screen.session().camera().yaw - yaw) > 0.2f, "arrastar no viewport gira a camera");
    require(canvas.uploads > 1, "girar reenvia a imagem");

    // Cada parte do corpo, com transicao.
    for (int region = 0; region < 18; ++region) {
        screen.selectRegion(region);
        for (int i = 0; i < 20; ++i) frame(screen, canvas);
        require(screen.session().selectedRegion() == region, "parte escolhida");
    }
    for (int i = 0; i < 40; ++i) frame(screen, canvas);
    require(!screen.session().animating(), "transicao termina");

    screen.selectRegion(screen.session().model()->regions.find("pe_direito"));
    screen.editableSession().editStyle().skeleton = false;
    screen.editableSession().editStyle().isolate = true;
    for (int i = 0; i < 60; ++i) frame(screen, canvas);
    snapshot(canvas, "tela_pe_direito.png");
    screen.editableSession().editStyle().skeleton = true;
    screen.editableSession().editStyle().isolate = false;

    // Fotos: a mao direita com os nomes, a malha antiga em raio-x e a
    // referencia de frente com os ossos por cima.
    screen.selectRegion(8);
    screen.editableSession().editStyle().jointLabels = true;
    for (int i = 0; i < 60; ++i) frame(screen, canvas);
    snapshot(canvas, "tela_mao_direita.png");
    // As dobras das maos: o punho do jogo (a malha deforma).
    screen.setBendGroup(3);
    screen.setHandPose(3);
    for (int i = 0; i < 10; ++i) frame(screen, canvas);
    require(screen.session().posed(), "o punho deforma a malha");
    snapshot(canvas, "tela_mao_punho.png");
    screen.setHandPose(0);
    for (int i = 0; i < 5; ++i) frame(screen, canvas);
    require(!screen.session().posed(), "a mao reta volta a ligacao");
    // A cabeca (a regiao 1): girar e inclinar deformam o rosto e o pescoco.
    screen.selectRegion(1);
    screen.editableSession().editStyle().jointLabels = false;
    screen.setBendGroup(2);
    screen.setBend(25, 35.0f);
    screen.setBend(26, 20.0f);
    for (int i = 0; i < 10; ++i) frame(screen, canvas);
    require(screen.session().posed(), "a cabeca dobrada deforma a malha");
    snapshot(canvas, "tela_cabeca_girada.png");
    screen.setBend(25, 0.0f);
    screen.setBend(26, 0.0f);
    for (int i = 0; i < 5; ++i) frame(screen, canvas);
    require(!screen.session().posed(), "a cabeca reta volta a ligacao");
    screen.setMeshSource("nenhuma");
    screen.waitUntilLoaded();
    for (int i = 0; i < 10; ++i) frame(screen, canvas);
    require(screen.loaded() && screen.session().model()->mesh.empty(), "a tela troca para so os ossos");
    screen.setMeshSource("montagem");
    screen.waitUntilLoaded();
    screen.selectRegion(3);
    screen.editableSession().editStyle().jointLabels = false;
    screen.editableSession().editStyle().surface = SurfaceMode::XRay;
    for (int i = 0; i < 60; ++i) frame(screen, canvas);
    require(screen.loaded() && !screen.session().model()->mesh.empty(), "a tela volta ao modelo montado");
    snapshot(canvas, "tela_peito_raiox.png");
    screen.setMeshSource("nenhuma");
    screen.waitUntilLoaded();
    screen.setReference(0);
    for (int i = 0; i < 10; ++i) frame(screen, canvas);
    snapshot(canvas, "tela_referencia_frente.png");
    screen.setReference(-1);
    for (int i = 0; i < 5; ++i) frame(screen, canvas);

    // A aba das animacoes: os clipes do personagem, o tocador embaixo e a
    // pose do jogo no modelo montado.
    screen.setMeshSource("montagem");
    screen.waitUntilLoaded();
    screen.editableSession().editStyle().surface = SurfaceMode::Solid;
    screen.editableSession().editStyle().skeleton = false;
    screen.setTab(1);
    screen.animations().waitUntilLoaded();
    const auto& entries = screen.animations().entries();
    require(screen.tab() == 1 && screen.animations().loaded(), "as animacoes carregam");
    require(entries.size() >= 20, "os clipes do personagem na lista (" + std::to_string(entries.size()) + ")");
    require(screen.animations().error().empty(), "todos os clipes carregaram: " + screen.animations().error());
    for (const auto& entry : entries) require(!entry.label.empty() && entry.label.find('(') == std::string::npos
            || entry.label == "Sprint de costas (físico)", "nome limpo: " + entry.label);
    for (int i = 0; i < 40; ++i) frame(screen, canvas);
    require(screen.session().posed(), "a animacao poe a pose");
    require(screen.session().selectedRegion() == 0, "a animacao mostra o corpo inteiro");
    snapshot(canvas, "tela_animacoes.png");
    std::size_t sprint = entries.size();
    for (std::size_t i = 0; i < entries.size(); ++i)
        if (entries[i].label == "Sprint") sprint = i;
    require(sprint < entries.size(), "o sprint na lista");
    screen.animations().select(sprint);
    const auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < 30; ++i) frame(screen, canvas);
    const double perFrame = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - started).count() / 30.0;
    std::cout << "animacao tocando: " << perFrame << " ms por quadro (" << canvas.width << " x " << canvas.height << ")\n";
    require(screen.animations().playing() && screen.animations().time() > 0.0f, "o sprint toca");
    snapshot(canvas, "tela_animacao_sprint.png");
    screen.animations().setPlaying(false);
    const float paused = screen.animations().time();
    for (int i = 0; i < 10; ++i) frame(screen, canvas);
    require(std::abs(screen.animations().time() - paused) < 1e-6f, "pausado, o instante fica");
    screen.animations().seek(0.1f);
    for (int i = 0; i < 3; ++i) frame(screen, canvas);
    require(std::abs(screen.animations().time() - 0.1f) < 1e-6f, "a linha do tempo leva ao instante");
    // Um clipe que nao repete para no fim, no ultimo quadro.
    for (std::size_t i = 0; i < entries.size(); ++i)
        if (entries[i].label == "Pulo parado") screen.animations().select(i);
    screen.animations().setSpeed(2.0f);
    for (int i = 0; i < 60; ++i) frame(screen, canvas);
    require(!screen.animations().playing() && std::abs(screen.animations().time() - screen.animations().duration()) < 1e-4f,
        "sem repetir, para no fim");
    screen.animations().setSpeed(1.0f);
    // De volta as partes do corpo: sem dobras, a pose de ligacao.
    screen.setTab(0);
    for (int i = 0; i < 5; ++i) frame(screen, canvas);
    require(screen.tab() == 0 && !screen.session().posed(), "as partes voltam a ligacao");
    snapshot(canvas, "tela_partes.png");
    require(!wentBack, "nao voltou ao menu sozinho");
    require(context.errors().empty(), "ImGui reclamou: " + (context.errors().empty() ? std::string() : context.errors().front()));
}

} // namespace

int main() {
    try {
        // (A tela primeiro: o avesso antigo do calcao em testPlayerModel nao a esconde.)
        testScreenWithoutWindow();
        testGameCharacterExport();
        testBodyVariants();
        testPlayerModel();
        testReferenceFit();
        testPaintExample();
    } catch (const std::exception& exception) {
        std::cerr << "MatterPlayerModel test failure: " << exception.what() << '\n';
        return 1;
    }
    std::cout << "MatterPlayerModel tests passed (" << checks << " checagens)\n";
    return 0;
}
