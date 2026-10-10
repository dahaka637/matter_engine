#include "Engine/Modeling/Assembly/Uniform.hpp"

#include "Engine/Modeling/Assembly/Garment.hpp"
#include "Engine/Modeling/Assembly/MeshEdit.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshRepair.hpp"
#include "Engine/Modeling/Mesh/MeshSubdivision.hpp"
#include "Engine/Modeling/Skin/SkinSmoothing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace MatterEngine::Modeling {

namespace {

float smoothstep(float edge0, float edge1, float x) {
    const float t = std::clamp((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

Vec3 jointAt(const Skeleton& skeleton, const char* name) {
    const int joint = skeleton.find(name);
    return joint >= 0 ? skeleton.joints[static_cast<std::size_t>(joint)].bind.position : Vec3 {};
}

// O quanto a junta puxa o vertice (o peso dela no skin).
float jointWeight(const Mesh& mesh, std::uint32_t v, int joint) {
    if (v >= mesh.skin.size() || joint < 0) return 0.0f;
    float weight = 0.0f;
    for (std::size_t i = 0; i < 4; ++i)
        if (mesh.skin[v].joints[i] == joint) weight += mesh.skin[v].weights[i];
    return weight;
}

} // namespace

float UniformLines::collarHeight(Vec3 point) const {
    const float angle = std::atan2(std::abs(point.y), point.x - neckX);
    const float height = angle <= collarOpening ? collarDepth + (collarSide - collarDepth) * angle / collarOpening
                                                : collarSide + (collarBack - collarSide) * smoothstep(collarOpening, 3.14159265f, angle);
    return ground + neck.liftedHeight(point.x, height);
}

UniformLines uniformLines(const AssemblyRecipe& recipe, const Skeleton& skeleton) {
    UniformLines lines;
    lines.ground = skeleton.groundHeight;
    const Vec3 shoulder = jointAt(skeleton, "RightUpperArm"), elbow = jointAt(skeleton, "RightForearm");
    const Vec3 hip = jointAt(skeleton, "RightThigh"), knee = jointAt(skeleton, "RightShin");
    lines.neckX = jointAt(skeleton, "Neck").x;
    lines.neck = recipe.neck;
    lines.collarDepth = recipe.collarDepth;
    lines.collarSide = recipe.collarSide;
    lines.collarBack = recipe.collarBack;
    lines.collarOpening = recipe.collarOpening * 3.14159265f / 180.0f;
    lines.armAlong = safeNormalized(elbow - shoulder, { 0.0f, -1.0f, 0.0f });
    lines.legAlong = safeNormalized(knee - hip, { 0.0f, 0.0f, -1.0f });
    lines.sleevePoint = shoulder + (elbow - shoulder) * recipe.sleeveAt;
    lines.legHemPoint = hip + lines.legAlong * ((hip.z - (lines.ground + lines.lifted(recipe.shortsHem))) / std::max(1e-3f, -lines.legAlong.z));
    lines.shirtHem = lines.ground + lines.lifted(recipe.shirtHem);
    lines.shortsTop = lines.ground + lines.lifted(recipe.shortsTop);
    return lines;
}

bool buildUniform(Mesh& mesh, const AssemblyRecipe& recipe, const Skeleton& skeleton, std::string& error) {
    const UniformLines lines = uniformLines(recipe, skeleton);
    const float ground = lines.ground;
    const Vec3 hip = jointAt(skeleton, "RightThigh"), knee = jointAt(skeleton, "RightShin");
    const Vec3 chest = jointAt(skeleton, "UpperChest"), neck = jointAt(skeleton, "Neck");
    const Vec3 hand = jointAt(skeleton, "RightHand"), headTop = jointAt(skeleton, "HeadTop");
    const Vec3 armAlong = lines.armAlong, legAlong = lines.legAlong;
    const Vec3 sleeve = lines.sleevePoint, legHem = lines.legHemPoint;
    const Vec3 torso { -0.01f, -0.09f, 0.0f };
    const auto collarHeight = [&](Vec3 p) { return lines.collarHeight(p); };
    const float neckRadius = 0.16f, armRadius = 0.085f, legRadius = 0.13f, torsoRadius = 0.22f;
    const CutSettings settings { 0.006f };

    // ---- A camisa: alem da barra, da manga e da gola sai ----
    Mesh shirt = mesh;
    const RegionCut shirtCuts[] {
        { planeField({ { torso.x, torso.y, lines.shirtHem }, { 0.0f, 0.0f, 1.0f } }), { torso.x, torso.y, lines.shirtHem },
            torsoRadius, knee },
        { planeField({ sleeve, -armAlong }), sleeve, armRadius, hand },
        { [&](Vec3 p) { return collarHeight(p) - p.z; }, neck, neckRadius, headTop } };
    RegionResult shirtRegion = keepRegion(shirt, shirtCuts, settings);
    if (!shirtRegion.ok) {
        error = "camisa: " + shirtRegion.error;
        return false;
    }
    // Na axila (onde o braco e o tronco dividem o vertice) os pesos da camisa
    // se espalham: o pano estica por cima da dobra em vez de virar.
    {
        const int arm = skeleton.find("RightUpperArm");
        std::vector<bool> transition(shirt.positions.size(), false);
        for (std::size_t v = 0; v < shirt.positions.size(); ++v) {
            const float w = jointWeight(shirt, static_cast<std::uint32_t>(v), arm);
            transition[v] = w > 0.03f && w < 0.97f;
        }
        // Mais uma volta de vizinhos em volta da passagem.
        std::vector<bool> around = transition;
        for (const Triangle& triangle : shirt.triangles) {
            bool touches = false;
            for (const std::uint32_t v : triangle) touches = touches || transition[v];
            if (touches)
                for (const std::uint32_t v : triangle) around[v] = true;
        }
        // Longe da barra da manga (ali o pano anda com o braco de dentro, os
        // mesmos pesos da pele) e acima da cintura.
        smoothSkinWeights(shirt, recipe.armpitWeightSmoothing, 0.5f, [&](std::uint32_t v) {
            const Vec3 p = shirt.positions[v];
            return around[v] && dot(p - lines.sleevePoint, lines.armAlong) < -0.03f && p.z > ground + lines.lifted(1.1f);
        });
    }
    // ---- O calcao: acima do alto e alem da barra da perna sai ----
    Mesh shorts = mesh;
    const RegionCut shortsCuts[] {
        { planeField({ { torso.x, torso.y, lines.shortsTop }, { 0.0f, 0.0f, -1.0f } }), { torso.x, torso.y, lines.shortsTop }, torsoRadius,
            chest },
        { planeField({ legHem, -legAlong }), legHem, legRadius, knee } };
    RegionResult shortsRegion = keepRegion(shorts, shortsCuts, settings);
    if (!shortsRegion.ok) {
        error = "calcao: " + shortsRegion.error;
        return false;
    }
    // ---- A pele coberta sai: fica a cabeca (acima da gola, por dentro), o
    // antebraco (por dentro da manga) e a perna (por dentro do calcao) ----
    const Mesh body = mesh;
    const RegionCut skinCuts[] { { [&](Vec3 p) { return p.z - (collarHeight(p) - recipe.recess); }, neck, neckRadius, headTop },
        { planeField({ sleeve - armAlong * recipe.recess, armAlong }), sleeve - armAlong * recipe.recess, armRadius, hand },
        { planeField({ legHem - legAlong * recipe.recess, legAlong }), legHem - legAlong * recipe.recess, legRadius, knee } };
    const RegionResult skin = removeCovered(mesh, skinCuts, chest, settings);
    if (!skin.ok) {
        error = "pele debaixo do uniforme: " + skin.error;
        return false;
    }

    // ---- O pano: inflado ate a folga, por cima dos vincos ----
    const int upperArm = skeleton.find("RightUpperArm");
    ClothShape shape;
    shape.relax = recipe.clothRelax;
    shape.steps = recipe.clothSteps;
    shape.smoothing = recipe.clothSmoothing;
    // A camisa cai do peito e das costas (em volta do eixo do tronco; a manga
    // nao conta); o calcao cai do quadril e do gluteo (em volta da perna).
    ClothShape shirtShape = shape;
    // Do ombro para cima o pano fica com a forma do corpo (shoulderRelax: o
    // deltoide, o trapezio); a axila, abaixo, e a nuca (atras da gola, perto
    // do meio: o pescoco mexe ali) arredondam inteiras.
    const Vec3 shoulderJoint = jointAt(skeleton, "RightUpperArm");
    shirtShape.relaxWeight = [&](std::uint32_t v) {
        const Vec3 p = shirt.positions[v];
        const float top = smoothstep(shoulderJoint.z - 0.06f, shoulderJoint.z - 0.01f, p.z);
        const float nape = smoothstep(0.07f, 0.03f, std::abs(p.y)) * smoothstep(neck.x - 0.02f, neck.x - 0.07f, p.x);
        return 1.0f - (1.0f - recipe.shoulderRelax) * top * (1.0f - nape);
    };
    // A axila: o lado de dentro do braco (para o tronco) perto do ombro, ate
    // o flanco embaixo dele - na metade direita; o espelho anda junto.
    const Vec3 inward = safeNormalized(Vec3 { 0.0f, 1.0f, 0.0f } - armAlong * armAlong.y, { 0.0f, 1.0f, 0.0f });
    const auto armpit = [&](Vec3 p) {
        const Vec3 q { p.x - shoulderJoint.x, -std::abs(p.y) - shoulderJoint.y, p.z - shoulderJoint.z };
        const float along = dot(q, armAlong);
        const Vec3 radial = q - armAlong * along;
        const float side = dot(safeNormalized(radial, {}), inward);
        return smoothstep(0.1f, 0.5f, side) * smoothstep(-0.05f, 0.0f, along) * smoothstep(0.24f, 0.16f, along)
            * smoothstep(0.15f, 0.1f, radial.length());
    };
    shirtShape.hangs.push_back({ { chest.x, 0.0f, chest.z }, { 0.0f, 0.0f, 1.0f }, recipe.clothHang, 0.3f, 18.0f,
        [&](std::uint32_t v) { return jointWeight(shirt, v, upperArm) < 0.3f; } });
    ClothShape shortsShape = shape;
    shortsShape.hangs.push_back({ knee, hip - knee, recipe.shortsHang, 0.3f, 18.0f, {} });
    inflateShell(
        shirt, body,
        [&](std::uint32_t v) {
            const Vec3 p = shirt.positions[v];
            float loose = recipe.shirtLoose + (recipe.shirtLooseHem - recipe.shirtLoose) * smoothstep(ground + lines.lifted(1.3f), lines.shirtHem, p.z);
            // Na manga a folga dela, passando macio pela axila (o peso do braco).
            loose += (recipe.sleeveLoose - loose) * smoothstep(0.2f, 0.8f, jointWeight(shirt, v, upperArm));
            // Na axila, a dela (se maior).
            if (recipe.armpitLoose > loose) loose += (recipe.armpitLoose - loose) * armpit(p);
            return recipe.collarLoose + (loose - recipe.collarLoose) * smoothstep(0.0f, 0.035f, std::abs(collarHeight(p) - p.z));
        },
        shirtShape);
    inflateShell(
        shorts, body,
        [&](std::uint32_t v) {
            const float h = shorts.positions[v].z - ground;
            const float hem = lines.shirtHem - ground, hips = lines.lifted(recipe.shirtHem - 0.12f), legHem = lines.lifted(recipe.shortsHem);
            return recipe.shortsLooseTop + (recipe.shortsLoose - recipe.shortsLooseTop) * smoothstep(hem - 0.02f, hips, h)
                + (recipe.shortsLooseHem - recipe.shortsLoose) * std::clamp((hips - h) / std::max(0.01f, hips - legHem), 0.0f, 1.0f);
        },
        shortsShape);

    // Os triangulos do pano melhores (sem lascas compridas: o skin vira lasca
    // do avesso): as arestas curtas demais para a vizinhanca se juntam (a
    // borda dos cortes e o meio ficam) e as diagonais trocam.
    for (Mesh* piece : { &shirt, &shorts }) {
        std::map<std::pair<std::uint32_t, std::uint32_t>, int> uses;
        for (const Triangle& triangle : piece->triangles)
            for (int i = 0; i < 3; ++i) {
                const std::uint32_t a = triangle[static_cast<std::size_t>(i)], b = triangle[static_cast<std::size_t>((i + 1) % 3)];
                ++uses[{ std::min(a, b), std::max(a, b) }];
            }
        std::vector<bool> rim(piece->positions.size(), false);
        for (const auto& [edge, count] : uses)
            if (count == 1) rim[edge.first] = rim[edge.second] = true;
        collapseShortEdges(*piece, recipe.clothShortEdge, [&](std::uint32_t v) { return rim[v]; });
        // No gancho do calcao, a faixa colada na costura do meio vai para a
        // costura (as agulhas compridas ao longo do meio somem).
        if (piece == &shorts && recipe.crotchBand > 0.0f)
            collapseInto(
                *piece,
                [&](std::uint32_t v) {
                    const Vec3 p = piece->positions[v];
                    return !rim[v] && std::abs(p.y) > 1e-6f && std::abs(p.y) < recipe.crotchBand && p.z < hip.z - 0.02f;
                },
                [&](std::uint32_t v) { return std::abs(piece->positions[v].y) < 1e-6f; });
        improveTriangles(*piece);
    }

    // Mais aneis no calcao pronto (subdivideTriangles): a forma parada fica a
    // mesma e as bordas dos cortes ganham os meios para as costuras; os
    // pesos da faixa e do gancho, abaixo, ja saem nos vertices novos (antes
    // de inflar, o pano denso cruzava ele mesmo no gancho). O gancho fica
    // grosso (shortsCoarseCrotch: o centro do triangulo perto do meio e
    // abaixo do quadril) e o alto do calcao tambem (4 cm, por baixo da barra
    // da camisa): a costura com a camisa fica de igual para igual, sem
    // agulhas.
    if (recipe.shortsSubdivision > 0) {
        std::vector<CutBorder*> borders;
        for (CutBorder& border : shortsRegion.borders) borders.push_back(&border);
        const auto refine = [&](std::size_t t) {
            const Vec3 c = shorts.center(t);
            return !(std::abs(c.y) < recipe.shortsCoarseCrotch && c.z < hip.z - 0.02f) && c.z < lines.shortsTop - recipe.shortsCoarseTop;
        };
        subdivideTriangles(shorts, recipe.shortsSubdivision, borders, refine);
    }
    // O quadril (hipCrease): a passagem da pelve para a coxa numa volta em
    // torno da coxa, com o centro e a largura mudando em volta dela; fora da
    // passagem so a pelve ou so a coxa (a perna do calcao e rigida ate a
    // dobra, sem banana nem arco). Na frente a volta segue a linha da
    // virilha desenhada pelo usuario (de frente, do lado de fora ao gancho),
    // estreita: a coxa levantada entra na barriga num vinco fechado (larga,
    // o canto virava um arco de ~10 cm em volta da junta). Do lado de fora,
    // na altura da junta: a listra faz o arco curto em volta dela (no plano
    // da linha ela subia ate a barra da camisa e beliscava o canto). Atras,
    // na dobra do gluteo, estreita e direta da pelve para a coxa: o gluteo
    // fica redondo na pelve e o pano estica reto ate a coxa (girando em
    // volta da junta, o volume do gluteo descia junto e virava um saco
    // embaixo da perna levantada). Do lado de dentro, na altura do gancho.
    // A passagem, fora de tras: pelve -> 1/4 -> 1/2 -> 3/4 -> coxa, cada anel
    // girando inteiro em volta da junta (a media dos pesos nao puxa o pano
    // para dentro). Nos vertices finais do pano: o peso e o do lugar onde o
    // vertice ficou.
    //
    // A dobra da frente (hipLift*): acima da linha, a cintura do calcao e a
    // barra da camisa vao para a auxiliar HipLift (uma fracao so da flexao do
    // quadril): a coxa que sobe dobra o pano para tras e a barra fica em cima
    // da perna (parada, a perna rigida subia ~5 cm atravessando a barra); a
    // coxa que abre de lado ou torce nao mexe nelas. Os dois com o mesmo peso
    // no mesmo lugar: andam juntos, um nao atravessa o outro.
    if (recipe.hipCreaseOuter[1] > 0.0f) {
        // A distancia ao longo da coxa (abaixo da junta) de um ponto numa
        // altura do chao.
        const auto alongAt = [&](float height) { return (hip.z - (ground + lines.lifted(height))) / std::max(1e-3f, -legAlong.z); };
        const float outerY = recipe.hipCreaseOuter[0], innerY = recipe.hipCreaseInner[0];
        const float outerAlong = alongAt(recipe.hipCreaseOuter[1]), innerAlong = alongAt(recipe.hipCreaseInner[1]);
        // A frente, o lado de fora e o de dentro em volta da coxa.
        const Vec3 forward = safeNormalized(Vec3 { 1.0f, 0.0f, 0.0f } - legAlong * legAlong.x, { 1.0f, 0.0f, 0.0f });
        const Vec3 outward = safeNormalized(Vec3 { 0.0f, -1.0f, 0.0f } - legAlong * -legAlong.y, { 0.0f, -1.0f, 0.0f });
        const auto aroundThigh = [&](Vec3 p) {
            const Vec3 offset = p - hip;
            return safeNormalized(offset - legAlong * dot(offset, legAlong), forward);
        };
        // Perto do meio (o gancho) a frente passa para a de dentro: estreita
        // ali, o agachamento fundo virava o pano do gancho.
        constexpr float MidlineFrom = 0.03f, MidlineTo = 0.08f;
        struct Crease {
            float center;
            float width;
            // A fracao direta (sem os aneis): atras.
            float straight;
        };
        // O centro e a largura no ponto: a mistura das quatro direcoes em
        // volta da coxa (a parte positiva de cada uma, ao quadrado: somam 1).
        const auto creaseAt = [&](Vec3 p) {
            const Vec3 radial = aroundThigh(p);
            const float f = dot(radial, forward), l = dot(radial, outward);
            const float front = (f > 0.0f ? f * f : 0.0f) * smoothstep(MidlineFrom, MidlineTo, std::abs(p.y)), back = f < 0.0f ? f * f : 0.0f;
            const float side = l > 0.0f ? l * l : 0.0f, inside = l < 0.0f ? l * l : 0.0f;
            const float total = std::max(1e-6f, front + back + side + inside);
            // Na frente, a linha desenhada na altura do ponto (de lado); do
            // lado de fora, a junta (0).
            const float u = std::clamp((p.y - outerY) / (innerY - outerY), 0.0f, 1.0f);
            const float frontCenter = outerAlong + (innerAlong - outerAlong) * u;
            const float center = (front * frontCenter + back * recipe.hipCreaseBackCenter + inside * innerAlong) / total;
            const float width = (front * recipe.hipCreaseFront + side * recipe.hipCreaseSide + back * recipe.hipCreaseBack
                                    + inside * recipe.hipCreaseInside) / total;
            return Crease { center, width, back / total };
        };
        // Quanto do peso da pelve vai para a HipLift (a dobra da frente).
        const int liftJoint = skeleton.find("RightHipLift");
        const bool lifting = recipe.hipLiftRise[1] > recipe.hipLiftRise[0] && liftJoint >= 0;
        const auto liftAt = [&](Vec3 p) {
            if (!lifting) return 0.0f;
            return smoothstep(recipe.hipLiftFront[0], recipe.hipLiftFront[1], dot(aroundThigh(p), forward))
                * smoothstep(lines.shirtHem + recipe.hipLiftRise[1], lines.shirtHem + recipe.hipLiftRise[0], p.z)
                * smoothstep(recipe.hipLiftMidline[0], recipe.hipLiftMidline[1], std::abs(p.y));
        };
        const int chain[] { skeleton.find("Pelvis"), skeleton.find("RightHipHelper25"), skeleton.find("RightHipHelper"),
            skeleton.find("RightHipHelper75"), skeleton.find("RightThigh") };
        const int pelvis[] { chain[0] };
        for (std::uint32_t v = 0; v < shorts.positions.size(); ++v) {
            const Vec3 p = shorts.positions[v];
            const Crease crease = creaseAt(p);
            const float band = std::clamp(0.5f + (dot(p - hip, legAlong) - crease.center) / crease.width, 0.0f, 1.0f);
            const float lift = liftAt(p);
            // Na dobra da frente a passagem tambem e direta: da cintura
            // dobrada para a coxa, sem aneis girando menos que ela.
            chainWeights(shorts, v, chain, band, 1.0f, crease.straight + (1.0f - crease.straight) * smoothstep(0.0f, 0.3f, lift));
            if (lift > 0.0f) moveWeights(shorts, v, pelvis, liftJoint, lift);
        }
        // A barra da camisa por cima: o peso da coxa que ela tinha (o da
        // pele) vai para a pelve e a mesma fracao da pelve para a HipLift.
        for (std::uint32_t v = 0; v < shirt.positions.size(); ++v) {
            const float lift = liftAt(shirt.positions[v]);
            if (lift <= 0.0f) continue;
            chainWeights(shirt, v, chain, 0.0f, std::min(1.0f, lift / 0.1f));
            moveWeights(shirt, v, pelvis, liftJoint, lift);
        }
    }
    // A costura do gancho presa na pelve: os vertices do meio entre as pernas
    // (abaixo da virilha) e a fracao dos pesos das coxas em volta deles.
    {
        std::vector<Vec3> seam;
        for (const Triangle& triangle : shorts.triangles)
            for (const std::uint32_t v : triangle) {
                const Vec3 p = shorts.positions[v];
                if (std::abs(p.y) < 1e-6f && p.z < hip.z - 0.02f) seam.push_back(p);
            }
        pinWeights(shorts, seam, recipe.crotchPinRadius, skeleton.find("Pelvis"), recipe.crotchPin);
    }
    // ---- As pecas na malha (partes e materiais) e os recuos ----
    MeshBuilder builder(mesh);
    builder.material("camisa", recipe.shirtColor);
    builder.material("calcao", recipe.shortsColor);
    const auto place = [&](Mesh& piece, const char* name) {
        for (std::size_t t = 0; t < piece.triangles.size(); ++t) piece.trianglePart[t] = 0;
        piece.parts = { MeshPart { name, "", {}, {}, false } };
        piece.materials = { MeshMaterial { name, std::string(name) == "camisa" ? recipe.shirtColor : recipe.shortsColor } };
        for (std::size_t t = 0; t < piece.triangleMaterial.size(); ++t) piece.triangleMaterial[t] = 0;
        return appendMesh(mesh, piece);
    };
    const std::uint32_t shirtFirst = place(shirt, "camisa");
    const std::uint32_t shortsFirst = place(shorts, "calcao");
    const auto shifted = [](CutBorder border, std::uint32_t offset) {
        for (std::uint32_t& v : border.vertices) v += offset;
        return border;
    };
    const auto partOf = [&](const char* name) { return static_cast<std::uint16_t>(mesh.findPart(name)); };
    const auto materialOf = [&](const char* name) { return static_cast<std::uint16_t>(mesh.findMaterial(name)); };
    // A camisa: a manga no braco, a gola no pescoco, a barra no calcao.
    if (!stitchRecess(mesh, shifted(shirtRegion.borders[1], shirtFirst), skin.borders[1], partOf("camisa"), materialOf("camisa"), &error)
        || !stitchRecess(mesh, shifted(shirtRegion.borders[2], shirtFirst), skin.borders[0], partOf("camisa"), materialOf("camisa"), &error)
        || !stitchRecess(mesh, shifted(shirtRegion.borders[0], shirtFirst), shifted(shortsRegion.borders[0], shortsFirst), partOf("camisa"),
            materialOf("camisa"), &error)
        || !stitchRecess(mesh, shifted(shortsRegion.borders[1], shortsFirst), skin.borders[2], partOf("calcao"), materialOf("calcao"), &error)) {
        error = "uniforme: " + error;
        return false;
    }
    // As costuras declaradas sao as que existem (a gola pode ficar so no
    // corpo ou subir ate a cabeca).
    std::map<std::pair<std::uint32_t, std::uint32_t>, std::set<int>> edgeParts;
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
        for (int i = 0; i < 3; ++i) {
            const std::uint32_t a = mesh.triangles[t][static_cast<std::size_t>(i)];
            const std::uint32_t b = mesh.triangles[t][static_cast<std::size_t>((i + 1) % 3)];
            edgeParts[{ std::min(a, b), std::max(a, b) }].insert(mesh.partOf(t));
        }
    for (const char* piece : { "camisa", "calcao" }) {
        const int part = mesh.findPart(piece);
        std::set<int> touching;
        for (const auto& [edge, parts] : edgeParts)
            if (parts.count(part) != 0)
                for (const int other : parts)
                    if (other != part) touching.insert(other);
        for (const int other : touching)
            if (other >= 0 && static_cast<std::size_t>(other) < mesh.parts.size())
                builder.part(piece).connectTo(mesh.parts[static_cast<std::size_t>(other)].name);
    }
    // Atras do quadril a camisa segue as coxas so em parte
    // (shirtBackThighWeight; o resto na pelve): a barra de tras pende do
    // tronco e a coxa muito dobrada (a largada agachada) nao vira o pano do
    // avesso. Na frente fica inteiro: a coxa levantada empurra a barra, que
    // sobe com ela (o calcao nao atravessa). A passagem em 3 cm em volta da
    // junta do quadril; todos os vertices dos triangulos da camisa (a faixa
    // da barra tambem, ate a borda do calcao).
    if (recipe.shirtBackThighWeight < 1.0f) {
        const int shirtPart = mesh.findPart("camisa"), pelvis = skeleton.find("Pelvis");
        const int thighs[] { skeleton.find("RightThigh"), skeleton.find("LeftThigh") };
        std::vector<bool> inShirt(mesh.positions.size(), false);
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (mesh.partOf(t) == shirtPart)
                for (const std::uint32_t v : mesh.triangles[t]) inShirt[v] = true;
        for (std::uint32_t v = 0; v < mesh.positions.size(); ++v)
            if (inShirt[v]) {
                const float back = smoothstep(hip.x + 0.03f, hip.x - 0.03f, mesh.positions[v].x);
                moveWeights(mesh, v, thighs, pelvis, (1.0f - recipe.shirtBackThighWeight) * back);
            }
    }
    // A gola anda com o peito (shirtNeckWeight): na camisa, o resto do peso do
    // pescoco vai para o alto do peito; o pescoco mexe por dentro da gola.
    // Todos os vertices dos triangulos da camisa (a borda de dentro da gola,
    // costurada na pele, tambem: a faixa do recuo anda inteira).
    if (recipe.shirtNeckWeight < 1.0f) {
        const int shirtPart = mesh.findPart("camisa"), chest = skeleton.find("UpperChest");
        const int neck[] { skeleton.find("Neck") };
        std::vector<bool> inShirt(mesh.positions.size(), false);
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (mesh.partOf(t) == shirtPart)
                for (const std::uint32_t v : mesh.triangles[t]) inShirt[v] = true;
        for (std::uint32_t v = 0; v < mesh.positions.size(); ++v)
            if (inShirt[v]) moveWeights(mesh, v, neck, chest, 1.0f - recipe.shirtNeckWeight);
    }
    return true;
}

} // namespace MatterEngine::Modeling
