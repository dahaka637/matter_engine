#include "Engine/Modeling/Assembly/PlayerAssembly.hpp"

#include "Engine/Modeling/Assembly/EyeSocket.hpp"
#include "Engine/Modeling/Assembly/MeshEdit.hpp"
#include "Engine/Modeling/Assembly/NeckBand.hpp"
#include "Engine/Modeling/Assembly/NeckJoin.hpp"
#include "Engine/Modeling/Assembly/PlayerPaint.hpp"
#include "Engine/Modeling/Assembly/PlayerPaintSpaces.hpp"
#include "Engine/Modeling/Assembly/Uniform.hpp"
#include "Engine/Modeling/Import/GltfImport.hpp"
#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshSculpt.hpp"
#include "Engine/Modeling/Mesh/MeshSkinning.hpp"
#include "Engine/Modeling/Mesh/MeshStitch.hpp"
#include "Engine/Modeling/Parts/BootGenerator.hpp"
#include "Engine/Modeling/Skin/BoneHeatSkin.hpp"
#include "Engine/Modeling/Skin/SkinSmoothing.hpp"
#include "Engine/Modeling/Texture/TextureBake.hpp"
#include "Engine/Modeling/Texture/UvAtlas.hpp"

#include <algorithm>
#include <fstream>
#include <optional>
#include <sstream>

namespace MatterEngine::Modeling {

namespace {

const Color SkinColor = rgb(212, 152, 106);

// A borda aberta unica de um corte (o pescoco na metade direita).
bool singleOpenBorder(const CutResult& cut, const char* what, std::vector<std::uint32_t>& border, std::string& error) {
    if (!cut.ok) {
        error = std::string(what) + ": " + cut.error;
        return false;
    }
    if (cut.borders.size() != 1 || cut.borders.front().closed) {
        error = std::string(what) + ": o corte devia dar um trecho so, do meio da frente ao de tras (deu "
            + std::to_string(cut.borders.size()) + ")";
        return false;
    }
    border = cut.borders.front().vertices;
    return true;
}

// A volta fechada unica de um corte (a perna acima da chuteira).
bool singleClosedLoop(const CutResult& cut, const char* what, std::vector<std::uint32_t>& loop, std::string& error) {
    if (!cut.ok) {
        error = std::string(what) + ": " + cut.error;
        return false;
    }
    if (cut.borders.size() != 1 || !cut.borders.front().closed) {
        error = std::string(what) + ": o corte devia dar uma volta so (deu " + std::to_string(cut.borders.size()) + ")";
        return false;
    }
    loop = cut.borders.front().vertices;
    return true;
}

// A chuteira direita nas juntas do pe (como na peca procedural), sem o
// cadarco em malha (ele vai pintado).
std::optional<BootPlacement> rightBootPlacement(const BootRecipe& recipe, const Skeleton& skeleton, float scale, std::string& error) {
    const int ankle = skeleton.find("RightFoot"), toe = skeleton.find("RightToe"), toeTip = skeleton.find("RightToeTip");
    if (ankle < 0 || toeTip < 0) {
        error = "chuteira: o esqueleto nao tem RightFoot/RightToeTip";
        return std::nullopt;
    }
    const Vec3 anklePosition = skeleton.joints[static_cast<std::size_t>(ankle)].bind.position;
    Vec3 forward = skeleton.joints[static_cast<std::size_t>(toeTip)].bind.position - anklePosition;
    forward.z = 0.0f;
    forward = safeNormalized(forward, { 1.0f, 0.0f, 0.0f });
    const Vec3 heel = anklePosition - forward * (recipe.heelBehindAnkle * scale);
    BootPlacement placement;
    placement.frame = { { heel.x, heel.y, skeleton.groundHeight }, frameAlong(forward, { 0.0f, 0.0f, 1.0f }) };
    placement.side = "direita";
    placement.region = "pe_direito";
    placement.footJoint = ankle;
    placement.toeJoint = toe;
    placement.collarOpening = false;
    placement.laceGeometry = false;
    if (toe >= 0)
        placement.toeBendAt = dot(skeleton.joints[static_cast<std::size_t>(toe)].bind.position - placement.frame.position, forward) / scale;
    return placement;
}

// A receita da chuteira (o arquivo relativo ao manifesto).
std::optional<BootRecipe> loadBootRecipe(const std::filesystem::path& path, std::string& error) {
    std::ifstream file(path);
    std::stringstream text;
    text << file.rdbuf();
    std::string parseError;
    std::optional<BootRecipe> recipe = file ? parseBootRecipe(text.str(), &parseError) : std::nullopt;
    if (!recipe) error = "chuteira: " + (file ? parseError : "nao abriu " + path.string());
    return recipe;
}

// A chuteira direita, escalada sobre o calcanhar.
bool buildRightBootOn(Mesh& mesh, const BootRecipe& recipe, const Skeleton& skeleton, float scale, BootSeams& seams,
    std::string& error) {
    const std::optional<BootPlacement> found = rightBootPlacement(recipe, skeleton, scale, error);
    if (!found) return false;
    const BootPlacement& placement = *found;
    const std::size_t first = mesh.positions.size();
    seams = buildBoot(mesh, recipe, placement);
    if (!seams.ok) {
        error = seams.error;
        return false;
    }
    for (std::size_t v = first; v < mesh.positions.size(); ++v)
        mesh.positions[v] = placement.frame.position + (mesh.positions[v] - placement.frame.position) * scale;
    return true;
}

} // namespace

AssemblyResult assembleRightHalf(const AssemblyRecipe& recipe, const std::filesystem::path& directory, const Skeleton& skeleton) {
    AssemblyResult result;
    StageClock clock(result.timings);
    const float ground = skeleton.groundHeight;
    // ---- O corpo: na altura e no chao do esqueleto, so a metade direita ----
    GltfImportSettings bodySettings;
    bodySettings.forward = recipe.bodyForward;
    bodySettings.part = "corpo";
    bodySettings.material = "pele";
    result.files = { directory / recipe.bodyFile, directory / recipe.headFile, directory / recipe.bootFile };
    if (!recipe.referenceFit.file.empty()) result.files.push_back(directory / recipe.referenceFit.file);
    GltfImportResult body = importGltfMesh(directory / recipe.bodyFile, bodySettings);
    if (!body.ok) {
        result.error = "corpo: " + body.error;
        return result;
    }
    Mesh& mesh = body.mesh;
    {
        const Box3 box = mesh.bounds();
        const float scale = recipe.bodyHeight / std::max(1e-6f, box.maximum.z - box.minimum.z);
        transformPositions(mesh, scale, { 0.0f, 0.0f, ground - box.minimum.z * scale });
    }
    if (const CutResult half = keepRightHalf(mesh); !half.ok) {
        result.error = "corpo: " + half.error;
        return result;
    }
    std::vector<std::uint32_t> bodyNeck;
    const CutPlane neckPlane = tiltedPlane(ground + recipe.neckHeight, recipe.neckTilt, 0.0f, true);
    const CutSettings neckCut { recipe.neckSnap };
    if (!singleOpenBorder(cutByPlane(mesh, { neckPlane.point, -neckPlane.normal }, neckCut), "pescoco do corpo", bodyNeck,
            result.error))
        return result;

    clock.mark("corpo");
    // ---- A cabeca: a metade direita, na escala, com o alto no lugar e o
    // centro do pescoco em cima do do corpo; cortada no mesmo plano ----
    GltfImportSettings headSettings;
    headSettings.forward = recipe.headForward;
    headSettings.part = "cabeca";
    headSettings.material = "pele";
    // A nuca da malha base tem uma fresta de 0,14 mm no meio: fecha.
    headSettings.weldTolerance = 5e-4f;
    GltfImportResult head = importGltfMesh(directory / recipe.headFile, headSettings);
    if (!head.ok) {
        result.error = "cabeca: " + head.error;
        return result;
    }
    Mesh& headMesh = head.mesh;
    headMesh.parts.front().root = false;
    if (const CutResult half = keepRightHalf(headMesh); !half.ok) {
        result.error = "cabeca: " + half.error;
        return result;
    }
    {
        const float top = headMesh.bounds().maximum.z * recipe.headScale;
        transformPositions(headMesh, recipe.headScale, { 0.0f, 0.0f, ground + recipe.headTop - top });
        // O centro do pescoco da cabeca no corte (numa copia) vai para cima do
        // centro do pescoco do corpo.
        Mesh probe = headMesh;
        std::vector<std::uint32_t> probeNeck;
        if (!singleOpenBorder(cutByPlane(probe, neckPlane, neckCut), "pescoco da cabeca", probeNeck, result.error)) return result;
        const float headCenter = 0.5f * (probe.positions[probeNeck.front()].x + probe.positions[probeNeck.back()].x);
        const float bodyCenter = 0.5f * (mesh.positions[bodyNeck.front()].x + mesh.positions[bodyNeck.back()].x);
        transformPositions(headMesh, 1.0f, { bodyCenter - headCenter + recipe.headShift, 0.0f, 0.0f });
    }
    std::vector<std::uint32_t> headNeck;
    if (!singleOpenBorder(cutByPlane(headMesh, neckPlane, neckCut), "pescoco da cabeca", headNeck, result.error)) return result;

    // ---- O encaixe do pescoco ----
    const std::uint32_t headFirst = appendMesh(mesh, headMesh);
    NeckJoin join;
    join.lower = bodyNeck;
    for (const std::uint32_t v : headNeck) join.upper.push_back(v + headFirst);
    for (std::uint32_t v = headFirst; v < mesh.positions.size(); ++v) join.upperVertices.push_back(v);
    join.plane = neckPlane;
    join.blend = recipe.neckBlend;
    join.weld = recipe.neckWeld;
    if (!joinNeck(mesh, join, &result.error)) return result;
    MeshBuilder builder(mesh);
    builder.part("cabeca").connectTo("corpo");
    clock.mark("cabeca e pescoco");

    // ---- O ajuste fino da cabeca (o queixo, a mandibula, a papada) e o
    // pescoco mais curto (o corpo sobe por baixo da cabeca) ----
    {
        std::vector<SculptPull> pulls = recipe.headSculpt;
        for (SculptPull& pull : pulls) pull.at.z += ground;
        sculptMesh(mesh, pulls);
        for (Vec3& position : mesh.positions) position = recipe.neck.apply(position, ground);
        // A forma do rosto e do pescoco: os vertices nos lugares da receita.
        std::vector<VertexMove> moves = recipe.faceShape;
        for (VertexMove& move : moves) {
            move.at.z += ground;
            move.to.z += ground;
        }
        if (!moveVertices(mesh, moves, 0.002f, result.error)) {
            result.error = "faceShape: " + result.error;
            return result;
        }
        // O queixo, a mandibula e o pescoco no molde da referencia.
        if (!fitToReference(mesh, recipe.referenceFit, directory, uniformLines(recipe, skeleton), mesh.findPart("cabeca"),
                result.referenceFit, result.error))
            return result;
    }

    clock.mark("rosto e molde da referencia");
    // ---- Os olhos modelados (desligados por padrao: pintados na textura) ----
    if (recipe.modeledEyes)
        if (const EyeSocketResult eye = buildEyeSocket(mesh, recipe.eye, mesh.findPart("cabeca")); !eye.ok) {
            result.error = eye.error;
            return result;
        }

    // ---- A chuteira: o pe da malha base sai, a perna e cortada acima da
    // gola e desce costurada ate ela ----
    {
        const std::optional<BootRecipe> bootRecipe = loadBootRecipe(directory / recipe.bootFile, result.error);
        if (!bootRecipe) return result;
        BootSeams seams;
        Mesh boot;
        if (!buildRightBootOn(boot, *bootRecipe, skeleton, recipe.bootScale, seams, result.error)) return result;
        float collarTop = -1e9f;
        for (const std::uint32_t v : seams.collar) collarTop = std::max(collarTop, boot.positions[v].z);
        std::vector<std::uint32_t> leg;
        if (!singleClosedLoop(cutByPlane(mesh, { { 0.0f, 0.0f, collarTop + recipe.bootCollarGap }, { 0.0f, 0.0f, 1.0f } }),
                "perna acima da chuteira", leg, result.error))
            return result;
        const std::uint32_t bootFirst = appendMesh(mesh, boot);
        std::vector<std::uint32_t> collar;
        for (const std::uint32_t v : seams.collar) collar.push_back(v + bootFirst);
        // As duas voltas anti-horarias vistas de cima: a gola embaixo, a perna
        // em cima; a faixa entre elas e a meia entrando na chuteira.
        if (loopWinding(mesh, collar, { 0.0f, 0.0f, 1.0f }) < 0.0f) collar = reversedLoop(collar);
        if (loopWinding(mesh, leg, { 0.0f, 0.0f, 1.0f }) < 0.0f) leg = reversedLoop(leg);
        builder.part("corpo").useMaterial("pele");
        stitchLoops(builder, collar, leg);
        builder.part("corpo").connectTo(seams.upperPart);
    }

    clock.mark("chuteira");
    // ---- Os pesos do corpo (a metade direita; o espelho leva) ----
    builder.material("pele", SkinColor);
    {
        const std::vector<int> skinParts { mesh.findPart("corpo"), mesh.findPart("cabeca") };
        BoneHeatSettings settings;
        settings.rightHalf = true;
        // O rosto preso na cabeca (gira inteiro com ela, sem deformar) e a base
        // do pescoco no alto do peito (a gola anda com o peito); o pescoco
        // passa de um ao outro (com neckGradient, em linha ao longo da faixa).
        NeckBand neckBand;
        neckBand.lines = uniformLines(recipe, skeleton);
        neckBand.headAbove = recipe.headPinAbove;
        neckBand.frontBase = recipe.neckFrontBase;
        neckBand.width = recipe.collarPinWidth;
        const UniformLines& lines = neckBand.lines;
        const int head = skeleton.find("Head"), chest = skeleton.find("UpperChest"), neck = skeleton.find("Neck");
        std::vector<bool> isHead(mesh.positions.size(), false);
        for (std::size_t t = 0; t < mesh.triangles.size(); ++t)
            if (mesh.partOf(t) == mesh.findPart("cabeca"))
                for (const std::uint32_t v : mesh.triangles[t]) isHead[v] = true;
        settings.pin = [&](std::uint32_t v) -> std::optional<SkinWeights> {
            const Vec3 p = mesh.positions[v];
            const float h = p.z - ground;
            SkinWeights only {};
            only.weights[0] = 1.0f;
            if (head >= 0 && isHead[v] && h > recipe.neck.top(p.x) + recipe.headPinAbove) {
                only.joints[0] = static_cast<std::uint16_t>(head);
                return only;
            }
            if (recipe.neckGradient)
                if (const std::optional<SkinWeights> band = neckBandWeights(neckBand, p, chest, neck, head)) return band;
            // Com o gradiente, a pele do V entre a gola e a base do pescoco
            // tambem (ela e peito).
            const float collar = lines.collarHeight(p);
            const float pinTop = recipe.neckGradient ? neckBand.base(p) : collar;
            if (chest >= 0 && std::abs(p.y) < recipe.collarPinWidth && p.z <= pinTop && p.z > collar - recipe.collarPinDepth) {
                only.joints[0] = static_cast<std::uint16_t>(chest);
                return only;
            }
            return std::nullopt;
        };
        if (const BoneHeatResult skin = boneHeatSkin(mesh, skeleton, skinParts, settings); !skin.ok) {
            result.error = skin.error;
            return result;
        }
        // Os dedos e o polegar: os pesos espalhados so nos vertices deles (os
        // da palma em volta seguram os deles).
        if (recipe.fingerWeightSmoothing > 0) {
            const std::vector<int> fingers = fingerJoints(skeleton, Side::Right, true);
            smoothSkinWeights(mesh, recipe.fingerWeightSmoothing, 0.5f, [&](std::uint32_t v) {
                float share = 0.0f;
                for (const int joint : fingers) share += weightFor(mesh.skin[v], joint);
                return share > 0.02f;
            });
        }
    }
    // Sem os vertices que os cortes soltaram.
    MeshBuilder(mesh).compact();
    clock.mark("pesos (calor dos ossos)");
    result.mesh = std::move(mesh);
    result.ok = true;
    return result;
}

void mirrorHalf(Mesh& mesh, const Skeleton& skeleton, float midlineBlend) {
    std::vector<std::string> names;
    for (const MeshPart& part : mesh.parts) names.push_back(part.name);
    std::vector<int> jointMirror(skeleton.joints.size());
    for (std::size_t j = 0; j < skeleton.joints.size(); ++j) jointMirror[j] = mirroredJoint(skeleton, static_cast<int>(j));
    blendMidlineWeights(mesh, jointMirror, midlineBlend);
    MeshBuilder(mesh).mirrorParts(names, jointMirror);
    MeshBuilder(mesh).compact();
}

AssemblyResult assemblePlayer(const AssemblyRecipe& recipe, const std::filesystem::path& directory, const Skeleton& skeleton) {
    AssemblyResult result = assembleRightHalf(recipe, directory, skeleton);
    if (!result.ok) return result;
    StageClock clock(result.timings);
    if (recipe.uniform && !buildUniform(result.mesh, recipe, skeleton, result.error)) {
        result.ok = false;
        return result;
    }
    clock.mark("uniforme");
    // O avesso que sobra nas poses do jogo, corrigido nos pontos medidos.
    relaxWeights(result.mesh, recipe.weightRelax, recipe.weightRelaxIterations);
    mirrorHalf(result.mesh, skeleton, recipe.midlineBlend);
    clock.mark("pesos finos e espelho");
    if (recipe.textured) {
        // O mapa de textura e a pintura (o atlas unico de todos os materiais).
        UvAtlasSettings atlas;
        atlas.size = recipe.atlasSize;
        atlas.partDensity = { { "cabeca", recipe.faceDensity }, { "camisa", recipe.garmentDensity }, { "calcao", recipe.garmentDensity } };
        for (const char* side : { "direita", "esquerda" })
            for (const char* piece : { "chuteira_cabedal_", "chuteira_lingua_" })
                atlas.partDensity.push_back({ std::string(piece) + side, recipe.laceDensity });
        const UvAtlasResult unwrapped = unwrapAtlas(result.mesh, atlas);
        if (!unwrapped.ok) {
            result.ok = false;
            result.error = unwrapped.error;
            return result;
        }
        result.texelsPerMeter = unwrapped.texelsPerMeter;
        result.atlasCoverage = unwrapped.coverage;
        clock.mark("atlas (mapa e arranjo)");
        // O cadarco pintado na chuteira direita (o espelho leva para a esquerda).
        std::optional<BootPaint> boot;
        if (const std::optional<BootRecipe> bootRecipe = loadBootRecipe(directory / recipe.bootFile, result.error))
            if (const std::optional<BootPlacement> placement = rightBootPlacement(*bootRecipe, skeleton, recipe.bootScale, result.error))
                boot = BootPaint { *bootRecipe, placement->frame, recipe.bootScale };
        if (!boot) {
            result.ok = false;
            return result;
        }
        // O desenho (pintura.json) nos espacos das pecas medidos na malha final.
        PlayerPaintDesign design;
        if (!recipe.paintFile.empty()) {
            auto spaces = std::make_shared<std::map<std::string, PaintSpace>>(playerPaintSpaces(result.mesh, recipe, skeleton));
            std::optional<PaintDesign> loaded = loadPaintDesign(directory / recipe.paintFile, *spaces, &result.error);
            if (!loaded) {
                result.ok = false;
                return result;
            }
            result.files.insert(result.files.end(), loaded->files.begin(), loaded->files.end());
            design.design = std::make_shared<PaintDesign>(std::move(*loaded));
            design.spaces = std::move(spaces);
        }
        clock.mark("espacos e desenho da pintura");
        TextureBakeSettings bake;
        bake.size = recipe.atlasSize;
        bake.samples = 3;
        bake.edgeTexels = recipe.paintEdge;
        // O rosto e o cadarco tem traco mais fino que um texel: todas as
        // amostras sempre (no resto, so nas bordas do desenho).
        bake.fineParts.push_back(result.mesh.findPart("cabeca"));
        for (const char* side : { "direita", "esquerda" })
            for (const char* piece : { "chuteira_cabedal_", "chuteira_lingua_" }) bake.fineParts.push_back(result.mesh.findPart(std::string(piece) + side));
        Image image = bakeTexture(result.mesh, playerPaint(result.mesh, recipe, skeleton, design, &*boot), bake);
        result.mesh.textures.push_back({ "jogador", std::move(image) });
        for (MeshMaterial& material : result.mesh.materials) material.texture = 0;
        clock.mark("pintura (bake)");
    }
    return result;
}

} // namespace MatterEngine::Modeling
