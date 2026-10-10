#include "Engine/Modeling/Parts/PartLibrary.hpp"

#include "Engine/Modeling/Mesh/MeshBuilder.hpp"
#include "Engine/Modeling/Mesh/MeshRepair.hpp"
#include "Engine/Modeling/Parts/ArmGenerator.hpp"
#include "Engine/Modeling/Parts/BootGenerator.hpp"
#include "Engine/Modeling/Parts/HandGenerator.hpp"
#include "Engine/Modeling/Parts/HeadGenerator.hpp"
#include "Engine/Modeling/Parts/LegGenerator.hpp"
#include "Engine/Modeling/Parts/NeckGenerator.hpp"
#include "Engine/Modeling/Parts/ShirtGenerator.hpp"
#include "Engine/Modeling/Parts/ShortsGenerator.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>

namespace MatterEngine::Modeling {

namespace {

std::optional<std::string> readText(const std::filesystem::path& path) {
    std::ifstream file(path);
    if (!file) return std::nullopt;
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// A chuteira no pe direito: o referencial vem do tornozelo (a junta do pe) e
// da ponta dos dedos; a dobra fica na base dos dedos. Com perna, a gola fica
// para ela costurar.
std::optional<BootSeams> buildRightBoot(Mesh& mesh, const BootRecipe& recipe, const Skeleton& skeleton, bool withLeg,
    std::string* error) {
    const int ankle = skeleton.find("RightFoot");
    const int toe = skeleton.find("RightToe");
    const int toeTip = skeleton.find("RightToeTip");
    if (ankle < 0 || toeTip < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem RightFoot/RightToeTip para a chuteira";
        return std::nullopt;
    }
    const Vec3 anklePosition = skeleton.joints[static_cast<std::size_t>(ankle)].bind.position;
    const Vec3 tipPosition = skeleton.joints[static_cast<std::size_t>(toeTip)].bind.position;
    Vec3 forward = tipPosition - anklePosition;
    forward.z = 0.0f;
    forward = safeNormalized(forward, { 1.0f, 0.0f, 0.0f });
    const Vec3 heel = anklePosition - forward * recipe.heelBehindAnkle;
    BootPlacement placement;
    placement.frame = { { heel.x, heel.y, skeleton.groundHeight }, frameAlong(forward, { 0.0f, 0.0f, 1.0f }) };
    placement.side = "direita";
    placement.region = "pe_direito";
    placement.footJoint = ankle;
    placement.toeJoint = toe;
    placement.collarOpening = !withLeg;
    if (toe >= 0)
        placement.toeBendAt = dot(skeleton.joints[static_cast<std::size_t>(toe)].bind.position - placement.frame.position, forward);
    BootSeams seams = buildBoot(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A perna direita no eixo do osso (o joelho visto de cima), da gola ate a
// barra.
std::optional<LegSeams> buildRightLeg(Mesh& mesh, const LegRecipe& recipe, const Skeleton& skeleton, const BootSeams& boot,
    bool withShorts, std::string* error) {
    const int foot = skeleton.find("RightFoot"), shin = skeleton.find("RightShin"), thigh = skeleton.find("RightThigh");
    if (foot < 0 || shin < 0 || thigh < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem RightThigh/RightShin/RightFoot para a perna";
        return std::nullopt;
    }
    const Vec3 knee = skeleton.joints[static_cast<std::size_t>(shin)].bind.position;
    const Vec3 ankle = skeleton.joints[static_cast<std::size_t>(foot)].bind.position;
    LegPlacement placement;
    placement.axis = { knee.x, knee.y };
    placement.ground = skeleton.groundHeight;
    placement.ankleHeight = ankle.z - skeleton.groundHeight;
    placement.kneeHeight = knee.z - skeleton.groundHeight;
    placement.footJoint = foot;
    placement.shinJoint = shin;
    placement.thighJoint = thigh;
    placement.kneeHelperJoint = skeleton.find("RightKneeHelper");
    placement.ankleHelperJoint = skeleton.find("RightAnkleHelper");
    placement.medialSign = knee.y < 0.0f ? 1.0f : -1.0f;
    placement.collar = boot.collar;
    placement.bootPart = boot.upperPart;
    placement.topOpening = !withShorts;
    LegSeams seams = buildLeg(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A metade direita do calcao, costurada na barra da perna direita (com a
// camisa, ele para onde a dobra da barra dela costura).
struct ShortsCover {
    float height = 0.0f;
    float abdomenFrom = 0.0f;
    float abdomenTo = 0.0f;
};

std::optional<ShortsSeams> buildRightShorts(Mesh& mesh, const ShortsRecipe& recipe, const Skeleton& skeleton,
    const LegSeams& leg, const ShortsCover& cover, std::string* error) {
    const int pelvis = skeleton.find("Pelvis"), thigh = skeleton.find("RightThigh"), leftThigh = skeleton.find("LeftThigh");
    const int shin = skeleton.find("RightShin");
    if (pelvis < 0 || thigh < 0 || leftThigh < 0 || shin < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem Pelvis/RightThigh/LeftThigh para o calcao";
        return std::nullopt;
    }
    const Vec3 knee = skeleton.joints[static_cast<std::size_t>(shin)].bind.position;
    ShortsPlacement placement;
    placement.legAxis = { knee.x, knee.y };
    placement.bodyX = skeleton.joints[static_cast<std::size_t>(pelvis)].bind.position.x;
    placement.ground = skeleton.groundHeight;
    placement.pelvisJoint = pelvis;
    placement.abdomenJoint = skeleton.find("Abdomen");
    placement.rightThighJoint = thigh;
    placement.leftThighJoint = leftThigh;
    placement.rightHipHelperJoint = skeleton.find("RightHipHelper");
    placement.leftHipHelperJoint = skeleton.find("LeftHipHelper");
    placement.legTop = leg.top;
    placement.legPart = leg.skinPart;
    placement.coverHeight = cover.height;
    placement.abdomenFrom = cover.abdomenFrom;
    placement.abdomenTo = cover.abdomenTo;
    ShortsSeams seams = buildShortsHalf(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A metade direita da camisa, por cima do calcao (a dobra da barra costura
// no alto dele), com a manga no braco direito.
std::optional<ShirtSeams> buildRightShirt(Mesh& mesh, const ShirtRecipe& recipe, const Skeleton& skeleton,
    const ShortsSeams& shorts, bool withArm, bool withNeck, std::string* error) {
    const int pelvis = skeleton.find("Pelvis"), abdomen = skeleton.find("Abdomen"), chest = skeleton.find("Chest");
    const int upperChest = skeleton.find("UpperChest"), upperArm = skeleton.find("RightUpperArm");
    const int forearm = skeleton.find("RightForearm");
    if (pelvis < 0 || abdomen < 0 || chest < 0 || upperChest < 0 || upperArm < 0 || forearm < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem a coluna (Pelvis..UpperChest) e o braco direito para a camisa";
        return std::nullopt;
    }
    const auto at = [&](int joint) { return skeleton.joints[static_cast<std::size_t>(joint)].bind.position; };
    ShirtPlacement placement;
    placement.ground = skeleton.groundHeight;
    placement.shoulder = at(upperArm);
    placement.elbow = at(forearm);
    placement.pelvisJoint = pelvis;
    placement.abdomenJoint = abdomen;
    placement.chestJoint = chest;
    placement.upperChestJoint = upperChest;
    placement.pelvisHeight = at(pelvis).z - skeleton.groundHeight;
    placement.abdomenHeight = at(abdomen).z - skeleton.groundHeight;
    placement.chestHeight = at(chest).z - skeleton.groundHeight;
    placement.upperChestHeight = at(upperChest).z - skeleton.groundHeight;
    placement.upperArmJoint = upperArm;
    placement.shoulderHelperJoint = skeleton.find("RightShoulderHelper");
    placement.tuck = shorts.top;
    placement.tuckPart = shorts.part;
    placement.cuffOpening = !withArm;
    placement.collarOpening = !withNeck;
    ShirtSeams seams = buildShirtHalf(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A pele do braco direito, da abertura do punho da manga ate o pulso (com
// a mao, o pulso fica para ela costurar).
std::optional<ArmSeams> buildRightArm(Mesh& mesh, const ArmRecipe& recipe, const Skeleton& skeleton, const ShirtSeams& shirt,
    bool withHand, std::string* error) {
    const int upperArm = skeleton.find("RightUpperArm"), forearm = skeleton.find("RightForearm");
    if (upperArm < 0 || forearm < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem RightUpperArm/RightForearm para o braco";
        return std::nullopt;
    }
    ArmPlacement placement;
    placement.shoulder = skeleton.joints[static_cast<std::size_t>(upperArm)].bind.position;
    placement.elbow = skeleton.joints[static_cast<std::size_t>(forearm)].bind.position;
    placement.upperArmJoint = upperArm;
    placement.elbowHelperJoint = skeleton.find("RightElbowHelper");
    placement.forearmJoint = forearm;
    placement.wristHelperJoint = skeleton.find("RightWristHelper");
    placement.handJoint = skeleton.find("RightHand");
    if (placement.handJoint >= 0)
        placement.wristStation =
            distance(skeleton.joints[static_cast<std::size_t>(placement.handJoint)].bind.position, placement.shoulder);
    placement.cuff = shirt.cuff;
    placement.sleevePart = shirt.sleevePart;
    placement.wristOpening = !withHand;
    ArmSeams seams = buildArm(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A mao direita no anel do pulso: as juntas dos dedos e o dorso de cada
// falange (o contrario da flexao, que gira em volta do -y do osso).
std::optional<HandSeams> buildRightHand(Mesh& mesh, const HandRecipe& recipe, const Skeleton& skeleton, const ArmSeams& arm,
    std::string* error) {
    const int handJoint = skeleton.find("RightHand");
    if (handJoint < 0) {
        if (error != nullptr) *error = "o esqueleto nao tem RightHand para a mao";
        return std::nullopt;
    }
    const auto digit = [&](const std::string& name, DigitPlacement& placement) {
        for (std::size_t k = 0; k < 3; ++k) {
            const int joint = skeleton.find("Right" + name + std::to_string(k + 1));
            if (joint < 0) return false;
            const Transform& bind = skeleton.joints[static_cast<std::size_t>(joint)].bind;
            placement.points[k] = bind.position;
            placement.dorsal[k] = -axisZ(bind.orientation);
            placement.joints[k] = joint;
        }
        const int tip = skeleton.find("Right" + name + "Tip");
        if (tip < 0) return false;
        placement.points[3] = skeleton.joints[static_cast<std::size_t>(tip)].bind.position;
        return true;
    };
    HandPlacement placement;
    placement.wrist = skeleton.joints[static_cast<std::size_t>(handJoint)].bind.position;
    placement.handJoint = handJoint;
    placement.wristHelperJoint = skeleton.find("RightWristHelper");
    const char* names[] = { "Index", "Middle", "Ring", "Pinky" };
    for (std::size_t i = 0; i < 4; ++i)
        if (!digit(names[i], placement.fingers[i])) {
            if (error != nullptr) *error = std::string("o esqueleto nao tem o dedo Right") + names[i] + "1..3/Tip para a mao";
            return std::nullopt;
        }
    if (!digit("Thumb", placement.thumb)) {
        if (error != nullptr) *error = "o esqueleto nao tem o polegar RightThumb1..3/Tip para a mao";
        return std::nullopt;
    }
    placement.wristRing = arm.wrist;
    placement.forearmPart = arm.forearmPart;
    HandSeams seams = buildHand(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A metade direita do pescoco, da volta de dentro da gola ate a cabeca (com
// a cabeca, a volta de cima fica para ela costurar).
std::optional<NeckSeams> buildRightNeck(Mesh& mesh, const NeckRecipe& recipe, const Skeleton& skeleton, const ShirtSeams& shirt,
    bool withHead, std::string* error) {
    NeckPlacement placement;
    placement.ground = skeleton.groundHeight;
    placement.upperChestJoint = skeleton.find("UpperChest");
    placement.neckJoint = skeleton.find("Neck");
    placement.headJoint = skeleton.find("Head");
    placement.collar = shirt.collar;
    placement.collarPart = shirt.collarPart;
    placement.headOpening = !withHead;
    NeckSeams seams = buildNeckHalf(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

// A metade direita da cabeca, na volta de cima do pescoco.
std::optional<HeadSeams> buildRightHead(Mesh& mesh, const HeadRecipe& recipe, const Skeleton& skeleton, const NeckSeams& neck,
    std::string* error) {
    HeadPlacement placement;
    placement.ground = skeleton.groundHeight;
    placement.neckJoint = skeleton.find("Neck");
    placement.headJoint = skeleton.find("Head");
    placement.neckRing = neck.top;
    placement.neckPart = neck.part;
    HeadSeams seams = buildHeadHalf(mesh, recipe, placement);
    if (!seams.ok) {
        if (error != nullptr) *error = seams.error;
        return std::nullopt;
    }
    return seams;
}

} // namespace

std::optional<Mesh> buildProceduralParts(std::span<const std::filesystem::path> files, const Skeleton& skeleton,
    std::string* error) {
    // As receitas por tipo (cada peca no maximo uma vez).
    std::optional<BootRecipe> boot;
    std::optional<LegRecipe> leg;
    std::optional<ShortsRecipe> shorts;
    std::optional<ShirtRecipe> shirt;
    std::optional<ArmRecipe> arm;
    std::optional<HandRecipe> hand;
    std::optional<NeckRecipe> neck;
    std::optional<HeadRecipe> head;
    bool mirror = true;
    for (const std::filesystem::path& file : files) {
        const std::optional<std::string> text = readText(file);
        if (!text) {
            if (error != nullptr) *error = "nao abriu " + file.string();
            return std::nullopt;
        }
        nlohmann::json document;
        try {
            document = nlohmann::json::parse(*text);
        } catch (const std::exception& exception) {
            if (error != nullptr) *error = file.filename().string() + ": " + exception.what();
            return std::nullopt;
        }
        const std::string type = document.value("type", std::string());
        std::string recipeError;
        if (type == "boot") {
            boot = parseBootRecipe(*text, &recipeError);
            mirror = mirror && document.value("mirror", true);
        } else if (type == "leg") {
            leg = parseLegRecipe(*text, &recipeError);
        } else if (type == "shorts") {
            shorts = parseShortsRecipe(*text, &recipeError);
        } else if (type == "shirt") {
            shirt = parseShirtRecipe(*text, &recipeError);
        } else if (type == "arm") {
            arm = parseArmRecipe(*text, &recipeError);
        } else if (type == "hand") {
            hand = parseHandRecipe(*text, &recipeError);
        } else if (type == "neck") {
            neck = parseNeckRecipe(*text, &recipeError);
        } else if (type == "head") {
            head = parseHeadRecipe(*text, &recipeError);
        } else {
            if (error != nullptr) *error = file.filename().string() + ": tipo de peca desconhecido \"" + type + "\"";
            return std::nullopt;
        }
        if (!recipeError.empty()) {
            if (error != nullptr) *error = file.filename().string() + ": " + recipeError;
            return std::nullopt;
        }
    }
    // A metade direita, de baixo para cima: cada peca comeca da borda da
    // anterior (a gola da chuteira, a barra da perna).
    Mesh mesh;
    if (leg && !boot) {
        if (error != nullptr) *error = "a perna comeca na gola da chuteira: falta partes/chuteira.json";
        return std::nullopt;
    }
    if (shorts && !leg) {
        if (error != nullptr) *error = "o calcao comeca na barra da perna: falta partes/perna.json";
        return std::nullopt;
    }
    if (shirt && !shorts) {
        if (error != nullptr) *error = "a camisa comeca no alto do calcao: falta partes/calcao.json";
        return std::nullopt;
    }
    if ((arm || neck) && !shirt) {
        if (error != nullptr) *error = "o braco e o pescoco comecam no punho e na gola: falta partes/camisa.json";
        return std::nullopt;
    }
    if (hand && !arm) {
        if (error != nullptr) *error = "a mao comeca no pulso: falta partes/braco.json";
        return std::nullopt;
    }
    if (head && !neck) {
        if (error != nullptr) *error = "a cabeca comeca no alto do pescoco: falta partes/pescoco.json";
        return std::nullopt;
    }
    std::string root;
    if (boot) {
        const std::optional<BootSeams> bootSeams = buildRightBoot(mesh, *boot, skeleton, leg.has_value(), error);
        if (!bootSeams) return std::nullopt;
        root = bootSeams->upperPart;
        if (leg) {
            const std::optional<LegSeams> legSeams = buildRightLeg(mesh, *leg, skeleton, *bootSeams, shorts.has_value(), error);
            if (!legSeams) return std::nullopt;
            root = legSeams->skinPart;
            if (shorts) {
                ShortsCover cover;
                if (shirt) {
                    // A camisa por cima: o calcao para na dobra da barra e a
                    // cintura dele passa ao abdomen na mesma faixa da barra.
                    cover.height = shirt->tuckHeight;
                    cover.abdomenFrom = shirt->hemHeight - 0.08f;
                    cover.abdomenTo = shirt->hemHeight + 0.05f;
                }
                const std::optional<ShortsSeams> shortsSeams =
                    buildRightShorts(mesh, *shorts, skeleton, *legSeams, cover, error);
                if (!shortsSeams) return std::nullopt;
                root = shortsSeams->part;
                if (shirt) {
                    const std::optional<ShirtSeams> shirtSeams =
                        buildRightShirt(mesh, *shirt, skeleton, *shortsSeams, arm.has_value(), neck.has_value(), error);
                    if (!shirtSeams) return std::nullopt;
                    root = shirtSeams->part;
                    if (arm) {
                        const std::optional<ArmSeams> armSeams =
                            buildRightArm(mesh, *arm, skeleton, *shirtSeams, hand.has_value(), error);
                        if (!armSeams) return std::nullopt;
                        if (hand && !buildRightHand(mesh, *hand, skeleton, *armSeams, error)) return std::nullopt;
                    }
                    if (neck) {
                        const std::optional<NeckSeams> neckSeams =
                            buildRightNeck(mesh, *neck, skeleton, *shirtSeams, head.has_value(), error);
                        if (!neckSeams) return std::nullopt;
                        if (head && !buildRightHead(mesh, *head, skeleton, *neckSeams, error)) return std::nullopt;
                    }
                }
            }
        }
    }
    // O corpo: a peca de cima (as outras chegam nela pela superficie).
    if (const int part = mesh.findPart(root); part >= 0) mesh.parts[static_cast<std::size_t>(part)].root = true;
    // A metade esquerda: o espelho de tudo (as costuras vao junto).
    if (mirror) {
        std::vector<int> jointMirror(skeleton.joints.size());
        for (std::size_t j = 0; j < skeleton.joints.size(); ++j) jointMirror[j] = mirroredJoint(skeleton, static_cast<int>(j));
        std::vector<std::string> names;
        for (const MeshPart& part : mesh.parts) names.push_back(part.name);
        MeshBuilder(mesh).mirrorParts(names, jointMirror);
    }
    // Os numeros da camisa (cruzam o meio: depois do espelho).
    if (shirt && mirror && !imprintShirtNumbers(mesh, *shirt, skeleton.groundHeight, "camisa", error)) return std::nullopt;
    // O numero so na perna esquerda do calcao (depois do espelho).
    if (shorts && mirror) {
        const int leftShin = skeleton.find("LeftShin");
        const Vec3 knee = leftShin >= 0 ? skeleton.joints[static_cast<std::size_t>(leftShin)].bind.position : Vec3 {};
        if (!imprintShortsNumber(mesh, *shorts, { knee.x, knee.y }, skeleton.groundHeight, "calcao", error)) return std::nullopt;
    }
    // Os recortes deixam vertices de dentro dos furos sem triangulo e
    // agulhas planas nas juncoes em T.
    MeshBuilder(mesh).compact();
    flipFlatNeedles(mesh);
    return mesh;
}

} // namespace MatterEngine::Modeling
