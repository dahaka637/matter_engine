#include "Engine/Physique/PlayerPhysique3D.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <string>

namespace MatterEngine {

namespace {

// A folha de pesos dele (26/10; assets/characters/player_model/referencias/
// corpo_peso_70_80_90.png): a largura de cada regiao de frente, a de 70 kg e a
// de 90 kg sobre a de 80 (medidas no vault, plano_corpo_variavel). O de 90 kg
// da folha engorda quase por igual (~1,17); a cabeca quase nao. A encorpada foi
// retocada por ordem dele (26/10, noite: "menos sensacao de gordura, parecer
// mais musculo, de forma equilibrada"; "algo mais anatomico e menos exagerado"):
// a cintura e o quadril crescem menos que o peito (um V leve) e as costas e o
// alto do braco quase nao crescem para tras (sem corcunda). A chuteira e
// rigida: o pe nao muda.
struct GirthRatios {
    float light;
    float heavy;
};
constexpr std::array<GirthRatios, static_cast<std::size_t>(PhysiqueRegion3D::Count)> Girth { {
    { 0.945f, 1.140f }, // Pelvis: o quadril (o encorpado e musculo: o quadril e a cintura crescem menos que o peito)
    { 0.915f, 1.120f }, // Abdomen: a cintura
    { 0.940f, 1.210f }, // Chest: o peito (a folha 0,918)
    { 0.890f, 1.240f }, // UpperChest: o peito alto, as costas e os ombros
    { 0.880f, 1.100f }, // Neck
    { 0.967f, 1.025f }, // Head: o rosto do encorpado redondo era gordura
    { 0.940f, 1.220f }, // UpperArm
    { 0.950f, 1.200f }, // Forearm: o antebraco e o punho
    { 1.000f, 1.060f }, // Hand: a folha engorda a mao 18%; a anatomia (e o punho fechado) nao. Nem afina: o punho do franzino virava
    { 0.900f, 1.240f }, // Thigh: a coxa e o joelho
    { 0.920f, 1.200f }, // Shin: a panturrilha e o tornozelo
    { 1.000f, 1.000f }, // Foot: a chuteira
} };

// O volume da malha no tonus sobre o de hoje (o inspetor, `medir --corpo`, a
// 1,80 m: 84,28 L no corpo de hoje; 73,88 e 104,19 L em -1 e +1). A malha anda
// em linha reta com o tonus, entao o volume e um cubico exato em |w| de cada
// lado: 1 + a|w| + b w^2 + c|w|^3.
constexpr std::array<float, 3> HeavyVolume { 0.223078f, 0.013038f, 0.000112f };
constexpr std::array<float, 3> LightVolume { -0.127811f, 0.004528f, -0.000074f };

// Abaixo de -2,4 (so os altos e leves: a 2,00 m, menos de ~79 kg) o tonus
// anda a um terco do ritmo: a virilha dobra numa agulha a partir de ~-3,2 (o
// canto, 2,00 m e 60 kg, pediria -4,1) e o punho fechado vira; assim o canto
// para em ~-3,0. A fisica continua com o peso de verdade.
constexpr float LightKnee = 2.4f;
// Acima de +0,9 a metade do ritmo (26/10, pedido dele: "uma suavizada" nos mais
// pesados, "menos rechonchudos"; era +1,2): o canto de 1,70 m e 100 kg (que
// pediria +1,94; a +1,68 o meiao atravessava a chuteira) para em ~+1,42 e o de
// 1,80 m e 100 kg em ~+0,98.
constexpr float HeavyKnee = 0.9f;

float sideVolume(const std::array<float, 3>& c, float u) { return 1.0f + u * (c[0] + u * (c[1] + u * c[2])); }

bool finite(float value) { return std::isfinite(value); }

} // namespace

PlayerBody3D clampedBody3D(PlayerBody3D body) {
    if (!finite(body.heightMeters)) body.heightMeters = BaseBodyHeightMeters;
    if (!finite(body.massKg)) body.massKg = BaseBodyMassKg;
    body.heightMeters = std::clamp(body.heightMeters, MinimumBodyHeightMeters, MaximumBodyHeightMeters);
    body.massKg = std::clamp(body.massKg, MinimumBodyMassKg, MaximumBodyMassKg);
    return body;
}

std::optional<PlayerBody3D> parseBody3D(std::string_view text) {
    std::array<float, 2> values {};
    std::size_t count = 0, at = 0;
    while (at < text.size() && count < values.size()) {
        while (at < text.size() && std::string_view(",;x/: ").find(text[at]) != std::string_view::npos) ++at;
        if (at >= text.size()) break;
        float value = 0.0f;
        const auto [end, error] = std::from_chars(text.data() + at, text.data() + text.size(), value);
        if (error != std::errc {} || end == text.data() + at) return std::nullopt;
        values[count++] = value;
        at = static_cast<std::size_t>(end - text.data());
    }
    while (at < text.size() && text[at] == ' ') ++at;
    if (count != 2 || at != text.size()) return std::nullopt;
    // A altura em cm (ou em metros, abaixo de 3).
    PlayerBody3D body { values[0] < 3.0f ? values[0] : values[0] / 100.0f, values[1] };
    if (!finite(body.heightMeters) || !finite(body.massKg) || clampedBody3D(body) != body) return std::nullopt;
    return body;
}

float bodyScale3D(const PlayerBody3D& body) {
    if (body.heightMeters == BaseBodyHeightMeters) return 1.0f;
    return body.heightMeters / BaseBodyHeightMeters;
}

float bodyToneVolume3D(float tone) {
    return tone >= 0.0f ? sideVolume(HeavyVolume, tone) : sideVolume(LightVolume, -tone);
}

float bodyTone3D(const PlayerBody3D& body) {
    if (isBaseBody3D(body)) return 0.0f;
    const float s = bodyScale3D(body);
    // O expoente da altura: 3 nos baixos (o mesmo fisico em escala) e 2 nos altos
    // (o peso sobre a altura ao quadrado, como o indice de massa corporal; 26/10,
    // pedido dele: os altos "mesmo com mais peso acabam ficando muito franzinos
    // desproporcionalmente" - com o 3, o de 2,00 m e 100 kg dava -0,7; com o 2,
    // ~0, o fisico do de hoje).
    const float volume = body.massKg / (BaseBodyMassKg * s * s * (s > 1.0f ? 1.0f : s));
    if (volume == 1.0f) return 0.0f;
    // O cubico ao contrario (Newton; ele e monotono na faixa).
    const std::array<float, 3>& c = volume > 1.0f ? HeavyVolume : LightVolume;
    float u = (volume - 1.0f) / c[0];
    for (int i = 0; i < 8; ++i)
        u -= (sideVolume(c, u) - volume) / (c[0] + u * (2.0f * c[1] + 3.0f * u * c[2]));
    if (volume > 1.0f) return u > HeavyKnee ? HeavyKnee + 0.5f * (u - HeavyKnee) : u;
    return -(u > LightKnee ? LightKnee + (u - LightKnee) / 3.0f : u);
}

std::optional<PhysiqueRegion3D> physiqueRegionOfLink3D(std::string_view linkId) {
    if (linkId.rfind("Left", 0) == 0) linkId.remove_prefix(4);
    else if (linkId.rfind("Right", 0) == 0) linkId.remove_prefix(5);
    static constexpr std::array<std::string_view, static_cast<std::size_t>(PhysiqueRegion3D::Count)> Names {
        "Pelvis", "Abdomen", "Chest", "UpperChest", "Neck", "Head",
        "UpperArm", "Forearm", "Hand", "Thigh", "Shin", "Foot"
    };
    for (std::size_t i = 0; i < Names.size(); ++i)
        if (Names[i] == linkId) return static_cast<PhysiqueRegion3D>(i);
    return std::nullopt;
}

float bodyGirth3D(PhysiqueRegion3D region, float tone) {
    const std::size_t index = static_cast<std::size_t>(region);
    if (index >= Girth.size() || tone == 0.0f) return 1.0f;
    const GirthRatios& ratios = Girth[index];
    return tone > 0.0f ? 1.0f + tone * (ratios.heavy - 1.0f) : 1.0f + tone * (1.0f - ratios.light);
}

float bodyGirthDepth3D(PhysiqueRegion3D region, float tone) {
    // A cintura do franzino so afina dos lados: a barriga recuando dobrava ao
    // avesso no agachamento do levantar (a folha so mostra a largura). (No
    // encorpado a barriga crescendo menos para a frente dobrava igual.)
    if (region == PhysiqueRegion3D::Abdomen && tone < 0.0f) return 1.0f;
    return bodyGirth3D(region, tone);
}

float bodyGirthBack3D(PhysiqueRegion3D region, float tone) {
    const float depth = bodyGirthDepth3D(region, tone);
    if (tone > 0.0f && (region == PhysiqueRegion3D::Chest || region == PhysiqueRegion3D::UpperChest || region == PhysiqueRegion3D::Neck))
        return 1.0f + 0.35f * (depth - 1.0f);
    return depth;
}

float bodyArmOpening3D(float tone) {
    // 7 graus por unidade de tonus (~10 no canto de 1,70 m e 100 kg); o resto
    // e o colisor que encolhe na ponta.
    return tone > 0.0f ? tone * 7.0f * 3.14159265f / 180.0f : 0.0f;
}

BodyMotorScale3D bodyMotorScale3D(const PlayerBody3D& body) {
    if (isBaseBody3D(body)) return {};
    const float m = body.massKg / BaseBodyMassKg, s = bodyScale3D(body);
    return { m * s, m * s * s, m * s * s };
}

} // namespace MatterEngine
