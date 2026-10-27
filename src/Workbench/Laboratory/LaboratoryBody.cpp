#include "Workbench/Laboratory/LaboratoryBody.hpp"

#include <algorithm>
#include <cmath>

namespace MatterEngine::Workbench {

const LaboratoryBody* LaboratoryBodyCache::select(const RagdollCharacter3D& base,
    const std::vector<AnimationClip3D>& baseClips, float heightCm, float massKg) {
    const int cm = static_cast<int>(std::lround(
        std::clamp(heightCm, MinimumBodyHeightMeters * 100.0f, MaximumBodyHeightMeters * 100.0f)));
    const int kg = static_cast<int>(std::lround(std::clamp(massKg, MinimumBodyMassKg, MaximumBodyMassKg)));
    // (cm / 100 com a divisao: 180 da exatamente o 1,80f do corpo de hoje.)
    const PlayerBody3D body { static_cast<float>(cm) / 100.0f, static_cast<float>(kg) };
    if (isBaseBody3D(body)) return nullptr;
    const std::pair<int, int> key { cm, kg };
    if (const auto found = m_bodies.find(key); found != m_bodies.end()) return &found->second;
    // O tonus da malha: uma vez, do personagem de hoje (so no primeiro corpo).
    if (m_tone.empty()) m_tone = computeRagdollToneDeltas3D(base);
    LaboratoryBody made;
    made.body = body;
    made.character = bodyRagdollCharacter3D(base, body, &m_tone,
        ragdollClearanceClips3D(base, baseClips));
    made.clips.reserve(baseClips.size());
    for (const AnimationClip3D& clip : baseClips) made.clips.push_back(bodyRagdollClip3D(clip, base.profile, body));
    return &m_bodies.emplace(key, std::move(made)).first->second;
}

} // namespace MatterEngine::Workbench
