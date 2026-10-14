#include "Engine/RL/Style/MotionDataset3D.hpp"

#include "Engine/Animation/AnimationClip3D.hpp"
#include "Engine/RL/Contract/Mirror3D.hpp"
#include "Engine/RL/Imitation/ReferenceMotion3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iterator>
#include <set>
#include <stdexcept>

namespace MatterEngine {
namespace {

[[noreturn]] void fail(const std::string& what) {
    throw std::runtime_error("acervo de estilo: " + what);
}

std::uint64_t fnv1a(const std::string& text, std::uint64_t hash) {
    for (const char c : text) {
        hash ^= static_cast<unsigned char>(c);
        hash *= 1099511628211ull;
    }
    return hash;
}

} // namespace

MotionDataset3D loadMotionCatalog3D(const std::filesystem::path& definitionPath) {
    std::ifstream file(definitionPath, std::ios::binary);
    if (!file) fail("nao abriu " + definitionPath.string());
    const std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    const nlohmann::json document = nlohmann::json::parse(text);
    if (document.value("schema", std::string()) != "matter-rl-style-1") fail("schema desconhecido em " + definitionPath.string());

    MotionDataset3D dataset;
    dataset.id = document.at("id").get<std::string>();
    dataset.definitionPath = definitionPath;
    dataset.pairFrames = document.value("pairFrames", dataset.pairFrames);
    dataset.mirror = document.value("mirror", false);
    if (dataset.pairFrames == 0) fail("pairFrames precisa ser positivo");
    const std::filesystem::path clips =
        (definitionPath.parent_path() / document.value("clipsDirectory", std::string("../../animations/clips")))
            .lexically_normal();
    dataset.hash = fnv1a(text, 14695981039346656037ull);
    std::set<std::string> seen;
    for (const nlohmann::json& g : document.at("groups")) {
        MotionGroup3D group;
        group.id = g.at("id").get<std::string>();
        group.displayName = g.value("displayName", group.id);
        group.weight = g.value("weight", 1.0f);
        if (group.weight < 0.0f) fail("peso negativo no grupo " + group.id);
        const auto groupIndex = static_cast<std::uint32_t>(dataset.groups.size());
        dataset.groups.push_back(group);
        std::set<std::string> reserved;
        for (const nlohmann::json& r : g.value("reserved", nlohmann::json::array())) reserved.insert(r.get<std::string>());
        for (const nlohmann::json& c : g.at("clips")) {
            MotionClip3D clip;
            clip.id = c.get<std::string>();
            if (!seen.insert(clip.id).second) fail("clipe repetido: " + clip.id);
            clip.group = groupIndex;
            clip.path = clips / (clip.id + ".matteranim.json");
            clip.reserved = reserved.erase(clip.id) > 0;
            clip.loops = loadAnimationClip3D(clip.path.string()).loops;
            dataset.clips.push_back(std::move(clip));
        }
        if (!reserved.empty()) fail("reservado fora da lista do grupo " + group.id + ": " + *reserved.begin());
    }
    if (dataset.clips.empty()) fail("sem clipes");
    return dataset;
}

MotionDataset3D loadMotionDataset3D(const RLBody3D& body, const StyleFeatures3D& features,
    const std::filesystem::path& definitionPath, float heightScale) {
    MotionDataset3D dataset = loadMotionCatalog3D(definitionPath);
    dataset.featureSize = features.size();
    const std::size_t F = dataset.featureSize;
    std::uint64_t hash = dataset.hash;
    std::vector<float> frameFeatures;
    std::vector<PhysicsBodyState3D> links;
    // O espelho das medidas (o mesmo mapa do juiz para os dois instantes).
    MirrorChannelMap3D mirrorMap;
    if (dataset.mirror) mirrorMap = Mirror3D(body).layoutMap(features.layout());
    std::vector<float> mirrored(F);
    for (std::size_t c = 0; c < dataset.clips.size(); ++c) {
        MotionClip3D& clip = dataset.clips[c];
        const ReferenceClip3D request { clip.id, clip.path, clip.loops };
        // (Como biblioteca de um trecho: um clipe que comeca e termina tambem vale.)
        const ReferenceMotion3D motion =
            buildReferenceMotion3D(body, std::span<const ReferenceClip3D>(&request, 1), 1.0f / 120.0f, true);
        hash ^= motion.hash;
        hash *= 1099511628211ull;
        const ReferenceSegment3D& segment = motion.segments.front();
        clip.frames = static_cast<std::uint32_t>(segment.frames.size());
        clip.meanSpeed = segment.meanSpeed;
        // As medidas de cada quadro, pelo estado dos links (a mesma conta do corpo).
        frameFeatures.assign(static_cast<std::size_t>(clip.frames) * F, 0.0f);
        for (std::uint32_t f = 0; f < clip.frames; ++f) {
            referenceLinkStates3D(body, segment.frames[f], {}, segment.frames[f].heading, links);
            features.write(links, heightScale, frameFeatures.data() + static_cast<std::size_t>(f) * F);
        }
        clip.firstPair = static_cast<std::uint32_t>(dataset.pairCount());
        for (std::uint32_t f = 0; f < clip.frames; ++f) {
            std::uint32_t next = f + dataset.pairFrames;
            if (next >= clip.frames) {
                if (!clip.loops) break;
                next %= clip.frames;
            }
            const float* a = frameFeatures.data() + static_cast<std::size_t>(f) * F;
            const float* b = frameFeatures.data() + static_cast<std::size_t>(next) * F;
            dataset.pairs.insert(dataset.pairs.end(), a, a + F);
            dataset.pairs.insert(dataset.pairs.end(), b, b + F);
            dataset.pairClip.push_back(static_cast<std::uint32_t>(c));
            dataset.pairMirrored.push_back(0);
        }
        if (dataset.mirror) {
            const std::size_t first = clip.firstPair, last = dataset.pairCount();
            for (std::size_t p = first; p < last; ++p) {
                for (std::size_t half = 0; half < 2; ++half) {
                    mirrorMap.apply(dataset.pairs.data() + (p * 2 + half) * F, mirrored.data());
                    dataset.pairs.insert(dataset.pairs.end(), mirrored.begin(), mirrored.end());
                }
                dataset.pairClip.push_back(static_cast<std::uint32_t>(c));
                dataset.pairMirrored.push_back(1);
            }
        }
        clip.pairCount = static_cast<std::uint32_t>(dataset.pairCount()) - clip.firstPair;
        if (clip.pairCount == 0) fail("o clipe " + clip.id + " e curto demais para um par");
    }

    // O peso: por grupo, dividido pelos pares de treino dele.
    std::vector<std::size_t> groupPairs(dataset.groups.size(), 0);
    for (const MotionClip3D& clip : dataset.clips)
        if (!clip.reserved) groupPairs[clip.group] += clip.pairCount;
    double total = 0.0;
    for (std::size_t g = 0; g < dataset.groups.size(); ++g)
        if (groupPairs[g] > 0) total += dataset.groups[g].weight;
    if (!(total > 0.0)) fail("nenhum grupo com peso e clipes de treino");
    dataset.pairWeight.assign(dataset.pairCount(), 0.0f);
    for (const MotionClip3D& clip : dataset.clips) {
        if (clip.reserved) continue;
        const double weight = dataset.groups[clip.group].weight / total / static_cast<double>(groupPairs[clip.group]);
        for (std::uint32_t k = 0; k < clip.pairCount; ++k) dataset.pairWeight[clip.firstPair + k] = static_cast<float>(weight);
    }
    dataset.hash = hash;
    return dataset;
}

} // namespace MatterEngine
