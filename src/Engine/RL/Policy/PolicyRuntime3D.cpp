#include "Engine/RL/Policy/PolicyRuntime3D.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iterator>
#include <stdexcept>

namespace MatterEngine {

PolicyRuntime3D::PolicyRuntime3D(const std::filesystem::path& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("politica ausente: " + path.string());
    const std::vector<char> bytes { std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>() };
    constexpr char Magic[8] = { 'M', 'R', 'L', 'P', 'O', 'L', '0', '1' };
    if (bytes.size() < 12 || std::memcmp(bytes.data(), Magic, 8) != 0)
        throw std::runtime_error("politica: arquivo nao e .mrlpolicy");
    std::uint32_t headerSize = 0;
    std::memcpy(&headerSize, bytes.data() + 8, 4);
    if (12u + headerSize > bytes.size()) throw std::runtime_error("politica: cabecalho truncado");
    const nlohmann::json header = nlohmann::json::parse(bytes.begin() + 12, bytes.begin() + 12 + headerSize);
    if (header.value("schema", std::string()) != "matter-rl-policy-1")
        throw std::runtime_error("politica: schema desconhecido");
    std::size_t dataStart = 12u + headerSize;
    dataStart = (dataStart + 3u) & ~std::size_t { 3u };
    if (dataStart > bytes.size() || (bytes.size() - dataStart) % 4u != 0)
        throw std::runtime_error("politica: dados desalinhados");
    m_data.resize((bytes.size() - dataStart) / 4u);
    std::memcpy(m_data.data(), bytes.data() + dataStart, m_data.size() * 4u);

    m_observationSize = header.at("observationSize").get<std::size_t>();
    m_actionSize = header.at("actionSize").get<std::size_t>();
    if (header.value("activation", std::string("elu")) != "elu") throw std::runtime_error("politica: ativacao nao suportada");
    const auto& normalizer = header.at("normalizer");
    m_mean = normalizer.at("mean").get<std::size_t>();
    m_inverseStd = normalizer.at("invStd").get<std::size_t>();
    m_clip = normalizer.value("clip", 5.0f);
    m_actionLimit = header.value("actionLimit", 10.0f);
    const auto inside = [&](std::size_t offset, std::size_t count) { return offset + count <= m_data.size(); };
    if (!inside(m_mean, m_observationSize) || !inside(m_inverseStd, m_observationSize))
        throw std::runtime_error("politica: normalizacao fora do arquivo");
    std::size_t previous = m_observationSize;
    std::size_t widest = 0;
    for (const auto& item : header.at("layers")) {
        Layer layer;
        layer.inputs = item.at("inputs").get<std::size_t>();
        layer.outputs = item.at("outputs").get<std::size_t>();
        layer.weights = item.at("weights").get<std::size_t>();
        layer.bias = item.at("bias").get<std::size_t>();
        if (layer.inputs != previous) throw std::runtime_error("politica: camadas encadeadas errado");
        if (!inside(layer.weights, layer.inputs * layer.outputs) || !inside(layer.bias, layer.outputs))
            throw std::runtime_error("politica: camada fora do arquivo");
        previous = layer.outputs;
        widest = std::max(widest, layer.outputs);
        m_layers.push_back(layer);
    }
    if (m_layers.empty() || previous != m_actionSize) throw std::runtime_error("politica: saida diferente da acao");
    for (const float value : m_data)
        if (!std::isfinite(value)) throw std::runtime_error("politica: pesos nao finitos");

    m_info.observationLayout = header.value("observationLayout", std::string());
    m_info.observationVersion = header.value("observationVersion", 0u);
    m_info.actionLayout = header.value("actionLayout", std::string());
    m_info.bodyId = header.value("bodyId", std::string());
    m_info.bodyHash = header.value("bodyHash", std::string());
    m_info.taskId = header.value("taskId", std::string());
    m_info.run = header.value("run", std::string());
    m_info.iteration = header.value("iteration", std::uint64_t { 0 });
    m_info.steps = header.value("steps", std::uint64_t { 0 });
    m_info.createdUtc = header.value("createdUtc", std::string());
    m_scratchA.resize(std::max(widest, m_observationSize));
    m_scratchB.resize(std::max(widest, m_observationSize));
}

void PolicyRuntime3D::evaluate(std::span<const float> observation, std::span<float> action) const {
    if (observation.size() != m_observationSize || action.size() != m_actionSize)
        throw std::invalid_argument("politica: dimensoes erradas");
    float* in = m_scratchA.data();
    float* out = m_scratchB.data();
    for (std::size_t i = 0; i < m_observationSize; ++i)
        in[i] = std::clamp((observation[i] - m_data[m_mean + i]) * m_data[m_inverseStd + i], -m_clip, m_clip);
    for (std::size_t l = 0; l < m_layers.size(); ++l) {
        const Layer& layer = m_layers[l];
        const float* w = m_data.data() + layer.weights;
        const float* b = m_data.data() + layer.bias;
        const bool hidden = l + 1 < m_layers.size();
        for (std::size_t o = 0; o < layer.outputs; ++o) {
            const float* row = w + o * layer.inputs;
            float sum = b[o];
            for (std::size_t i = 0; i < layer.inputs; ++i) sum += row[i] * in[i];
            out[o] = hidden ? (sum > 0.0f ? sum : std::expm1(sum)) : sum;
        }
        std::swap(in, out);
    }
    for (std::size_t a = 0; a < m_actionSize; ++a) action[a] = std::clamp(in[a], -m_actionLimit, m_actionLimit);
}

} // namespace MatterEngine
