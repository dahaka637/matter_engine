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
    const std::string activation = header.value("activation", std::string("elu"));
    if (activation != "elu" && activation != "tanh" && activation != "relu")
        throw std::runtime_error("politica: ativacao nao suportada");
    m_tanh = activation == "tanh";
    m_relu = activation == "relu";
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
        layer.transposed = m_transposed.size();
        m_transposed.resize(m_transposed.size() + layer.inputs * layer.outputs);
        for (std::size_t o = 0; o < layer.outputs; ++o)
            for (std::size_t i = 0; i < layer.inputs; ++i)
                m_transposed[layer.transposed + i * layer.outputs + o] = m_data[layer.weights + o * layer.inputs + i];
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
    if (header.contains("observationNames")) m_info.observationNames = header["observationNames"].get<std::vector<std::string>>();
    if (header.contains("explorationStd")) {
        m_info.explorationStd = header.at("explorationStd").get<std::vector<float>>();
        if (m_info.explorationStd.size() != m_actionSize ||
            !std::all_of(m_info.explorationStd.begin(), m_info.explorationStd.end(),
                         [](float v) { return std::isfinite(v) && v > 0.f && v <= std::exp(1.5f); }))
            throw std::runtime_error("politica: ruido exploratorio invalido");
    }
    m_scratchA.resize(std::max(widest, m_observationSize));
    m_scratchB.resize(std::max(widest, m_observationSize));
}

namespace {

// Uma camada densa em blocos de saidas: cada bloco de 16 fica em registradores
// enquanto percorre todas as entradas (antes, cada entrada lia e escrevia as
// 512 saidas). A soma de cada saida segue a mesma ordem: o vies, depois as
// entradas em ordem crescente.
inline void denseLayer(const float* w, const float* b, const float* in, float* out,
    std::size_t inputs, std::size_t outputs) {
    constexpr std::size_t Block = 16;
    std::size_t o = 0;
    for (; o + Block <= outputs; o += Block) {
        float acc[Block];
        for (std::size_t k = 0; k < Block; ++k) acc[k] = b[o + k];
        for (std::size_t i = 0; i < inputs; ++i) {
            const float x = in[i];
            const float* row = w + i * outputs + o;
            for (std::size_t k = 0; k < Block; ++k) acc[k] += row[k] * x;
        }
        for (std::size_t k = 0; k < Block; ++k) out[o + k] = acc[k];
    }
    for (; o < outputs; ++o) {
        float acc = b[o];
        for (std::size_t i = 0; i < inputs; ++i) acc += w[i * outputs + o] * in[i];
        out[o] = acc;
    }
}

} // namespace

void PolicyRuntime3D::evaluate(std::span<const float> observation, std::span<float> action) const {
    if (observation.size() != m_observationSize || action.size() != m_actionSize)
        throw std::invalid_argument("politica: dimensoes erradas");
    // (Rascunho por thread: a mesma politica pode servir varios atletas em
    // paralelo.)
    thread_local std::vector<float> scratchA, scratchB;
    if (scratchA.size() < m_scratchA.size()) scratchA.resize(m_scratchA.size());
    if (scratchB.size() < m_scratchB.size()) scratchB.resize(m_scratchB.size());
    float* in = scratchA.data();
    float* out = scratchB.data();
    for (std::size_t i = 0; i < m_observationSize; ++i)
        in[i] = std::clamp((observation[i] - m_data[m_mean + i]) * m_data[m_inverseStd + i], -m_clip, m_clip);
    for (std::size_t l = 0; l < m_layers.size(); ++l) {
        const Layer& layer = m_layers[l];
        const bool hidden = l + 1 < m_layers.size();
        const std::size_t outputs = layer.outputs;
        denseLayer(m_transposed.data() + layer.transposed, m_data.data() + layer.bias, in, out, layer.inputs, outputs);
        if (hidden && m_tanh)
            for (std::size_t o = 0; o < outputs; ++o) out[o] = std::tanh(out[o]);
        else if (hidden && m_relu)
            for (std::size_t o = 0; o < outputs; ++o) out[o] = std::max(0.0f, out[o]);
        else if (hidden)
            for (std::size_t o = 0; o < outputs; ++o) out[o] = out[o] > 0.0f ? out[o] : std::expm1(out[o]);
        std::swap(in, out);
    }
    for (std::size_t a = 0; a < m_actionSize; ++a) action[a] = std::clamp(in[a], -m_actionLimit, m_actionLimit);
}

void PolicyRuntime3D::evaluateBatch(std::span<const float> observations, std::span<float> actions,
    std::size_t count) const {
    if (observations.size() != count * m_observationSize || actions.size() != count * m_actionSize)
        throw std::invalid_argument("politica: dimensoes erradas no lote");
    std::size_t widest = m_observationSize;
    for (const Layer& layer : m_layers) widest = std::max(widest, layer.outputs);
    thread_local std::vector<float> batchA, batchB;
    if (batchA.size() < count * widest) batchA.resize(count * widest);
    if (batchB.size() < count * widest) batchB.resize(count * widest);
    float* in = batchA.data();
    float* out = batchB.data();
    std::size_t width = m_observationSize;
    for (std::size_t n = 0; n < count; ++n)
        for (std::size_t i = 0; i < m_observationSize; ++i)
            in[n * width + i] = std::clamp((observations[n * m_observationSize + i] - m_data[m_mean + i])
                    * m_data[m_inverseStd + i], -m_clip, m_clip);
    for (std::size_t l = 0; l < m_layers.size(); ++l) {
        const Layer& layer = m_layers[l];
        const float* w = m_transposed.data() + layer.transposed;
        const float* b = m_data.data() + layer.bias;
        const bool hidden = l + 1 < m_layers.size();
        const std::size_t outputs = layer.outputs;
        for (std::size_t n = 0; n < count; ++n)
            for (std::size_t o = 0; o < outputs; ++o) out[n * outputs + o] = b[o];
        // Cada coluna de pesos (as saidas de uma entrada) fica no cache
        // enquanto passa pelos jogadores.
        for (std::size_t i = 0; i < layer.inputs; ++i) {
            const float* column = w + i * outputs;
            for (std::size_t n = 0; n < count; ++n) {
                const float x = in[n * width + i];
                float* target = out + n * outputs;
                for (std::size_t o = 0; o < outputs; ++o) target[o] += column[o] * x;
            }
        }
        if (hidden && m_tanh)
            for (std::size_t k = 0; k < count * outputs; ++k) out[k] = std::tanh(out[k]);
        else if (hidden && m_relu)
            for (std::size_t k = 0; k < count * outputs; ++k) out[k] = std::max(0.0f, out[k]);
        else if (hidden)
            for (std::size_t k = 0; k < count * outputs; ++k) out[k] = out[k] > 0.0f ? out[k] : std::expm1(out[k]);
        std::swap(in, out);
        width = outputs;
    }
    for (std::size_t n = 0; n < count; ++n)
        for (std::size_t a = 0; a < m_actionSize; ++a)
            actions[n * m_actionSize + a] = std::clamp(in[n * width + a], -m_actionLimit, m_actionLimit);
}

} // namespace MatterEngine
