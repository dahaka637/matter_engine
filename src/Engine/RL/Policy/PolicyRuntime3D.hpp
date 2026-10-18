#pragma once

#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// Uma politica exportada pelo treinador (arquivo .mrlpolicy): a MLP do ator
// (ELU nas camadas escondidas, saida linear = a media da acao) e a
// normalizacao da observacao. E o mesmo calculo do PyTorch, com teste de
// paridade; nao depende de fisica nem do treinador.
//
// Formato: "MRLPOL01", uint32 com o tamanho do cabecalho JSON, o cabecalho,
// enchimento ate multiplo de 4 bytes e os floats (little-endian). Os pesos de
// cada camada sao [saidas][entradas].
struct PolicyInfo3D {
    std::string observationLayout;
    std::uint32_t observationVersion = 0;
    std::string actionLayout;
    std::string bodyId;
    std::string bodyHash;
    std::string taskId;
    std::string run;
    std::uint64_t iteration = 0;
    std::uint64_t steps = 0;
    std::string createdUtc;
    // O nome de cada entrada, na ordem (vazio nos arquivos de antes): com eles,
    // quem usa a politica monta a observacao pelos nomes, e uma politica serve a
    // uma tarefa com outras entradas (as que faltam entram com zero, como na
    // migracao do treinador).
    std::vector<std::string> observationNames;
};

class PolicyRuntime3D final {
public:
    // Lanca std::runtime_error com a razao se o arquivo nao serve.
    explicit PolicyRuntime3D(const std::filesystem::path& path);

    [[nodiscard]] std::size_t observationSize() const { return m_observationSize; }
    [[nodiscard]] std::size_t actionSize() const { return m_actionSize; }
    [[nodiscard]] const PolicyInfo3D& info() const { return m_info; }

    // A acao media (sem ruido) para uma observacao; cada canal limitado como o
    // ambiente faz (actionLimit do arquivo, 10 por padrao).
    void evaluate(std::span<const float> observation, std::span<float> action) const;
    // Varios jogadores numa conta so (o jogo: os 22 juntos): count observacoes
    // seguidas em observations e count acoes em actions. Os pesos sao lidos
    // uma vez por chamada, nao uma por jogador (a conta de um jogador e
    // limitada pela memoria: ~4,9 MB de pesos com a rede do plano).
    void evaluateBatch(std::span<const float> observations, std::span<float> actions, std::size_t count) const;

private:
    struct Layer {
        std::size_t inputs = 0;
        std::size_t outputs = 0;
        std::size_t weights = 0;
        std::size_t bias = 0;
        // Os pesos transpostos ([entradas][saidas], em m_transposed): o laco de
        // dentro anda pelas saidas, cada uma com a sua soma, e vetoriza sem
        // reordenar somas de float (a conta por linha nao vetoriza: 22
        // jogadores com a rede do plano custavam ~58% de um nucleo).
        std::size_t transposed = 0;
    };

    std::vector<float> m_data;
    std::vector<float> m_transposed;
    std::vector<Layer> m_layers;
    std::size_t m_mean = 0;
    std::size_t m_inverseStd = 0;
    float m_clip = 5.0f;
    float m_actionLimit = 10.0f;
    // A ativacao das camadas escondidas: ELU (o padrao, as redes do Corpo e da
    // locomocao) ou tanh (o rastreador, DReCon 6.6).
    bool m_tanh = false;
    std::size_t m_observationSize = 0;
    std::size_t m_actionSize = 0;
    PolicyInfo3D m_info;
    mutable std::vector<float> m_scratchA;
    mutable std::vector<float> m_scratchB;
    mutable std::vector<float> m_batchA;
    mutable std::vector<float> m_batchB;
};

} // namespace MatterEngine
