#pragma once

#include "Engine/Animation/MotionMatching/MotionMatchingSchema3D.hpp"
#include "Engine/Math/Vec3.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

// Um instante de uma pose: a raiz (o ponto no chao sob o corpo e o rumo, yaw em rad em torno de Z), os pontos (os pes, a
// pelve) e os eixos (unitarios: os da pelve), no mundo - na ordem que os esquemas usam.
struct MotionMatchingPose3D {
    Vec3 rootPosition;
    float rootYaw = 0.0f;
    std::vector<Vec3> points;
    std::vector<Vec3> axes;
};

// Uma fonte do banco: um clipe com o deslocamento da raiz, amostrado em [0, durationSeconds].
class MotionMatchingSource3D {
public:
    virtual ~MotionMatchingSource3D() = default;
    [[nodiscard]] virtual float durationSeconds() const = 0;
    [[nodiscard]] virtual bool loops() const = 0;
    virtual void sample(float seconds, MotionMatchingPose3D& out) const = 0;
};

// Um ponto da trajetoria do personagem em volta de agora (offsetSeconds: negativo o passado, positivo a previsao).
struct MotionMatchingTrajectoryPoint3D {
    float offsetSeconds = 0.0f;
    Vec3 position;
    float yaw = 0.0f;
    Vec3 velocity;
};

// O personagem agora, para a consulta: a raiz, a velocidade dela (do mundo) e o giro (rad/s), os pontos, as velocidades deles
// (do mundo) e os eixos, e a trajetoria em ordem de tempo (o passado do historico, o futuro previsto); um instante fora dos
// pontos dados fica no da ponta (sem extrapolar, como o PoseSearch).
struct MotionMatchingQueryInput3D {
    Vec3 rootPosition;
    float rootYaw = 0.0f;
    Vec3 rootVelocity;
    float rootYawRate = 0.0f;
    std::vector<Vec3> points;
    std::vector<Vec3> pointVelocities;
    std::vector<Vec3> axes;
    std::vector<MotionMatchingTrajectoryPoint3D> trajectory;
};

// Um banco do conjunto (o PoseSearchDatabase): o esquema, as fontes e os vieses do custo.
struct MotionMatchingDatabaseGroup3D {
    std::string name;
    MotionMatchingSchema3D schema;
    std::vector<const MotionMatchingSource3D*> sources;
    float baseCostBias = 0.0f;
    float loopingCostBias = -0.005f;
    float continuingCostBias = -0.01f;
};

struct MotionMatchingEntry3D {
    std::size_t group = 0;
    std::size_t source = 0;
    float timeSeconds = 0.0f;
    float costAddend = 0.0f;
};

struct MotionMatchingResult3D {
    static constexpr std::size_t None = std::numeric_limits<std::size_t>::max();
    std::size_t index = None;
    float cost = std::numeric_limits<float>::infinity();
    bool continuing = false;
    [[nodiscard]] bool valid() const { return index != None; }
};

struct MotionMatchingSearchOptions3D {
    // Os grupos procurados (bit g = grupo g; o "AllResults" do chooser do Game Animation Sample), o quadro que ja toca (o
    // continuar, com o viés do grupo dele) e os que nao podem ser escolhidos (o historico de reescolha, 0,3 s).
    std::uint64_t groups = ~std::uint64_t { 0 };
    std::size_t continuingIndex = MotionMatchingResult3D::None;
    std::span<const std::size_t> excluded;
};

// O conjunto de bancos (vault: o PoseSearch da UE 5.8): cada fonte amostrada a sampleRate do esquema do grupo (as que nao
// repetem sem os ultimos excludeEndSeconds); fora do clipe, a raiz segue a velocidade da ponta (as que repetem continuam no
// ciclo seguinte). Os valores ficam crus; a normalizacao, comum a todos os grupos (o NormalizeWithCommonSchema), vai nos
// pesos: o peso da folha sobre a soma dos pesos das dimensoes do esquema dela, a raiz quadrada, dividido pelo desvio medio do
// rotulo (a media, em todos os quadros de todos os grupos, da norma da folha menos a media dela; 1 se quase zero). O custo e
// a soma dos quadrados das diferencas ponderadas mais os vieses - comparavel entre grupos.
class MotionMatchingDatabase3D {
public:
    explicit MotionMatchingDatabase3D(std::vector<MotionMatchingDatabaseGroup3D> groups);

    [[nodiscard]] std::size_t groupCount() const { return m_groups.size(); }
    [[nodiscard]] const MotionMatchingDatabaseGroup3D& group(std::size_t g) const { return m_groups[g].spec; }
    [[nodiscard]] std::size_t featureSize(std::size_t g) const { return m_groups[g].size; }
    // Os quadros de um grupo: [groupBegin, groupEnd) em entries().
    [[nodiscard]] std::size_t groupBegin(std::size_t g) const { return m_groups[g].first; }
    [[nodiscard]] std::size_t groupEnd(std::size_t g) const { return m_groups[g].last; }
    [[nodiscard]] std::size_t size() const { return m_entries.size(); }
    [[nodiscard]] const std::vector<MotionMatchingEntry3D>& entries() const { return m_entries; }
    [[nodiscard]] std::span<const float> features(std::size_t index) const;
    [[nodiscard]] std::span<const float> weightsSqrt(std::size_t g) const { return m_groups[g].weightsSqrt; }
    // Os valores ponderados (cru vezes o peso de cada dimensao do grupo): a distancia euclidiana ao quadrado entre dois
    // vetores ponderados e o custo sem os vieses (a busca em lote noutro lugar - a placa de video do treino).
    void weighted(std::size_t g, std::span<const float> raw, std::span<float> out) const;

    // O vetor de um instante de uma fonte de um grupo (o do banco) e o de um personagem agora no esquema de um grupo.
    void sourceFeatures(std::size_t g, std::size_t source, float seconds, std::span<float> out) const;
    void queryFeatures(std::size_t g, const MotionMatchingQueryInput3D& input, std::span<float> out) const;

    // O custo de um quadro com a consulta do grupo dele.
    [[nodiscard]] float cost(std::size_t index, std::span<const float> groupQuery) const;
    // A busca: queries[g] e a consulta no esquema do grupo g (so os procurados precisam estar preenchidos).
    [[nodiscard]] MotionMatchingResult3D search(std::span<const std::vector<float>> queries,
        const MotionMatchingSearchOptions3D& options = {}) const;
    // Os k mais baratos (sem o continuar) dos grupos pedidos, do mais barato ao mais caro.
    [[nodiscard]] std::vector<MotionMatchingResult3D> nearest(std::span<const std::vector<float>> queries, std::size_t k,
        std::uint64_t groups = ~std::uint64_t { 0 }) const;
    // Os quadros de uma fonte de um grupo: [first, second) em entries() (o "nao reescolher" do PoseSearch exclui todos eles
    // enquanto a fonte toca).
    [[nodiscard]] std::pair<std::size_t, std::size_t> sourceEntries(std::size_t g, std::size_t source) const {
        return { m_groups[g].firstEntry[source], m_groups[g].firstEntry[source + 1] };
    }
    // O quadro de uma fonte de um grupo num instante (round(t x sampleRate); no ciclo, dando a volta); None fora.
    [[nodiscard]] std::size_t entryAt(std::size_t g, std::size_t source, float seconds) const;

private:
    struct Group {
        MotionMatchingDatabaseGroup3D spec;
        std::vector<MotionMatchingSchema3D::Leaf> leaves;
        std::size_t size = 0;
        std::size_t first = 0, last = 0;
        std::vector<std::size_t> firstEntry;
        std::vector<float> values;
        std::vector<float> weightsSqrt;
    };
    void sampleExtended(const Group& g, std::size_t source, float seconds, MotionMatchingPose3D& out) const;

    std::vector<Group> m_groups;
    std::vector<MotionMatchingEntry3D> m_entries;
};

} // namespace MatterEngine
