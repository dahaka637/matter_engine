#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace MatterEngine::Modeling {

enum class Side : std::uint8_t { Center, Left, Right };

// Body: os ossos fisicos do jogo; Finger/Toe: os visuais (dedos); Tip: a
// ponta de um osso sem filho (alto da cabeca, ponta dos dedos), so para
// desenhar o osso ate o fim; Helper: uma junta auxiliar do skin, com a pose
// tirada de duas outras (o joelho girando pela metade), sem osso proprio.
enum class JointKind : std::uint8_t { Body, Finger, Toe, Tip, Helper };

[[nodiscard]] std::string_view sideName(Side side);
[[nodiscard]] std::string_view jointKindName(JointKind kind);

struct Joint {
    std::string name;
    int parent = -1;
    // Referencial da junta na pose de ligacao (espaco do modelo).
    Transform bind;
    JointKind kind = JointKind::Body;
    Side side = Side::Center;
    // Raio aproximado do corpo em volta do osso que sai desta junta: o
    // enquadramento das partes enquanto nao ha malha.
    float radius = 0.03f;
    // Auxiliar (JointKind::Helper): gira "derivedFraction" do giro de
    // "derivedTo" em relacao a "derivedFrom", em volta da junta derivedTo
    // (a frente do joelho fica numa junta que dobra a metade: sem perder o
    // volume da quina).
    int derivedFrom = -1;
    int derivedTo = -1;
    float derivedFraction = 0.5f;
    // Com eixo (no espaco do modelo), so a parte do giro em volta dele (a
    // flexao do quadril: a coxa que abre de lado ou torce nao mexe nela).
    Vec3 derivedAxis {};
};

class Skeleton {
public:
    // Pais sempre antes dos filhos.
    std::vector<Joint> joints;
    // Altura do chao no espaco do modelo (os pes apoiados na ligacao).
    float groundHeight = 0.0f;

    [[nodiscard]] int find(std::string_view name) const;
    [[nodiscard]] std::vector<int> children(int joint) const;
    [[nodiscard]] Box3 bounds(float padding = 0.0f) const;
    // Do chao ao ponto mais alto.
    [[nodiscard]] float height() const;
    // Do pai ate a junta (zero na raiz).
    [[nodiscard]] float boneLength(int joint) const;
    // Problemas de estrutura: pai depois do filho, nomes repetidos.
    [[nodiscard]] std::vector<std::string> validate() const;
};

// A pose: o referencial de cada junta no espaco do modelo.
struct SkeletonPose {
    std::vector<Transform> joints;
};

[[nodiscard]] SkeletonPose bindPose(const Skeleton& skeleton);
// Uma junta auxiliar entre "from" (a de cima) e "to" (a de baixo): a pose
// dela vem das duas (updateDerivedJoints); com "axis", so do giro em volta
// dele. Devolve o indice.
int addHelperJoint(Skeleton& skeleton, std::string name, int from, int to, float fraction, Vec3 axis = {});
// Recalcula as juntas auxiliares de uma pose (depois de todas as outras).
void updateDerivedJoints(const Skeleton& skeleton, SkeletonPose& pose);

// O eixo de flexao de um dedo no referencial dado (o da junta): o -y do osso
// (os dedos, como no jogo); no polegar, o meio entre o -y e o z (o polegar e
// girado em relacao aos dedos: em volta desse eixo ele fecha atravessando a
// palma, rente a ela, em vez de ir para baixo dela). Na mao esquerda (o
// espelho da direita: o y do osso troca de sinal, o z nao) o polegar usa o
// -z.
[[nodiscard]] Vec3 fingerFlexionAxis(const Quaternion& frame, bool thumb, bool left = false);

// A dobradica de uma junta na pose (o eixo em volta do qual ela fecha, no
// espaco do modelo): nos dedos, fingerFlexionAxis; nas outras, a normal do
// plano do membro (o osso de cima e o de baixo: o cotovelo fecha o antebraco
// para o braco) ou, com o membro quase reto, a do plano dele com o x (o
// joelho leva a canela para tras, o cotovelo leva o antebraco para a frente).
[[nodiscard]] Vec3 jointHingeAxis(const Skeleton& skeleton, const SkeletonPose& pose, int joint);

// Uma junta girada em volta de um eixo do modelo (radianos). Com
// "followParent", o eixo e o da pose de partida e gira junto com o pai (o
// cotovelo dobra no plano do braco mesmo com o braco levantado; os dedos
// fecham para a palma com o pulso dobrado).
struct JointBend {
    int joint = -1;
    Vec3 axis { 0.0f, 1.0f, 0.0f };
    float radians = 0.0f;
    bool followParent = false;
};
// A ligacao com as juntas giradas (cada uma leva os filhos; de cima para
// baixo na arvore) e as auxiliares recalculadas: o teste de dobrar o joelho,
// o quadril, o tornozelo.
[[nodiscard]] SkeletonPose bendPose(const Skeleton& skeleton, std::span<const JointBend> bends);
// O mesmo a partir de uma pose (a mao fechada do jogo, por exemplo).
[[nodiscard]] SkeletonPose bendPose(const Skeleton& skeleton, const SkeletonPose& start, std::span<const JointBend> bends);
// As juntas dos dedos de um lado (indicador, medio, anelar e minimo, da base
// a ponta; com "thumb", o polegar tambem).
[[nodiscard]] std::vector<int> fingerJoints(const Skeleton& skeleton, Side side, bool thumb);
// As dobras que levam cada junta dada (o giro dela no pai) da pose "from" a
// pose "to", na fracao (0 = from, 1 = to), presas ao pai: a mao fechando da
// partida ate o punho do jogo sem passar da palma.
[[nodiscard]] std::vector<JointBend> blendJointBends(const Skeleton& skeleton, const SkeletonPose& from, const SkeletonPose& to,
    std::span<const int> joints, float fraction);
// Pose vezes a inversa da ligacao, por junta (o skin linear).
[[nodiscard]] std::vector<Transform> skinningTransforms(const Skeleton& skeleton, const SkeletonPose& pose);

// Troca Left/Right (e esquerda/direita) no nome: o lado espelhado.
[[nodiscard]] std::string mirroredName(std::string_view name);
// A junta do outro lado (ou ela mesma, no centro).
[[nodiscard]] int mirroredJoint(const Skeleton& skeleton, int joint);

} // namespace MatterEngine::Modeling
