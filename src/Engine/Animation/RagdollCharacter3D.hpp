#pragma once

#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"
#include "Engine/Geometry/MeshData3D.hpp"
#include "Engine/Math/Mat4.hpp"
#include "Engine/Physics/RagdollProfile3D.hpp"
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace MatterEngine {

// Osso visual (os dedos da mao e os do pe): segue um osso pai e nao tem
// fisica. A pose de ligacao fica guardada em relacao ao pai; restRotation
// dobra o osso na pose de repouso (a mao relaxada, semifechada; os dedos do pe
// retos). Animar - o goleiro, no futuro, e os dedos do pe no chao - e trocar
// essas rotacoes (buildRagdollSkinMatrices3D, RagdollVisualPose3D).
struct RagdollVisualBone3D {
    enum class Kind : std::uint8_t { Other, Finger, Toe };
    std::string name;
    std::size_t parentBone = 0; // na paleta: fisicos primeiro, depois visuais
    Vec3 localBindPosition;
    Quaternion localBindOrientation;
    Quaternion restRotation;
    // O que ele e (07/10): um dedo da mao - finger 0 polegar a 4 minimo, joint
    // 1 a 3 (da base a ponta); a pose relaxada: a flexao (rad, em volta de -y
    // do osso) e, na base, a fracao close do angulo closeAngle (rad, em volta
    // do z do osso) ate o dedo de referencia - ou os dedos do pe (length: da
    // base dos dedos a ponta, m). side 0 esquerdo, 1 direito.
    Kind kind = Kind::Other;
    int side = 0;
    int finger = 0;
    int joint = 0;
    float flexion = 0.0f;
    float close = 0.0f;
    float closeAngle = 0.0f;
    float length = 0.0f;
};

// Osso auxiliar do skin (o joelho, o cotovelo, o pulso...): gira a fracao do
// giro do osso de baixo em relacao ao de cima, em volta do pivo de verdade
// desse giro - a quina nao perde volume. Sem fisica nem animacao: sai dos dois
// ossos da paleta a cada pose (buildRagdollSkinMatrices3D). Vem na paleta
// depois dos visuais.
struct RagdollHelperBone3D {
    std::string name;
    std::size_t upperBone = 0;
    std::size_t lowerBone = 0;
    float fraction = 0.5f;
    // O pivo quando o giro e quase nulo (no espaco da malha: a junta de baixo
    // na ligacao da malha).
    Vec3 pivot;
    // Com eixo (no espaco da malha), so a parte do giro em volta dele (a
    // flexao do quadril); sem, o giro inteiro. sign: so um sentido desse giro
    // (+1 o positivo, -1 o negativo; 0 os dois).
    Vec3 axis;
    float sign = 0.0f;
    // A clavicula: so o quanto o osso de baixo (axis, a direcao dele na
    // ligacao) sobe acima da ligacao, em volta do eixo frente-tras, em volta
    // do pivo (fixo: a junta esterno-clavicular); e leva junto os ossos da
    // paleta em "carried" (o braco) - a mesma da modelagem (derivedCarry).
    bool carry = false;
    std::vector<std::size_t> carried;
    // O teto suave da subida (radianos; 0 sem teto) - o da modelagem (derivedCap).
    float cap = 0.0f;
    // A junta do ombro na ligacao (no espaco da malha): o ponto que a clavicula
    // leva; sem ela, a origem do osso de baixo.
    std::optional<Vec3> root;
    // A rampa da fracao com o osso de baixo erguido (rampAlong, a direcao dele
    // na ligacao; radianos) - a da modelagem (derivedRamp*); rampFraction 0 sem.
    float rampFraction = 0.0f;
    float rampStart = 0.0f;
    float rampEnd = 0.0f;
    Vec3 rampAlong;
    // E o deslocamento no fim da rampa (no referencial do osso de cima, vezes a
    // rampa) - o da modelagem (derivedRampOffset).
    Vec3 rampOffset;
};

// A character pairs a physical profile with a weighted visual surface.
// Bone names are resolved once on load, never guessed during playback.
//
// A pele (skin) liga cada osso a malha numa pose de ligacao. Na versao 1 ela
// e o repouso da fisica (a pose T do perfil). Na versao 2 cada osso tem as
// duas: o repouso (o do perfil; os ossos visuais se prendem ao pai por ele) e
// a ligacao da malha (outra pose: o modelo montado vem em pose A), e a malha
// deforma direto da pose dela - sem passar por uma pose T no meio.
struct RagdollCharacter3D {
    std::string id;
    std::string displayName;
    std::string thumbnailPath;
    std::string albedoPath;
    // Papéis de locomoção (manifesto: "locomotion"). idle e walk são
    // obrigatórios; walkBackward e sprint são opcionais - sem eles o corpo
    // gira para o movimento e não há sprint. sprintBackward (opcional) é o
    // recuo com sprint; sem ele, recuando, segurar sprint não corre.
    std::string idleClipId, walkClipId, walkBackwardClipId, sprintClipId;
    std::string idleToSprintClipId;
    // Arrancada agachada (Crouched To Sprinting): o levantar com pressa.
    std::string crouchToSprintClipId;
    std::string runForwardArcLeftClipId, runForwardArcRightClipId;
    std::string runBackwardArcRightClipId;
    std::string sprintBackwardClipId;
    // Strafe (opcionais): correr de lado de frente para a camera.
    std::string strafeLeftClipId, strafeRightClipId;
    std::string sprintStrafeLeftClipId, sprintStrafeRightClipId;
    // Pulos (opcionais): parado e por setor de movimento.
    std::string jumpStandingClipId, jumpForwardClipId, jumpBackwardClipId;
    std::string jumpLeftClipId, jumpRightClipId;
    // Preparo do pulo parado (agachando enquanto carrega).
    std::string jumpPrepClipId;
    std::string standUpFrontClipId, standUpBackClipId;
    // Giro parado (ajuda cheia): o clipe para a esquerda e para a direita.
    std::string turnLeftClipId, turnRightClipId;
    // Carrinho (opcionais): um firmando cada pe.
    std::vector<std::string> tackleClipIds;
    // Locomocao padrao (manifesto: "defaultLocomotion", opcional): papel ->
    // clipe que substitui o da locomocao fisica so no modo padrao (o trote
    // e o sprint das referencias; as diagonais so existem nele).
    std::vector<std::pair<std::string, std::string>> defaultLocomotionClipIds;
    RagdollProfile3D profile;
    MeshData3D mesh;
    // Paleta: um osso por link fisico (boneLinkIndices), depois os visuais e
    // os auxiliares.
    std::vector<std::size_t> boneLinkIndices;
    std::vector<RagdollVisualBone3D> visualBones;
    std::vector<RagdollHelperBone3D> helperBones;
    std::vector<Mat4> inverseBindMatrices; // fisicos e visuais
    // A ligacao da malha de cada osso fisico e visual (a inversa e a de cima).
    std::vector<Vec3> bindPositions;
    std::vector<Quaternion> bindOrientations;
    bool flatShaded = true;
    // O sombreamento (manifesto "facet"): 0 liso pelas normais da malha, 1
    // facetado (a normal de cada face), no meio a mistura. Sem ele, o de
    // flatShaded.
    float facet = 1.0f;
    // O sistema de aparencia (com a pele que o tem; vazios sem ele): por vertice
    // da pele, o endereco no atlas do uniforme (UV0), o da identificacao (UV1), a
    // regiao, a zona e o vertice de origem na modelagem; e o template de
    // superficie (manifesto "surface"), por triangulo na ordem da pele.
    struct SurfaceVertex {
        Vec2 kitUv { -1.0f, -1.0f };
        Vec2 identityUv { -1.0f, -1.0f };
        std::int16_t region = -1;
        std::int8_t identity = -1;
        std::uint32_t source = 0;
    };
    std::vector<SurfaceVertex> surfaceVertices;
    std::shared_ptr<const Appearance::SurfaceTemplate> surface;
};

[[nodiscard]] RagdollCharacter3D loadRagdollCharacter3D(const std::string& manifestPath);
// visualRotations: a rotacao de cada osso visual (na ordem do personagem) no
// lugar da de repouso; vazio, a de repouso.
[[nodiscard]] std::vector<Mat4> buildRagdollSkinMatrices3D(
    const RagdollCharacter3D& character,
    std::span<const Vec3> positions, std::span<const Quaternion> orientations,
    std::span<const Quaternion> visualRotations = {});
// Reference CPU deformation used by asset validation, independent of GPU.
[[nodiscard]] Vec3 skinVertexPosition3D(const MeshVertex3D& vertex,
    std::span<const Mat4> palette);

} // namespace MatterEngine
