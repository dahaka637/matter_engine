#pragma once

#include "Engine/RL/Body/RLBody3D.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <span>
#include <string>
#include <vector>

namespace MatterEngine {

// As partes-chave da imitacao, na ordem da observacao: cabeca, maos e pes.
inline constexpr std::size_t ReferenceKeyBodyCount3D = 5;

// Um quadro da referencia no corpo de RL, a cada passo da fisica (1/120 s).
//
// O que se compara com o corpo fica no referencial do rumo da propria
// referencia (o eixo da frente da pelve, como o ambiente le o corpo): a
// imitacao vale para qualquer rumo, e o corpo pode virar seguindo a pose.
struct ReferenceFrame3D {
    // Por eixo atuado (na ordem de RLBody3D::dofs), presos no curso.
    std::vector<float> dofRadians;
    std::vector<float> dofVelocities;
    // Por link (o estado inicial pela referencia): as coordenadas da junta e
    // a derivada delas.
    std::vector<Vec3> jointCoordinates;
    std::vector<Vec3> jointVelocities;
    // A raiz e rumo * modelOrientation da pelve * rootRotation (a convencao
    // das posturas e dos clipes).
    Quaternion rootRotation;
    // A altura da origem da pelve, com o pe de apoio no chao.
    float rootHeight = 0.0f;
    // O rumo da pelve no mundo do clipe (rad, do eixo x).
    float heading = 0.0f;
    // A velocidade e o giro da pelve, no referencial do rumo.
    Vec3 rootVelocity;
    Vec3 rootSpin;
    // O "para cima" da pelve no referencial do rumo.
    Vec3 pelvisUp { 0.0f, 0.0f, 1.0f };
    // Cabeca, maos e pes em relacao a pelve, no referencial do rumo.
    std::array<Vec3, ReferenceKeyBodyCount3D> keyBodies {};
    // Os pes no apoio pelo clipe (1 apoiado, 0 no ar).
    std::array<float, 2> footContact {};
};

// Um trecho: um clipe tocado do comeco ao fim, ou em ciclo (o ultimo).
struct ReferenceSegment3D {
    std::string id;
    std::filesystem::path clipPath;
    bool loops = false;
    std::vector<ReferenceFrame3D> frames;
    // Onde a sequencia entra neste trecho vindo do anterior: o quadro mais
    // parecido com o fim dele (no primeiro trecho, 0).
    std::uint32_t entryFrame = 0;
    // O rumo em que o trecho anda no mundo do clipe (rad): o alvo de quem sai
    // deste trecho fica nessa direcao.
    float travelHeading = 0.0f;
    // A direcao em que ele anda em relacao ao rumo medio da pelve (rad): 0 para
    // a frente, pi/2 para a esquerda, pi para tras (o planejador escolhe por ela).
    float travelRelative = 0.0f;
    // A velocidade media no plano (m/s).
    float meanSpeed = 0.0f;
    // O maior pulo de pose na entrada (rad, RMS dos eixos): a emenda.
    float entryJumpRms = 0.0f;
};

// Uma referencia de movimento: os trechos em sequencia, o ultimo em ciclo; ou
// uma biblioteca (todos em ciclo, o planejador do ambiente escolhe qual), com
// a tabela de emendas: de cada quadro de um trecho, o quadro mais parecido de
// cada outro.
struct ReferenceMotion3D {
    std::vector<ReferenceSegment3D> segments;
    float frameSeconds = 1.0f / 120.0f;
    bool library = false;
    // transitions[de][para][quadro de "de"] = o quadro de entrada em "para".
    std::vector<std::vector<std::vector<std::uint32_t>>> transitions;
    std::uint64_t hash = 0;
};

// Um trecho pedido: o clipe e se ele fica em ciclo.
struct ReferenceClip3D {
    std::string id;
    std::filesystem::path clipPath;
    bool loops = false;
};

// Amostra os clipes no corpo. Lanca se um clipe nao for do corpo, se so o
// ultimo trecho nao estiver em ciclo, ou se a sequencia for vazia.
// Com library, todos os trechos ficam em ciclo e a tabela de emendas e feita.
[[nodiscard]] ReferenceMotion3D buildReferenceMotion3D(const RLBody3D& body, std::span<const ReferenceClip3D> clips,
    float frameSeconds = 1.0f / 120.0f, bool library = false);

// Onde a referencia esta: o trecho e o quadro dentro dele.
struct ReferenceCursor3D {
    std::uint32_t segment = 0;
    std::uint32_t frame = 0;
};

// O cursor andando frames quadros: no fim de um trecho a sequencia entra no
// seguinte pelo entryFrame dele; o ultimo da a volta.
[[nodiscard]] ReferenceCursor3D advanceReference3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor,
    std::uint32_t frames);

[[nodiscard]] const ReferenceFrame3D& referenceFrame3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor);

// Numa biblioteca: o cursor no trecho pedido, pelo quadro da emenda (no mesmo
// trecho, nada muda).
[[nodiscard]] ReferenceCursor3D switchReference3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor,
    std::uint32_t segment);

// O quanto o trecho ja andou (0 a 1; num ciclo, a fase).
[[nodiscard]] float referenceProgress3D(const ReferenceMotion3D& motion, ReferenceCursor3D cursor);

// Os links do corpo num quadro, no mundo: a pelve em (x, y) com o rumo dado
// (o rumo da pelve, rad).
[[nodiscard]] RLBodyPose3D referencePose3D(const RLBody3D& body, const ReferenceFrame3D& frame, Vec3 position,
    float heading);

// O estado inicial pela referencia (o RSI do DeepMimic): a pose e as
// velocidades do quadro, com a pelve em (x, y) e o rumo dado.
void referenceLinkStates3D(const RLBody3D& body, const ReferenceFrame3D& frame, Vec3 position, float heading,
    std::vector<PhysicsBodyState3D>& out);

} // namespace MatterEngine
