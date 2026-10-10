#pragma once

#include "Engine/Modeling/Geometry/ModelGeometry.hpp"

#include <span>
#include <string_view>

namespace MatterEngine::Modeling {

// Camera em orbita de um alvo. yaw 0 = de frente (camera em +x olhando o
// personagem); +90 graus = do lado esquerdo dele (+y); pitch positivo = de
// cima.
struct OrbitCamera {
    Vec3 target {};
    float yaw = 0.0f;
    float pitch = 0.1f;
    float distance = 3.0f;
    float verticalFov = 28.0f * DegreesToRadians;
    bool orthographic = false;
    float orthoHalfHeight = 1.0f;

    [[nodiscard]] Vec3 eye() const;
    [[nodiscard]] Vec3 forward() const;
    [[nodiscard]] Vec3 right() const;
    [[nodiscard]] Vec3 up() const;
};

struct ScreenPoint {
    // Pixels (y para baixo).
    float x = 0.0f;
    float y = 0.0f;
    // Chave de profundidade linear na tela: maior = mais perto.
    float depth = 0.0f;
    // Distancia ao longo do olhar (metros).
    float cameraZ = 0.0f;
    bool visible = false;
};

// A camera numa imagem de tamanho dado: projecao, profundidade e raios.
class Projector {
public:
    Projector(const OrbitCamera& camera, int width, int height);

    [[nodiscard]] int width() const { return m_width; }
    [[nodiscard]] int height() const { return m_height; }
    [[nodiscard]] bool orthographic() const { return m_orthographic; }
    [[nodiscard]] Vec3 eye() const { return m_eye; }
    [[nodiscard]] Vec3 forward() const { return m_forward; }
    [[nodiscard]] Vec3 right() const { return m_right; }
    [[nodiscard]] Vec3 up() const { return m_up; }
    [[nodiscard]] float nearPlane() const { return m_near; }

    // (direita, cima, frente) relativos ao olho.
    [[nodiscard]] Vec3 toCamera(Vec3 world) const;
    [[nodiscard]] ScreenPoint projectCamera(Vec3 cameraSpace) const;
    [[nodiscard]] ScreenPoint project(Vec3 world) const;
    [[nodiscard]] float depthKey(float cameraZ) const;
    // A chave de um ponto "meters" mais perto da camera (o bias das linhas).
    [[nodiscard]] float towardCamera(float key, float meters) const;
    // Quantos pixels um metro ocupa nessa distancia.
    [[nodiscard]] float pixelsPerMeter(float cameraZ) const;
    // Raio do olho pelo pixel (escolher juntas com o mouse).
    void ray(float pixelX, float pixelY, Vec3& origin, Vec3& direction) const;
    // A direcao do olho ate um ponto (para iluminar e para o fresnel).
    [[nodiscard]] Vec3 viewDirection(Vec3 world) const;

private:
    int m_width = 1;
    int m_height = 1;
    bool m_orthographic = false;
    Vec3 m_eye, m_forward, m_right, m_up;
    float m_focal = 1.0f;
    float m_aspect = 1.0f;
    float m_orthoHalfHeight = 1.0f;
    float m_near = 0.005f;
};

// Afasta a camera (ou abre a ortografica) para a caixa caber na imagem,
// mantendo o angulo; o alvo vai para o centro da caixa.
void frameBox(OrbitCamera& camera, const Box3& box, float aspect, float margin = 1.15f);

// O mesmo, justo: esferas (ponto e raio) em vez da caixa alinhada aos eixos,
// que cresce quando a camera esta inclinada.
struct FramePoint {
    Vec3 position;
    float radius = 0.0f;
};
void framePoints(OrbitCamera& camera, std::span<const FramePoint> points, float aspect, float margin = 1.15f);

struct ViewPreset {
    std::string_view id;
    std::string_view label;
    float yawDegrees = 0.0f;
    float pitchDegrees = 0.0f;
};
// frente, costas, direita, esquerda, cima, baixo, 34, 34e, 34c.
[[nodiscard]] std::span<const ViewPreset> viewPresets();
[[nodiscard]] const ViewPreset* findViewPreset(std::string_view id);

} // namespace MatterEngine::Modeling
