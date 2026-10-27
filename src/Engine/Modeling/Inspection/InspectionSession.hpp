#pragma once

#include "Engine/Appearance/Compose/KitCompositor.hpp"
#include "Engine/Appearance/Surface/SurfaceTemplate.hpp"
#include "Engine/Modeling/Inspection/InspectionRenderer.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace MatterEngine::Modeling {

// O modelo inspecionado: esqueleto, regioes e malha (vazia so com ossos).
struct InspectionModel {
    Skeleton skeleton;
    BodyRegions regions;
    Mesh mesh;
    std::vector<int> jointRegion;
    // O template de superficie da malha (o sistema de aparencia; vazio sem ele),
    // por triangulo na ordem de mesh.triangles.
    std::shared_ptr<const Appearance::SurfaceTemplate> surface;
    // O uniforme composto sobre o template (vazio: a textura antiga).
    std::shared_ptr<const Appearance::ComposedKit> kit;
    // Uma linha de onde veio (mostrada na tela).
    std::string source;
};

// O estado de uma inspecao interativa (a tela "Inspecao de jogador"): a
// regiao escolhida, a camera com transicoes suaves e a imagem, renderizada
// so quando algo muda. Nao conhece interface nem GPU: a tela so repassa o
// mouse e mostra image().
class InspectionSession {
public:
    void setModel(std::shared_ptr<const InspectionModel> model);
    [[nodiscard]] const InspectionModel* model() const { return m_model.get(); }
    // A pose mostrada (nada = a de ligacao): os ossos e a malha deformada.
    void setPose(std::optional<SkeletonPose> pose);
    [[nodiscard]] bool posed() const { return m_pose.has_value(); }

    void selectRegion(int region, bool animate = true);
    [[nodiscard]] int selectedRegion() const { return m_region; }
    void applyView(const ViewPreset& view, bool animate = true);
    // Volta ao enquadramento da regiao atual no angulo dela.
    void resetView(bool animate = true);

    // Mouse no viewport (pixels).
    void orbit(float deltaX, float deltaY);
    void pan(float deltaX, float deltaY, int viewportHeight);
    void zoom(float wheelSteps);
    // Clique: seleciona a regiao da junta mais perto (true se pegou uma).
    bool click(float x, float y, int width, int height);

    // Avanca as transicoes da camera.
    void update(float deltaSeconds);
    // Uma mudanca continua (a animacao tocando): a imagem rapida enquanto
    // ela dura e a completa quando para.
    void keepFast();

    [[nodiscard]] const InspectionStyle& style() const { return m_style; }
    // Mudancas no estilo pedem render novo.
    InspectionStyle& editStyle();
    void setOrthographic(bool orthographic);
    [[nodiscard]] bool orthographic() const { return m_goal.orthographic; }

    // Renderiza se algo mudou (ou o tamanho); true quando a imagem mudou.
    bool render(int width, int height);
    [[nodiscard]] const Image& image() const { return m_image; }
    [[nodiscard]] const OrbitCamera& camera() const { return m_camera; }
    [[nodiscard]] bool animating() const { return m_animating; }
    [[nodiscard]] InspectionScene scene() const;

private:
    void interacted();

    std::shared_ptr<const InspectionModel> m_model;
    std::optional<SkeletonPose> m_pose;
    std::vector<Vec3> m_posedPositions;
    // Os grupos do sombreamento liso do modelo (refeitos com o modelo ou com
    // a quina).
    std::shared_ptr<const ShadingNormals> m_shading;
    std::uint64_t m_modelGeneration = 0, m_shadingGeneration = 0;
    InspectionStyle m_style;
    OrbitCamera m_camera;
    OrbitCamera m_goal;
    int m_region = 0;
    bool m_animating = false;
    // O usuario girou/arrastou/aproximou desde o ultimo enquadramento.
    bool m_userAdjusted = false;
    bool m_dirty = true;
    // Enquanto mexe: render rapido (sem supersampling); parado, o completo.
    float m_interactionSeconds = 0.0f;
    bool m_lastFast = false;
    float m_aspect = 1.6f;
    Image m_image;
};

} // namespace MatterEngine::Modeling
