#pragma once

namespace MatterEngine::UI {

// O tema do ImGui da engine (cantos, espacos e cores). Fica aqui, e nao no
// Renderer, para quem desenha a interface sem janela (os testes da tela de
// inspecao) usar o mesmo visual.
void applyMatterEngineStyle();

} // namespace MatterEngine::UI
