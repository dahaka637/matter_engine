#pragma once

namespace MatterEngine::UI {

// Adds the default UI font and merges Font Awesome Free Solid into the same
// ImGui atlas, so text and icons can be rendered in a single draw call.
void configureImGuiFonts();

} // namespace MatterEngine::UI
