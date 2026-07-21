#pragma once

#include "Engine/Math/Vec3.hpp"

namespace MatterEngine {

struct Ray3D {
    Vec3 origin;
    Vec3 direction { 0.0f, 1.0f, 0.0f };
};

} // namespace MatterEngine
