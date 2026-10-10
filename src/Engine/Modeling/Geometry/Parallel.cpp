#include "Engine/Modeling/Geometry/Parallel.hpp"

namespace MatterEngine::Modeling {

TaskScheduler& modelingScheduler() {
    static TaskScheduler scheduler;
    return scheduler;
}

} // namespace MatterEngine::Modeling
