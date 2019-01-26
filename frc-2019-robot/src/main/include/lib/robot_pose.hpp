#pragma once

#include <garage_math/vector.hpp>

namespace garage {
    namespace lib {
        struct RobotPose {
        public:
            math::vector<double, 2> position;
        };
    }
}