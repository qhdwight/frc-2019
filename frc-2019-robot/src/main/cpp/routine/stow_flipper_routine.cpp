#include <routine/stow_flipper_routine.hpp>

namespace garage {
    StowFlipperRoutine::StowFlipperRoutine(std::shared_ptr<Robot>& robot)
            : SetFlipperAngleRoutine(robot, 90.0, "Stow Flipper") {

    }
}
