#pragma once

#include <lib/routine.hpp>

namespace garage {
    class LowerElevatorRoutine : lib::Routine {
    public:
        LowerElevatorRoutine(std::shared_ptr<Robot>& robot);
    };
}