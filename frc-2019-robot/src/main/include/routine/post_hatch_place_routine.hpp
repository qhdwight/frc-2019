#pragma once

#include <lib/sequential_routine.hpp>

namespace garage {
    class PostHatchPlaceRoutine : public lib::SequentialRoutine {
    public:
        PostHatchPlaceRoutine(std::shared_ptr<Robot> robot);
    };

    class OpenHatchIntakeRoutine : public lib::Routine {
    public:
        OpenHatchIntakeRoutine(std::shared_ptr<Robot> robot);

    private:
        void Start() override;
    };
}