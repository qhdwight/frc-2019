#pragma once

#include <lib/subsystem_routine.hpp>

namespace garage {
    class Flipper;

    class FlipOverRoutine : public lib::SubsystemRoutine<Flipper> {
    public:
        FlipOverRoutine(std::shared_ptr<Robot> robot);

    private:
        void Start() override;

        void Terminate() override;
    };
}