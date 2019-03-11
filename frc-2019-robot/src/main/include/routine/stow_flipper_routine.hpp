#pragma once

#include <robot.hpp>

#include <lib/subsystem_routine.hpp>
#include <lib/sequential_routine.hpp>

namespace garage {
    class StowRoutine : public lib::SubsystemRoutine<Flipper> {
    public:
        StowRoutine(std::shared_ptr<Robot>& robot);

        void Begin() override;

        void Terminate() override;
    };
}