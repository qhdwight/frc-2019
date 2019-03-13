#pragma once

#include <lib/parallel_routine.hpp>

namespace garage {
    class OutriggerAutoLevelRoutine : public lib::Routine {
    public:
        OutriggerAutoLevelRoutine(std::shared_ptr<Robot>& robot);

        void Start() override;

        void Terminate() override;

        void Update() override;
    };

    class ElevatorRaiseRoutine : public lib::Routine {
    public:
        ElevatorRaiseRoutine(std::shared_ptr<Robot>& robot);

        void Start() override;

        void Terminate() override;

        void Update() override;
    };

    class ClimbHabRoutine : public lib::ParallelRoutine {
    public:
        ClimbHabRoutine(std::shared_ptr<Robot>& robot);
    };
}