#pragma once

#include <lib/parallel_routine.hpp>
#include <lib/sequential_routine.hpp>

namespace garage {
    class OutriggerAutoLevelRoutine : public lib::Routine {
    protected:
        void Update() override;

    public:

        OutriggerAutoLevelRoutine(std::shared_ptr<Robot>& robot);

        void Start() override;

        void Terminate() override;
    };

    class ElevatorRaiseRoutine : public lib::Routine {
    protected:
        void Update() override;

    public:
        ElevatorRaiseRoutine(std::shared_ptr<Robot>& robot);

        void Start() override;

        void Terminate() override;
    };

    class ClimbHabRoutine : public lib::SequentialRoutine {
    public:
        ClimbHabRoutine(std::shared_ptr<Robot>& robot);
    };
}