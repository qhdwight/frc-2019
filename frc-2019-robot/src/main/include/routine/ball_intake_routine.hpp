#pragma once

#include <lib/subsystem_routine.hpp>
#include <lib/sequential_routine.hpp>

#include <memory>

namespace garage {
    class Robot;

    class BallIntake;

    class IntakeBallUntilIn : public lib::SubsystemRoutine<BallIntake> {
    protected:
        void Start() override;

        bool CheckFinished() override;

    public:
        IntakeBallUntilIn(std::shared_ptr<Robot>& robot);

        void Terminate() override;
    };

    class BallIntakeRoutine : lib::SequentialRoutine {
    public:
        BallIntakeRoutine(std::shared_ptr<Robot>& robot);
    };
}