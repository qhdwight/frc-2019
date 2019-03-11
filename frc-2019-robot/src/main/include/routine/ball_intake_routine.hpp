#pragma once

#include <lib/subsystem_routine.hpp>
#include <lib/sequential_routine.hpp>

#include <memory>

namespace garage {
    class Robot;
    class BallIntake;
    class IntakeBallUntilIn : public lib::SubsystemRoutine<BallIntake> {
    private:
        void Begin() override;

    public:
        IntakeBallUntilIn(std::shared_ptr<Robot>& robot);

        bool CheckFinished() override;

        void Terminate() override;
    };

    class BallIntakeRoutine : lib::SequentialRoutine {
    public:
        BallIntakeRoutine(std::shared_ptr<Robot>& robot);
    };
}