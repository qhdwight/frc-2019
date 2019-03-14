#pragma once

#include <routine/elevator_and_flipper_routine.hpp>

#include <lib/wait_routine.hpp>
#include <lib/subsystem_routine.hpp>
#include <lib/sequential_routine.hpp>

#include <memory>

namespace garage {
    class BallIntake;

    class IntakeBallUntilIn : public lib::SubsystemRoutine<BallIntake> {
    protected:
        void Start() override;

        bool CheckFinished() override;

    public:
        IntakeBallUntilIn(std::shared_ptr<Robot> robot);

        void Terminate() override;
    };

    class BallIntakeRoutine : public lib::SequentialRoutine {
    public:
        BallIntakeRoutine(std::shared_ptr<Robot> robot) : lib::SequentialRoutine(robot, "Intake Ball", {
                // TODO tune values
                std::make_shared<ElevatorAndFlipperRoutine>(robot, 5000, 180.0, "Flipper Ball Intake"),
                std::make_shared<IntakeBallUntilIn>(robot),
                std::make_shared<lib::WaitRoutine>(robot, 200l),
                std::make_shared<ElevatorAndFlipperRoutine>(robot, 0, 90.0, "Reset")
        }) {

        }
    };
}