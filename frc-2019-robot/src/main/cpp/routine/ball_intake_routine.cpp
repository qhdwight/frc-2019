#include <routine/ball_intake_routine.hpp>

#include <lib/wait_routine.hpp>

namespace garage {
    IntakeBallThenStowRoutine::IntakeBallThenStowRoutine(std::shared_ptr<Robot>& robot)
        : lib::Routine(robot, "Intake Ball Then Stow"), m_BallIntake(robot->GetBallIntake()) {

    }

    void IntakeBallThenStowRoutine::Begin() {
        Routine::Begin();
        m_BallIntake->SetMode(IntakeMode::k_Intaking, 1.0);
    }

    StowRoutine::StowRoutine(std::shared_ptr<Robot>& robot)
        : lib::Routine(robot, "Stow Flipper"), m_Flipper(robot->GetFlipper()) {

    }

    BallIntakeRoutine::BallIntakeRoutine(std::shared_ptr<Robot>& robot) : lib::SequentialRoutine(robot, "Intake Ball", {
        std::make_shared<IntakeBallThenStowRoutine>(robot),
        std::make_shared<lib::WaitRoutine>(robot, 0.5)
    }) {

    }
}