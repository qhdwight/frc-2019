#include <routine/ball_intake_routine.hpp>

#include <robot.hpp>

namespace garage {
    IntakeBallUntilIn::IntakeBallUntilIn(std::shared_ptr<Robot> robot)
            : SubsystemRoutine(robot, robot->GetBallIntake(), "Intake Ball Until In") {

    }

    void IntakeBallUntilIn::Start() {
        Routine::Start();
        m_Subsystem->Intake();
    }

    bool IntakeBallUntilIn::CheckFinished() {
        return m_Subsystem->HasBall();
    }

    void IntakeBallUntilIn::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    BallIntakeRoutine::BallIntakeRoutine(std::shared_ptr<Robot> robot) : lib::SequentialRoutine(robot, "Intake Ball", {
            // TODO tune values
            std::make_shared<ElevatorAndFlipperRoutine>(robot, 6000, FLIPPER_UPPER_ANGLE, "Flipper Ball Intake"),
            std::make_shared<IntakeBallUntilIn>(robot),
            std::make_shared<lib::WaitRoutine>(robot, 200l),
            std::make_shared<ElevatorAndFlipperRoutine>(robot, 0, FLIPPER_STOW_ANGLE, "Reset")
    }) {

    }
}