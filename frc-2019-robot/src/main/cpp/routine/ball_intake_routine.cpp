#include <routine/ball_intake_routine.hpp>

#include <robot.hpp>

namespace garage {
    IntakeBallUntilIn::IntakeBallUntilIn(std::shared_ptr<Robot> robot)
            : SubsystemRoutine(robot, "Intake Ball Until In") {

    }

    void IntakeBallUntilIn::Start() {
        Routine::Start();
        if (m_Subsystem) {
            m_Subsystem->Intake();
        }
    }

    bool IntakeBallUntilIn::CheckFinished() {
        return m_Subsystem ? m_Subsystem->HasBall() : true;
    }

    void IntakeBallUntilIn::Terminate() {
        Routine::Terminate();
        if (m_Subsystem) {
            m_Subsystem->Unlock();
        }
    }

    void BallIntakeRoutine::Start() {
        SequentialRoutine::Start();
        m_Robot->SetLedMode(Robot::LedMode::k_BallIntake);
    }

    void BallIntakeRoutine::Terminate() {
        SequentialRoutine::Terminate();
        m_Robot->SetLedMode(Robot::LedMode::k_Idle);
    }

    BallIntakeRoutine::BallIntakeRoutine(std::shared_ptr<Robot> robot, double setPoint, double angle) : SequentialRoutine(robot, "Intake Ball", {
            // TODO tune values
            std::make_shared<ElevatorAndFlipperRoutine>(robot, setPoint, angle, "Flipper Ball Intake"),
            std::make_shared<IntakeBallUntilIn>(robot)
//            std::make_shared<lib::WaitRoutine>(robot, 200l),
//            std::make_shared<ElevatorAndFlipperRoutine>(robot, 0, FLIPPER_LOWER_ANGLE, "Reset")
    }) {

    }
}