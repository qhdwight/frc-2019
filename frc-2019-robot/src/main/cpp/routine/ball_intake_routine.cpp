#include <routine/ball_intake_routine.hpp>
#include <routine/stow_flipper_routine.hpp>
#include <routine/set_flipper_angle_routine.hpp>

#include <lib/wait_routine.hpp>

namespace garage {
    IntakeBallUntilIn::IntakeBallUntilIn(std::shared_ptr<Robot>& robot)
            : SubsystemRoutine(robot, robot->GetBallIntake(), "Intake Ball Then Stow") {

    }

    void IntakeBallUntilIn::Start() {
        Routine::Start();
        m_Subsystem->SetMode(IntakeMode::k_Intaking, 1.0);
    }

    bool IntakeBallUntilIn::CheckFinished() {
        return m_Subsystem->HasBall();
    }

    void IntakeBallUntilIn::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    BallIntakeRoutine::BallIntakeRoutine(std::shared_ptr<Robot>& robot) : lib::SequentialRoutine(robot, "Intake Ball", {
            std::make_shared<SetFlipperAngleRoutine>(robot, 180.0, "Flipper Ball Intake"),
            std::make_shared<IntakeBallUntilIn>(robot),
            std::make_shared<lib::WaitRoutine>(robot, 200l),
            std::make_shared<StowFlipperRoutine>(robot)
    }) {

    }
}