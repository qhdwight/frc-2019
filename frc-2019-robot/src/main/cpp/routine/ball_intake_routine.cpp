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
}