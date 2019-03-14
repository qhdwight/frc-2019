#include <routine/flip_over_routine.hpp>

#include <robot.hpp>

namespace garage {
    FlipOverRoutine::FlipOverRoutine(std::shared_ptr<Robot> robot) : SubsystemRoutine(robot, robot->GetFlipper(), "Flip Over") {

    }

    void FlipOverRoutine::Start() {
        Routine::Start();
        const double currentAngle = m_Subsystem->GetAngle();
        m_Subsystem->SetAngle(currentAngle < 90.0 ? 180.0 : 0.0);
    }

    void FlipOverRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }
}