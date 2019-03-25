#include <routine/timed_drive_routine.hpp>

#include <robot.hpp>

namespace garage {
    TimedDriveRoutine::TimedDriveRoutine(std::shared_ptr<Robot> robot, long durationMilliseconds, double output, const std::string &name)
        : WaitRoutine(robot, durationMilliseconds, name), m_Drive(robot->GetSubsystem<Drive>()), m_Output(output) {
    }

    void TimedDriveRoutine::Start() {
        WaitRoutine::Start();
        if (m_Drive) {
            m_Drive->SetDriveOutput(m_Output, m_Output);
        }
    }

    void TimedDriveRoutine::Terminate() {
        Routine::Terminate();
        if (m_Drive) {
            m_Drive->Unlock();
        }
    }
}