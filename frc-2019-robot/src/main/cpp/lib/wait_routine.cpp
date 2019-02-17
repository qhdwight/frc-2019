#include <lib/wait_routine.hpp>

namespace garage {
    namespace lib {
        void WaitRoutine::Begin() {
            m_StartTime = frc::Timer::GetFPGATimestamp();
            m_EndTime = m_StartTime + m_Duration;
        }

        WaitRoutine::WaitRoutine(std::shared_ptr<Robot>& robot, double duration) : Routine(robot), m_Duration(duration) {}

        bool WaitRoutine::CheckFinished() {
            return frc::Timer::GetFPGATimestamp() > m_EndTime;
        }
    }
}