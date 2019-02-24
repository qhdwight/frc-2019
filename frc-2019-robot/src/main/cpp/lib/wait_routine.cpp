#include <lib/wait_routine.hpp>

namespace garage {
    namespace lib {
        void WaitRoutine::Begin() {
            Routine::Begin();
            m_StartTime = frc::Timer::GetFPGATimestamp();
            m_EndTime = m_StartTime + m_Duration;
        }

        bool WaitRoutine::CheckFinished() {
            return frc::Timer::GetFPGATimestamp() > m_EndTime;
        }
    }
}