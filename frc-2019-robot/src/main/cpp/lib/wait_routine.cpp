#include <lib/wait_routine.hpp>

namespace garage {
    namespace lib {
        void WaitRoutine::Start() {
            Routine::Start();
            m_StartTime = std::chrono::system_clock::now();
            m_EndTime = m_StartTime + m_DurationMilliseconds;
        }

        bool WaitRoutine::CheckFinished() {
            return std::chrono::system_clock::now() > m_EndTime;
        }
    }
}