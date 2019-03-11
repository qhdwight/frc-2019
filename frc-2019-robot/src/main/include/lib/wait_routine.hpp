#pragma once

#include <lib/routine.hpp>

#include <chrono>

namespace garage {
    namespace lib {
        class WaitRoutine : public Routine {
        protected:
            std::chrono::system_clock::time_point m_StartTime, m_EndTime;
            std::chrono::milliseconds m_DurationMilliseconds;
        public:
            WaitRoutine(std::shared_ptr<Robot>& robot, long durationMilliseconds, const std::string& name = "Wait Routine")
                    : Routine(robot, name), m_DurationMilliseconds(durationMilliseconds) {};

            void Begin() override;

            bool CheckFinished() override;
        };
    }
}