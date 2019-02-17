#pragma once

#include <lib/routine.hpp>

#include <frc/Timer.h>

namespace garage {
    namespace lib {
        class WaitRoutine : public Routine {
        protected:
            double m_StartTime, m_EndTime, m_Duration;
        public:
            WaitRoutine(std::shared_ptr<Robot>& robot, double duration);

            void Begin() override;

            bool CheckFinished() override;
        };
    }
}