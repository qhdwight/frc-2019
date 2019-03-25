#pragma once

#include <lib/wait_routine.hpp>

namespace garage {
    class Drive;

    class TimedDriveRoutine : public lib::WaitRoutine {
    protected:
        double m_Output;
        std::shared_ptr<Drive> m_Drive;

    public:
        TimedDriveRoutine(std::shared_ptr<Robot> robot, long durationMilliseconds, double output = 0.2, const std::string& name = "Drive Distance Routine");

        void Start() override;

        void Terminate() override;
    };
}