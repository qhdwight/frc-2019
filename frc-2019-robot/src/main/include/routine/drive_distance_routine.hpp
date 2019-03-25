#pragma once

#include <lib/subsystem_routine.hpp>

namespace garage {
    class Drive;

    class DriveDistanceRoutine : lib::SubsystemRoutine<Drive> {
    protected:
        double m_Distance;

        bool CheckFinished() override;

    public:
        DriveDistanceRoutine(std::shared_ptr<Robot> robot, double distance, const std::string& name = "Drive Distance Routine");
    };
}