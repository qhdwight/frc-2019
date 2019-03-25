#include <routine/drive_distance_routine.hpp>

namespace garage {

    bool DriveDistanceRoutine::CheckFinished() {
        return false;
    }

    DriveDistanceRoutine::DriveDistanceRoutine(std::shared_ptr<Robot> robot, double distance, const std::string &name)
        : SubsystemRoutine(robot, name) {}
}