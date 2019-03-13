#pragma once

#include <lib/parallel_routine.hpp>

#include <routine/set_flipper_angle_routine.hpp>
#include <routine/set_elevator_position_routine.hpp>

namespace garage {
    class Robot;
    
    class ElevatorAndFlipperRoutine : public lib::ParallelRoutine {
    public:
        ElevatorAndFlipperRoutine(std::shared_ptr<Robot>& robot, int setPoint, double angle, const std::string& name)
                : lib::ParallelRoutine(robot, name, {
                std::make_shared<SetElevatorPositionRoutine>(robot, setPoint),
                std::make_shared<SetFlipperAngleRoutine>(robot, angle)
        }) {

        }
    };
}
