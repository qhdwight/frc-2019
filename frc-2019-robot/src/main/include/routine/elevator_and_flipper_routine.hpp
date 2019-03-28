#pragma once

#include <lib/parallel_routine.hpp>

#include <routine/set_flipper_angle_routine.hpp>
#include <routine/set_elevator_position_routine.hpp>

namespace garage {
    class ElevatorAndFlipperRoutine : public lib::ParallelRoutine {
    public:
        ElevatorAndFlipperRoutine(std::shared_ptr<Robot> robot, double setPoint, double angle,
                                  const std::string& name = "Elevator And Flipper Routine")
                : ParallelRoutine(robot, name, {
                std::make_shared<SetElevatorPositionRoutine>(robot, setPoint),
                std::make_shared<SetFlipperAngleRoutine>(robot, angle)
        }) {

        }
    };
}
