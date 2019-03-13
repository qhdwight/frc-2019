#pragma once

#include <lib/sequential_routine.hpp>

#include <routine/elevator_and_flipper_routine.hpp>

namespace garage {
    class Robot;

    class Elevator;

    class BallPlacementRoutine : public ElevatorAndFlipperRoutine {
    protected:

    public:
        BallPlacementRoutine(std::shared_ptr<Robot>& robot, int setPoint, double angle, const std::string& name)
                : lib::SequentialRoutine(robot, name, {
                std::make_shared<SetElevatorPositionRoutine>(robot, setPoint, angle, name),
        }) {

        }
    };
}