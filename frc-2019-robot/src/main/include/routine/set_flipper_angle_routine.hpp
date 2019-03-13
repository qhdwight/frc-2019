#pragma once

#include <lib/subsystem_routine.hpp>

namespace garage {
    class Robot;
    class Flipper;
    class SetFlipperAngleRoutine : public lib::SubsystemRoutine<Flipper> {
    protected:
        double m_Angle = 0.0;

    public:
        SetFlipperAngleRoutine(std::shared_ptr<Robot>& robot, double angle, const std::string& name = "Set Flipper Angle");

        void Start() override;

        void Terminate() override;
    };
}