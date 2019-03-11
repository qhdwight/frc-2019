#pragma once

#include <lib/subsystem_routine.hpp>

namespace garage {
    class Robot;
    class Flipper;
    class SetFlipperAngleRoutine : public lib::SubsystemRoutine<Flipper> {
    protected:
        double m_Angle;

    public:
        SetFlipperAngleRoutine(std::shared_ptr<Robot>& robot, double angle, const std::string& name);

        void Begin() override;

        void Terminate() override;
    };
}