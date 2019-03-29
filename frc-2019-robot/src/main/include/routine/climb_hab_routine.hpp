#pragma once

#include <lib/subsystem_routine.hpp>
#include <lib/sequential_routine.hpp>

#define CLIMB_DRIVE_GRAB_OUTPUT 0.1 // Percent output
#define CLIMB_INITIAL_GAP_HEIGHT 2.0 // Elevator encoder ticks

#define OUTRIGGER_CLIMB_P 0.0
#define OUTRIGGER_CLIMB_F 0.0

namespace garage {
    class Outrigger;

    class SetOutriggerAngleRoutine : public lib::SubsystemRoutine<Outrigger> {
    protected:
        double m_Angle = 0.0;

    public:
        SetOutriggerAngleRoutine(std::shared_ptr<Robot>& robot, double angle, const std::string& name = "Set Outrigger Angle Routine");

        void Start() override;

        void Terminate() override;

    protected:
        bool CheckFinished() override;
    };

    class MainClimbRoutine : public lib::Routine {
    protected:
        double m_Height = 0.0;

        void Update() override;

        bool CheckFinished() override;

    public:
        MainClimbRoutine(std::shared_ptr<Robot>& robot, double height);

        void Start() override;

        void Terminate() override;
    };

    class ClimbHabRoutine : public lib::SequentialRoutine {
    public:
        ClimbHabRoutine(std::shared_ptr<Robot>& robot, double height);
    };
}