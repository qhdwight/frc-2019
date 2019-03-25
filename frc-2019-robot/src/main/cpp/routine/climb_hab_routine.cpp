#include <routine/climb_hab_routine.hpp>

#include <robot.hpp>

#include <routine/set_elevator_position_routine.hpp>

#include <lib/logger.hpp>

#include <frc/PIDController.h>

namespace garage {
    OutriggerAutoLevelRoutine::OutriggerAutoLevelRoutine(std::shared_ptr<Robot>& robot) : Routine(robot, "Outrigger Auto Level") {

    }

    void OutriggerAutoLevelRoutine::Start() {
        Routine::Start();
    }

    void OutriggerAutoLevelRoutine::Update() {
        auto drive = m_Robot->GetSubsystem<Drive>();
        const double tilt = drive->GetTilt();
        drive->LogSample(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(" [%s] Tilt: %f", FMT_STR(m_Name), tilt));
//        const double p = 1.0 / 90.0;
//        m_Robot->GetSubsystem<Outrigger>()->SetOutput(tilt * p);
    }

    void OutriggerAutoLevelRoutine::Terminate() {
        Routine::Terminate();
        m_Robot->GetSubsystem<Outrigger>()->Unlock();
    }

    ClimbHabRoutine::ClimbHabRoutine(std::shared_ptr<Robot>& robot) : SequentialRoutine(robot, "Climb Hab", {
        std::make_shared<SetElevatorPositionRoutine>(robot, 20.0),
        std::make_shared<OutriggerAutoLevelRoutine>(robot)
    }) {

    }
}

