#include <routine/climb_hab_routine.hpp>

#include <robot.hpp>

#include <lib/logger.hpp>

#include <frc/PIDController.h>

namespace garage {
    OutriggerAutoLevelRoutine::OutriggerAutoLevelRoutine(std::shared_ptr<Robot>& robot) : lib::Routine(robot, "Outrigger Auto Level") {

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

    ElevatorRaiseRoutine::ElevatorRaiseRoutine(std::shared_ptr<Robot>& robot) : lib::Routine(robot, "Elevator Climb Routine") {

    }

    void ElevatorRaiseRoutine::Start() {
        Routine::Start();
        // TODO tune
        m_Robot->GetSubsystem<Elevator>()->SetWantedSetPoint(150000);
    }

    void ElevatorRaiseRoutine::Update() {
        Routine::Update();
    }

    void ElevatorRaiseRoutine::Terminate() {
        Routine::Terminate();
        m_Robot->GetSubsystem<Elevator>()->Unlock();
    }

    ClimbHabRoutine::ClimbHabRoutine(std::shared_ptr<Robot>& robot) : lib::ParallelRoutine(robot, "Climb Hab", {
        std::make_shared<OutriggerAutoLevelRoutine>(robot)
    }) {

    }
}

