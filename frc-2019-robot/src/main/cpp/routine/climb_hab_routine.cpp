#include <routine/climb_hab_routine.hpp>

#include <robot.hpp>

#include <lib/logger.hpp>

namespace garage {
    OutriggerAutoLevelRoutine::OutriggerAutoLevelRoutine(std::shared_ptr<Robot>& robot) : lib::Routine(robot, "Outrigger Auto Level") {

    }

    void OutriggerAutoLevelRoutine::Begin() {
        Routine::Begin();
    }

    void OutriggerAutoLevelRoutine::Update() {
        const double tilt = m_Robot->GetDrive()->GetTilt();
        m_Robot->GetDrive()->LogSample(lib::Logger::LogLevel::k_Info, lib::Logger::Format(" [%s] Tilt: %f", FMT_STR(m_Name), tilt));
//        const double p = 1.0 / 90.0;
//        m_Robot->GetOutrigger()->SetOutput(tilt * p);
    }

    void OutriggerAutoLevelRoutine::Terminate() {
        Routine::Terminate();
        m_Robot->GetOutrigger()->Unlock();
    }

    ElevatorRaiseRoutine::ElevatorRaiseRoutine(std::shared_ptr<Robot>& robot) : lib::Routine(robot, "Elevator Climb Routine") {

    }

    void ElevatorRaiseRoutine::Begin() {
        Routine::Begin();
        m_Robot->GetElevator()->SetWantedSetPoint(150000);
    }

    void ElevatorRaiseRoutine::Update() {
        Routine::Update();
    }

    void ElevatorRaiseRoutine::Terminate() {
        Routine::Terminate();
        m_Robot->GetElevator()->Unlock();
    }

    ClimbHabRoutine::ClimbHabRoutine(std::shared_ptr<Robot>& robot) : lib::ParallelRoutine(robot, "Climb Hab", {
        std::make_shared<OutriggerAutoLevelRoutine>(robot)
    }) {

    }
}

