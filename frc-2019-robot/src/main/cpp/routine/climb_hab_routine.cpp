#include <routine/climb_hab_routine.hpp>

#include <robot.hpp>

#include <routine/set_elevator_position_routine.hpp>

#include <lib/logger.hpp>
#include <lib/parallel_routine.hpp>

namespace garage {
    MainClimbRoutine::MainClimbRoutine(std::shared_ptr<Robot>& robot, double height)
            : Routine(robot, "Main Climb"), m_Height(height) {

    }

    void MainClimbRoutine::Start() {
        Routine::Start();
        m_Robot->SetLedMode(Robot::LedMode::k_Climb);
        auto elevator = m_Robot->GetSubsystem<Elevator>();
        auto drive = m_Robot->GetSubsystem<Drive>();
        drive->SetDriveOutput(CLIMB_DRIVE_GRAB_OUTPUT, CLIMB_DRIVE_GRAB_OUTPUT);
        elevator->Climb();
    }

    void MainClimbRoutine::Update() {
        auto drive = m_Robot->GetSubsystem<Drive>();
        auto elevator = m_Robot->GetSubsystem<Elevator>();
        auto outrigger = m_Robot->GetSubsystem<Outrigger>();
        const double tilt = drive->GetTilt();
        drive->LogSample(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(" [%s] Tilt: %f", FMT_STR(m_Name), tilt));
        outrigger->SetRawOutput(tilt * OUTRIGGER_CLIMB_P + tilt * OUTRIGGER_CLIMB_F);
        if (elevator->GetPosition() < m_Height + ELEVATOR_WITHIN_SET_POINT_AMOUNT) {
            outrigger->SetWheelRawOutput(0.1);
        }
    }

    void MainClimbRoutine::Terminate() {
        Routine::Terminate();
        m_Robot->SetLedMode(Robot::LedMode::k_Idle);
        m_Robot->GetSubsystem<Drive>()->Unlock();
        m_Robot->GetSubsystem<Elevator>()->Unlock();
        m_Robot->GetSubsystem<Outrigger>()->Unlock();
    }

    bool MainClimbRoutine::CheckFinished() {
        return false;
    }

    ClimbHabRoutine::ClimbHabRoutine(std::shared_ptr<Robot>& robot, double height) : SequentialRoutine(robot, "Climb Hab", {
            std::make_shared<lib::ParallelRoutine>(robot, "Elevator and Outrigger initial Positions", lib::RoutineVector {
                    std::make_shared<SetElevatorPositionRoutine>(robot, height + CLIMB_INITIAL_GAP_HEIGHT),
                    std::make_shared<SetOutriggerAngleRoutine>(robot, 180.0)
            }),
            std::make_shared<MainClimbRoutine>(robot, height)
    }) {

    }

    SetOutriggerAngleRoutine::SetOutriggerAngleRoutine(std::shared_ptr<Robot>& robot, double angle, const std::string& name)
            : SubsystemRoutine(robot, name), m_Angle(angle) {

    }

    void SetOutriggerAngleRoutine::Start() {
        Routine::Start();
        m_Subsystem->SetWantedAngle(m_Angle);
    }

    void SetOutriggerAngleRoutine::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool SetOutriggerAngleRoutine::CheckFinished() {
        return m_Subsystem->WithinAngle(m_Angle);
    }
}

