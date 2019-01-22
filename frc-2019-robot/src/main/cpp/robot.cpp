#include <algorithm>

#include <robot.hpp>

#include <cameraserver/CameraServer.h>

namespace garage {
    void Robot::RobotInit() {
//        std::thread visionThread(VisionThread);
//        visionThread.detach();
        m_Subsystems.push_back(m_Drive);
        m_Subsystems.push_back(m_Intake);
    }

    void Robot::RobotPeriodic() {}

    void Robot::DisabledInit() {}

    void Robot::DisabledPeriodic() {}

    void Robot::AutonomousInit() {}

    void Robot::AutonomousPeriodic() {}

    void Robot::TeleopInit() {}

    void Robot::TeleopPeriodic() {
        Command command = GetCommand();
        for (const auto& subsystem : m_Subsystems)
            subsystem->ExecuteCommand(command);
    }

    Command Robot::GetCommand() {
        return {-m_Stick.GetY(), m_Stick.GetX(), m_Stick.GetTrigger()};
    }

    void Robot::VisionThread() {}
}

#ifndef RUNNING_FRC_TESTS

int main() {
    return frc::StartRobot<garage::Robot>();
}

#endif
