#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Elevator") {
        m_ElevatorMaster.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveOne.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveTwo.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveThree.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, 0, CONFIG_TIMEOUT);
        // Set brake mode
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorMaster.ConfigOpenloopRamp(0.2, CONFIG_TIMEOUT);
        // Current limiting
//        m_ElevatorMaster.EnableCurrentLimit(true);
//        m_ElevatorMaster.ConfigPeakCurrentLimit(300, CONFIG_TIMEOUT);
//        m_ElevatorMaster.ConfigContinuousCurrentLimit(100, CONFIG_TIMEOUT);
        // Configure following and inversion
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorMaster.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
        m_ElevatorSlaveOne.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/kP", ELEVATOR_P);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/kD", ELEVATOR_D);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/kF", ELEVATOR_F);
        m_ElevatorMaster.ConfigMotionAcceleration(ELEVATOR_ACCELERATION, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMotionCruiseVelocity(ELEVATOR_VELOCITY, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kF(0, ELEVATOR_F, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kP(0, ELEVATOR_P, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kD(0, ELEVATOR_D, CONFIG_TIMEOUT);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/Acceleration").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.ConfigMotionAcceleration(static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/Velocity").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.ConfigMotionCruiseVelocity(static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kF").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kF(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kD").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kD(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
//        m_Robot->GetNetworkTable()->GetEntry("Elevator/kP").AddListener([&](const nt::EntryNotification& notification) {
//            m_ElevatorMaster.Config_kP(0, static_cast<int>(notification.value->GetDouble()));
//        }, 0xFF);
    }

    void Elevator::TeleopInit() {
        m_KillActivated = false;
    }

    void Elevator::ExecuteCommand(Command& command) {
        switch (m_ControlMode) {
            case MANUAL: {
                m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, math::threshold(command.driveForwardFine, 0.1));
                break;
            }
            case SETPOINT: {
                const bool isLimitSwitch = static_cast<const bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());
                if (isLimitSwitch && m_FirstLimitSwitchHit) {
                    auto error = m_ElevatorMaster.SetSelectedSensorPosition(0);
                    if (error == ctre::phoenix::ErrorCode::OKAY) {
                        Log(lib::LogLevel::kInfo, "Limit switch hit and encoder reset");
                        m_FirstLimitSwitchHit = false;
                    } else {
                        Log(lib::LogLevel::kError, "CTRE Error: " + std::to_string(static_cast<int>(error)));
                    }
                }
                if (!m_IsLocked)
                    m_WantedSetPoint = command.elevatorPosition;
                const int encoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(0);
                const double output = math::threshold(command.driveForward, 0.05) * 0.4;
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Encoder", encoderPosition);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Wanted Position", command.elevatorPosition);
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Output", m_ElevatorMaster.GetMotorOutputPercent());
                m_Robot->GetNetworkTable()->PutNumber("Elevator/Master Amperage", m_ElevatorMaster.GetOutputCurrent());
//        m_Robot->GetNetworkTable()->PutNumber("Elevator/Limit Switch", isLimitSwitch ? 1.0 : 0.0);
                LogSample(lib::LogLevel::kInfo,
                          "Wanted Position: " + std::to_string(m_WantedSetPoint) + ", Kill Switch: " + std::to_string(m_KillActivated) +
                          ", Encoder: " +
                          std::to_string(encoderPosition) + ", Theoretical out: " + std::to_string(output) + ", Pos:" +
                          std::to_string(encoderPosition) +
                          ", Real out:" +
                          std::to_string(m_ElevatorMaster.GetMotorOutputPercent()) + ", Current: " +
                          std::to_string(m_ElevatorMaster.GetOutputCurrent()) + ", Limit switch: " + std::to_string(isLimitSwitch), 5);
                if (command.killSwitch) m_KillActivated = !m_KillActivated;
                if (m_KillActivated) {
                    command.elevatorPosition = ELEVATOR_MIN;
                    if (encoderPosition > 30000) {
                        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.09);
                    } else if (encoderPosition > 9000) {
                        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.11);
                    }
                } else {
                    // We want to go a negligible amount
                    if (m_WantedSetPoint > ELEVATOR_MIN_SETPOINT_HEIGHT) {
                        if (encoderPosition < ELEVATOR_MAX) {
                            // In middle zone
                            LogSample(lib::LogLevel::kInfo, "Theoretically Okay");
                            if (m_WantedSetPoint != m_LastSetPoint) {
                                m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint);
                                m_LastSetPoint = m_WantedSetPoint;
                            }
                        } else {
                            // Too high
                            Log(lib::LogLevel::kInfo, "Too High");
                            m_KillActivated = true;
                        }
                    } else {
                        LogSample(lib::LogLevel::kInfo, "Not High Enough");
                        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.105);
                    }
                }
                break;
            }
            case HYBRID: {
                break;
            }
        }
    }

    int Elevator::GetElevatorPosition() {
        return m_ElevatorMaster.GetSelectedSensorPosition(0);
    }

    void Elevator::SetElevatorWantedPosition(int wantedPosition) {
        m_WantedSetPoint = wantedPosition;
    }
}