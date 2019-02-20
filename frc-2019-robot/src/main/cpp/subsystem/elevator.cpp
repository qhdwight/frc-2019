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
        m_ElevatorMaster.ConfigOpenloopRamp(ELEVATOR_OPEN_LOOP_RAMP, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClosedloopRamp(0.2, CONFIG_TIMEOUT);
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
        m_ElevatorMaster.ConfigMotionAcceleration(ELEVATOR_ACCELERATION, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMotionCruiseVelocity(ELEVATOR_VELOCITY, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, ELEVATOR_F, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, ELEVATOR_P, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, ELEVATOR_D, CONFIG_TIMEOUT);
        // Network table values
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kP", ELEVATOR_P);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kD", ELEVATOR_D);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/kF", ELEVATOR_F);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Acceleration").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.ConfigMotionAcceleration(static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Velocity").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.ConfigMotionCruiseVelocity(static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/kF").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/kD").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/kP").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
    }

    void Elevator::TeleopInit() {
        m_ControlMode = m_DefaultControlMode;
    }

    void Elevator::ExecuteCommand(Command& command) {
        // Reset encoder with limit switch
        const bool isLimitSwitchDown = static_cast<const bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());
        if (isLimitSwitchDown && m_FirstLimitSwitchHit) {
            auto error = m_ElevatorMaster.SetSelectedSensorPosition(0);
            if (error == ctre::phoenix::ErrorCode::OKAY) {
                Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchHit = false;
            } else {
                Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CTRE Error: %d", static_cast<int>(error)));
            }
        }
        const int encoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(0);
        if (command.killSwitch)
            m_ControlMode = m_ControlMode == ElevatorControlMode::k_Killed ? m_DefaultControlMode : ElevatorControlMode::k_Killed;
        switch (m_ControlMode) {
            case ElevatorControlMode::k_Idle: {
                break;
            }
            case ElevatorControlMode::k_Manual: {
                m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, math::threshold(command.driveForwardFine, 0.1) * 0.5);
                break;
            }
            case ElevatorControlMode::k_Killed: {
                // Reset our command
                command.elevatorSetPoint = ELEVATOR_MIN;
                if (encoderPosition > KILL_ELEVATOR_POSITION_WEAK) {
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_WEAK);
                } else if (encoderPosition > KILL_ELEVATOR_POSITION_STRONG) {
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_STRONG);
                }
                break;
            }
            case ElevatorControlMode::k_SetPoint: {
                if (!m_IsLocked)
                    m_WantedSetPoint = command.elevatorSetPoint;
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Encoder", encoderPosition);
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Wanted Position", command.elevatorPosition);
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Output", m_ElevatorMaster.GetMotorOutputPercent());
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Master Amperage", m_ElevatorMaster.GetOutputCurrent());
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Limit Switch", isLimitSwitch ? 1.0 : 0.0);
                if (m_WantedSetPoint > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
                    // Our set point is above a negligible amount
                    if (encoderPosition < ELEVATOR_MAX) {
                        // In middle zone
                        LogSample(lib::LogLevel::k_Info, "Theoretically Okay and Working");
                        if (m_WantedSetPoint != m_LastSetPoint) {
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint);
                            m_LastSetPoint = m_WantedSetPoint;
                        }
                    } else {
                        // Too high and we must kill the elevator
                        Log(lib::LogLevel::k_Info, "Too High");
                        m_ControlMode = ElevatorControlMode::k_Killed;
                    }
                } else {
                    // We want to go a negligible amount
                    LogSample(lib::LogLevel::k_Info, "Not High Enough");
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                         isLimitSwitchDown ? 0.0 : SAFE_ELEVATOR_DOWN_STRONG);
                }
                break;
            }
            case ElevatorControlMode::k_Hybrid: {
                if (encoderPosition > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
                    const bool inputDifferent = math::abs(m_LastCommand.elevatorInput - command.elevatorInput) > 0.5;
                    if (math::abs(command.elevatorInput) > 0.5) {
                        if (inputDifferent)
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                                 command.elevatorInput > 0.0 ? ELEVATOR_UP_OUTPUT : ELEVATOR_DOWN_OUTPUT);
                    } else {
                        if (inputDifferent)
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 0.0);
                    }
                    break;
                } else {
                    LogSample(lib::LogLevel::k_Info, "Not High Enough");
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                         isLimitSwitchDown ? 0.0 : SAFE_ELEVATOR_DOWN_STRONG);
                }
            }
        }
        LogSample(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format(
                "Control Mode: %d, Wanted Set Point: %d, Encoder: %d, Real Output: %f, Current: %f, Limit Switch: %s",
                m_ControlMode, m_WantedSetPoint, encoderPosition, m_ElevatorMaster.GetMotorOutputPercent(), m_ElevatorMaster.GetOutputCurrent(),
                isLimitSwitchDown ? "true" : "false"));
    }

    int Elevator::GetElevatorPosition() {
        return m_ElevatorMaster.GetSelectedSensorPosition(0);
    }

    void Elevator::SetElevatorWantedPosition(int wantedPosition) {
        m_WantedSetPoint = wantedPosition;
    }
}