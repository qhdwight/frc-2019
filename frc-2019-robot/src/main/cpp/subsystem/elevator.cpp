#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Elevator") {
        m_ElevatorMaster.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveOne.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveTwo.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveThree.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, SET_POINT_SLOT_INDEX, CONFIG_TIMEOUT);
        // Set brake mode
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorMaster.ConfigOpenloopRamp(ELEVATOR_OPEN_LOOP_RAMP, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClosedloopRamp(ELEVATOR_CLOSED_LOOP_RAMP, CONFIG_TIMEOUT);
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
        m_ElevatorMaster.Config_kI(SET_POINT_SLOT_INDEX, ELEVATOR_I, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_IntegralZone(SET_POINT_SLOT_INDEX, ELEVATOR_I_ZONE, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigAllowableClosedloopError(SET_POINT_SLOT_INDEX, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, CONFIG_TIMEOUT);
        // Network table values
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/P", ELEVATOR_P);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/D", ELEVATOR_D);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/F", ELEVATOR_F);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/I", ELEVATOR_F);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/I Zone", ELEVATOR_F);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Acceleration").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.ConfigMotionAcceleration(static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Velocity").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.ConfigMotionCruiseVelocity(static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/F").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, notification.value->GetDouble(), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/D").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, notification.value->GetDouble(), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/P").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, notification.value->GetDouble(), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/I").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_kI(SET_POINT_SLOT_INDEX, notification.value->GetDouble(), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/I Zone").AddListener([&](const nt::EntryNotification& notification) {
            m_ElevatorMaster.Config_IntegralZone(SET_POINT_SLOT_INDEX, static_cast<int>(notification.value->GetDouble()), CONFIG_TIMEOUT);
        }, NT_NOTIFY_UPDATE);
    }

    void Elevator::TeleopInit() {
        m_ControlMode = m_DefaultControlMode;
    }

    void Elevator::ProcessCommand(Command& command) {
        m_Input = math::threshold(m_ControlMode == ElevatorControlMode::k_Manual ? command.driveForwardFine : command.elevatorInput, 0.05);
        if (command.elevatorSoftLand) {
            m_ControlMode = ElevatorControlMode::k_SoftLand;
            m_WantedSetPoint = ELEVATOR_MIN;
        } else if (m_Input != 0.0) {
            m_ControlMode = m_DefaultControlMode;
            m_WantedSetPoint += static_cast<int>(m_Input * 3000.0);
            m_WantedSetPoint = math::clamp(m_WantedSetPoint, ELEVATOR_MIN, ELEVATOR_MAX);
        }
    }

    void Elevator::Update() {
        // TODO add brownout detection and smart current monitoring, check sticky faults
//        m_ElevatorMaster.GetStickyFaults(m_StickyFaults);
//        m_ElevatorMaster.ClearStickyFaults();
        // Reset encoder with limit switch
        m_IsLimitSwitchDown = static_cast<const bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());
        if (m_IsLimitSwitchDown && m_FirstLimitSwitchHit) {
            /* This is the first update frame that our limit switch has been set to the down position */
            // We want to reset to encoder back to zero because we know we are at
            // the lowest position possible and the encoder has probably drifted over time
            auto error = m_ElevatorMaster.SetSelectedSensorPosition(0);
            if (error == ctre::phoenix::ErrorCode::OKAY) {
                Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchHit = false;
            } else {
                Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CTRE Error: %d", static_cast<int>(error)));
            }
        }
        m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(0);
        switch (m_ControlMode) {
            case ElevatorControlMode::k_Idle: {
                break;
            }
            case ElevatorControlMode::k_Manual: {
                m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Input * 0.5);
                break;
            }
            case ElevatorControlMode::k_SoftLand: {
                if (m_EncoderPosition > SOFT_LAND_ELEVATOR_POSITION_WEAK) {
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_WEAK);
                } else if (m_EncoderPosition > SOFT_LAND_ELEVATOR_POSITION_STRONG) {
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_STRONG);
                }
                break;
            }
            case ElevatorControlMode::k_SetPoint: {
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Encoder", encoderPosition);
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Wanted Position", command.elevatorPosition);
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Output", m_ElevatorMaster.GetMotorOutputPercent());
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Master Amperage", m_ElevatorMaster.GetOutputCurrent());
//                m_Robot->GetNetworkTable()->PutNumber("Elevator/Limit Switch", isLimitSwitch ? 1.0 : 0.0);
                if (m_WantedSetPoint > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
                    // Our set point is above a negligible amount
                    if (m_EncoderPosition < ELEVATOR_MAX) {
                        // In middle zone
                        LogSample(lib::LogLevel::k_Info, "Theoretically Okay and Working");
                        if (m_WantedSetPoint != m_LastSetPoint) {
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint);
                            m_LastSetPoint = m_WantedSetPoint;
                        }
                    } else {
                        // Too high and we must kill the elevator
                        Log(lib::LogLevel::k_Error, "Too High");
                        m_ControlMode = ElevatorControlMode::k_SoftLand;
                    }
                } else {
                    // We want to go a negligible amount, not worth using a closed loop near the bottom
                    LogSample(lib::LogLevel::k_Info, "Not High Enough");
                    // Set the elevator to coast output or zero if we are hitting the limit switch
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                         m_IsLimitSwitchDown ? 0.0 : SAFE_ELEVATOR_DOWN_STRONG);
                }
                break;
            }
            case ElevatorControlMode::k_Hybrid: {
                // TODO add too high checking
                if (m_EncoderPosition > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
                    /* We are a negligible amount above the bottom */
                    // Test if our input is different from the last one so we avoid flooding the controller with requests
                    const bool inputDifferent = math::abs(m_LastCommand.elevatorInput - m_Input) > 0.5;
                    if (m_Input == 0.0 || (m_EncoderPosition > ELEVATOR_MAX && m_Input >= 0.0)) {
                        // When our input is zero we want to hold the current position with a closed loop velocity control
                        if (inputDifferent)
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 0.0);
                    } else {
                        // If our input is either up or down set to corresponding open loop pre-determined output
                        if (inputDifferent)
                            m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                                 m_Input > 0.0 ? ELEVATOR_UP_OUTPUT : ELEVATOR_DOWN_OUTPUT);
                    }
                    break;
                } else {
                    // Coast the elevator down so it does not slam at this height
                    LogSample(lib::LogLevel::k_Info, "Not High Enough");
                    m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                         m_IsLimitSwitchDown ? 0.0 : SAFE_ELEVATOR_DOWN_STRONG);
                }
            }
        }
    }

    int Elevator::GetElevatorPosition() {
        return m_EncoderPosition;
    }

    void Elevator::SetElevatorWantedPosition(int wantedPosition) {
        m_ControlMode = ElevatorControlMode::k_SetPoint;
        m_WantedSetPoint = wantedPosition;
    }

    void Elevator::SpacedUpdate(Command& command) {
        Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format(
                "Control Mode: %d, Wanted Set Point: %d, Encoder: %d, Real Output: %f, Current: %f, Limit Switch: %s",
                m_ControlMode, m_WantedSetPoint, m_EncoderPosition, m_ElevatorMaster.GetMotorOutputPercent(), m_ElevatorMaster.GetOutputCurrent(),
                m_IsLimitSwitchDown ? "true" : "false"));
    }

    bool Elevator::WithinPosition(int targetPosition) {
        return math::withinRange(m_EncoderPosition, targetPosition, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR / 2);
    }

    bool Elevator::ShouldUnlock(Command& command) {
        return math::abs(command.elevatorInput) > DEFAULT_INPUT_THRESHOLD;
    }

    void Elevator::OnUnlock() {
        m_ControlMode = m_DefaultControlMode;
    }
}