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
        m_ElevatorMaster.EnableVoltageCompensation(true);
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
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(SET_POINT_SLOT_INDEX, 0.3, CONFIG_TIMEOUT);
//        m_ElevatorMaster.ConfigAllowableClosedloopError(SET_POINT_SLOT_INDEX, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, CONFIG_TIMEOUT);
        // Setup network table
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
        // Setup controllers
        // TODO think about more
        auto elevator = std::shared_ptr<Elevator>(this, [](auto elevator) {});
        m_RawController = std::make_shared<RawElevatorController>(elevator);
        m_SetPointController = std::make_shared<SetPointElevatorController>(elevator);
        m_HybridController = std::make_shared<HybridElevatorController>(elevator);
        m_SoftLandController = std::make_shared<SoftLandElevatorController>(elevator);
//        SetElevatorWantedSetPoint(0);
    }

    void Elevator::TeleopInit() {
        Unlock();
        m_FirstLimitSwitchDown = true;
        if (m_Controller)
            m_Controller->Reset();
    }

    void Elevator::UpdateUnlocked(Command& command) {
//        if (command.elevatorSoftLand) {
//            SetController(m_SoftLandController);
//        } else if (math::abs(command.elevatorInput) > JOYSTICK_THRESHOLD) {
//            SetController(m_RawController);
//        }
        if (m_Controller)
            m_Controller->ProcessCommand(command);
    }

    void Elevator::Update() {
        // TODO add brownout detection and smart current monitoring, check sticky faults
//        m_ElevatorMaster.GetStickyFaults(m_StickyFaults);
//        m_ElevatorMaster.ClearStickyFaults();
        // Reset encoder with limit switch
        m_IsLimitSwitchDown = static_cast<bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());
        if (m_IsLimitSwitchDown && m_FirstLimitSwitchDown) {
            /* This is the first update frame that our limit switch has been set to the down position */
            // We want to reset to encoder back to zero because we know we are at
            // the lowest position possible and the encoder has probably drifted over time
            auto error = m_ElevatorMaster.SetSelectedSensorPosition(0, SET_POINT_SLOT_INDEX);
            if (error == ctre::phoenix::ErrorCode::OKAY) {
                Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");
                m_FirstLimitSwitchDown = false;
            } else {
                Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CTRE Error: %d", error));
            }
        }
        if (!m_IsLimitSwitchDown)
            m_FirstLimitSwitchDown = true;
        m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(SET_POINT_SLOT_INDEX);
        m_EncoderVelocity = m_ElevatorMaster.GetSelectedSensorVelocity(SET_POINT_SLOT_INDEX);
        if (m_Controller) {
            m_Controller->Control();
        } else {
            LogSample(lib::LogLevel::k_Warning, "No controller detected");
        }
    }

    int Elevator::GetElevatorPosition() {
        return m_EncoderPosition;
    }

    void Elevator::SetElevatorWantedSetPoint(int wantedSetPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetWantedSetPoint(wantedSetPoint);
    }

    void Elevator::SpacedUpdate(Command& command) {
        Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format(
                "Output: %f, Current: %f, Encoder Position: %d, Encoder Velocity: %d, Limit Switch: %s",
                m_ElevatorMaster.GetMotorOutputPercent(), m_ElevatorMaster.GetOutputCurrent(), m_EncoderPosition, m_EncoderVelocity,
                m_IsLimitSwitchDown ? "true" : "false"));
    }

    bool Elevator::WithinPosition(int targetPosition) {
        return math::withinRange(m_EncoderPosition, targetPosition, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR / 2);
    }

    bool Elevator::ShouldUnlock(Command& command) {
        return math::absolute(command.elevatorInput) > DEFAULT_INPUT_THRESHOLD;
    }

    void Elevator::OnUnlock() {

    }

    bool Elevator::SetController(std::shared_ptr<ElevatorController> controller) {
        bool different = controller != m_Controller;
        if (different) {
            if (m_Controller) m_Controller->OnDisable();
            m_Controller = controller;
            controller->OnEnable();
        }
        return different;
    }

    void RawElevatorController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.elevatorInput, JOYSTICK_THRESHOLD);
    }

    void RawElevatorController::Control() {
        Log(lib::LogLevel::k_Info, m_Subsystem->GetLogger()->Format("Input Value: %f", m_Input));
        m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Input * 0.2);
    }

    void SetPointElevatorController::ProcessCommand(Command& command) {
        if (!m_Subsystem->m_IsLocked) {
            const double input = math::threshold(command.elevatorInput, JOYSTICK_THRESHOLD);
            m_WantedSetPoint += static_cast<int>(input * 5000.0);
        }
    }

    void SetPointElevatorController::Control() {
        m_WantedSetPoint = math::clamp(m_WantedSetPoint, ELEVATOR_MIN, ELEVATOR_MAX);
        Log(lib::LogLevel::k_Info, m_Subsystem->GetLogger()->Format("Wanted Set Point: %d", m_WantedSetPoint));
        // Our set point is above a negligible amount
        if (m_Subsystem->m_EncoderPosition < ELEVATOR_MAX) {
            // In middle zone
            m_Subsystem->LogSample(lib::LogLevel::k_Info, "Theoretically Okay and Working");
            if (!m_LastSetPointSet || m_WantedSetPoint != m_LastSetPointSet) {
                m_Subsystem->LogSample(lib::LogLevel::k_Info, "Set Set Point");
                m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint,
                                                  ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, ELEVATOR_FF);
//                    m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint);
                m_LastSetPointSet = m_WantedSetPoint;
            }
        } else {
            // Too high and we must kill the elevator
            m_Subsystem->Log(lib::LogLevel::k_Error, "Too High");
            m_Subsystem->SetController(m_Subsystem->m_SoftLandController);
        }
    }

    void SetPointElevatorController::Reset() {
        m_WantedSetPoint = 0;
        m_LastSetPointSet.reset();
    }

    void HybridElevatorController::ProcessCommand(Command& command) {
        // TODO add too high checking
        m_Input = math::threshold(command.elevatorInput, JOYSTICK_THRESHOLD);
    }

    void HybridElevatorController::Control() {
        if (m_Subsystem->m_EncoderPosition > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
            /* We are a negligible amount above the bottom */
            // Test if our input is different from the last one so we avoid flooding the controller with requests
            const bool inputDifferent = math::absolute(m_Subsystem->m_LastCommand.elevatorInput - m_Input) > 0.5;
            if (m_Input == 0.0 || (m_Subsystem->m_EncoderPosition > ELEVATOR_MAX && m_Input >= 0.0)) {
                // When our input is zero we want to hold the current position with a closed loop velocity control
                if (inputDifferent)
                    m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, 0.0);
            } else {
                // If our input is either up or down set to corresponding open loop pre-determined output
                if (inputDifferent)
                    m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                                      m_Input > 0.0 ? ELEVATOR_UP_OUTPUT : ELEVATOR_DOWN_OUTPUT);
            }
        } else {
            // Coast the elevator down so it does not slam at this height
            m_Subsystem->LogSample(lib::LogLevel::k_Info, "Not High Enough");
//            m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
//                                              m_Subsystem->m_IsLimitSwitchDown ? 0.0 : SAFE_ELEVATOR_DOWN_STRONG);
            m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
        }
    }

    void SoftLandElevatorController::Control() {
        if (m_Subsystem->m_EncoderPosition > SOFT_LAND_ELEVATOR_POSITION_WEAK) {
            m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_WEAK);
        } else if (m_Subsystem->m_EncoderPosition > SOFT_LAND_ELEVATOR_POSITION_STRONG) {
            m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, SAFE_ELEVATOR_DOWN_STRONG);
        }
    }
}