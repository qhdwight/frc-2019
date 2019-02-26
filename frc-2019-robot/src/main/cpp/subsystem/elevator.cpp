#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : Subsystem(robot, "Elevator") {
        m_ElevatorMaster.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveOne.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveTwo.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveThree.ConfigFactoryDefault(CONFIG_TIMEOUT);
        /* Sensors and Limits */
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, SET_POINT_SLOT_INDEX, CONFIG_TIMEOUT);
        // Soft limit
        m_ElevatorMaster.ConfigForwardSoftLimitThreshold(ELEVATOR_MAX, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigForwardSoftLimitEnable(true, CONFIG_TIMEOUT);
        // Limit switch
        m_ElevatorMaster.ConfigReverseLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource_FeedbackConnector,
                                                        ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyClosed, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClearPositionOnLimitR(true, CONFIG_TIMEOUT);
        // Set brake mode
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        // Ramping
        m_ElevatorMaster.ConfigOpenloopRamp(ELEVATOR_OPEN_LOOP_RAMP, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClosedloopRamp(ELEVATOR_CLOSED_LOOP_RAMP, CONFIG_TIMEOUT);
        // Current limiting
        m_ElevatorMaster.ConfigPeakCurrentLimit(ELEVATOR_PEAK_CURRENT_LIMIT, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigContinuousCurrentLimit(ELEVATOR_CONTINOUS_CURRENT_LIMIT, CONFIG_TIMEOUT);
        m_ElevatorMaster.EnableCurrentLimit(false);
        // Voltage compensation
        m_ElevatorMaster.ConfigVoltageCompSaturation(ELEVATOR_VOLTAGE_SATURATION, CONFIG_TIMEOUT);
        m_ElevatorMaster.EnableVoltageCompensation(true);
        // Configure following and inversion
        m_ElevatorSlaveOne.Follow(m_ElevatorMaster);
        m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);
        m_ElevatorSlaveThree.Follow(m_ElevatorMaster);
        m_ElevatorMaster.SetInverted(ctre::phoenix::motorcontrol::InvertType::None);
        m_ElevatorSlaveOne.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveTwo.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        m_ElevatorSlaveThree.SetInverted(ctre::phoenix::motorcontrol::InvertType::FollowMaster);
        // Gains and Motion profiling
        m_ElevatorMaster.ConfigMotionAcceleration(ELEVATOR_ACCELERATION, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMotionCruiseVelocity(ELEVATOR_VELOCITY, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, ELEVATOR_F, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, ELEVATOR_P, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, ELEVATOR_D, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kI(SET_POINT_SLOT_INDEX, ELEVATOR_I, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_IntegralZone(SET_POINT_SLOT_INDEX, ELEVATOR_I_ZONE, CONFIG_TIMEOUT);
        // Safety
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(SET_POINT_SLOT_INDEX, 0.35, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigAllowableClosedloopError(SET_POINT_SLOT_INDEX, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, CONFIG_TIMEOUT);
        // Final enabling of limits
        m_ElevatorMaster.OverrideSoftLimitsEnable(true);
        m_ElevatorMaster.OverrideLimitSwitchesEnable(true);
        // Setup network table
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Acceleration", ELEVATOR_ACCELERATION);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/Velocity", ELEVATOR_VELOCITY);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/P", ELEVATOR_P);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/D", ELEVATOR_D);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/F", ELEVATOR_F);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/FF", ELEVATOR_FF);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/I", ELEVATOR_F);
        m_Robot->GetNetworkTable()->PutNumber("Elevator/I Zone", ELEVATOR_F);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Acceleration").AddListener([&](const nt::EntryNotification& notification) {
            auto acceleration = static_cast<int>(notification.value->GetDouble());
            m_ElevatorMaster.ConfigMotionAcceleration(acceleration, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator acceleration to %d", acceleration));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/Velocity").AddListener([&](const nt::EntryNotification& notification) {
            auto velocity = static_cast<int>(notification.value->GetDouble());
            m_ElevatorMaster.ConfigMotionCruiseVelocity(velocity, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator velocity to %d", velocity));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/F").AddListener([&](const nt::EntryNotification& notification) {
            auto f = notification.value->GetDouble();
            m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, f, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator F %f", f));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/FF").AddListener([&](const nt::EntryNotification& notification) {
            auto ff = notification.value->GetDouble();
            m_SetPointController->SetFeedForward(ff);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator F %f", ff));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/D").AddListener([&](const nt::EntryNotification& notification) {
            auto d = notification.value->GetDouble();
            m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, d, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator D %f", d));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/P").AddListener([&](const nt::EntryNotification& notification) {
            auto p = notification.value->GetDouble();
            m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, p, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator P %f", p));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/I").AddListener([&](const nt::EntryNotification& notification) {
            auto i = notification.value->GetDouble();
            m_ElevatorMaster.Config_kI(SET_POINT_SLOT_INDEX, i, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator I %f", i));
        }, NT_NOTIFY_UPDATE);
        m_Robot->GetNetworkTable()->GetEntry("Elevator/I Zone").AddListener([&](const nt::EntryNotification& notification) {
            auto i_zone = static_cast<int>(notification.value->GetDouble());
            m_ElevatorMaster.Config_IntegralZone(SET_POINT_SLOT_INDEX, i_zone, CONFIG_TIMEOUT);
            Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format("Changed elevator I Zone to: %d", i_zone));
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
        m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(SET_POINT_SLOT_INDEX);
        m_EncoderVelocity = m_ElevatorMaster.GetSelectedSensorVelocity(SET_POINT_SLOT_INDEX);
        if (m_Controller) {
            m_Controller->Control();
        } else {
            LogSample(lib::LogLevel::k_Warning, "No controller detected");
        }
    }

    void Elevator::SetElevatorWantedSetPoint(int wantedSetPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetWantedSetPoint(wantedSetPoint);
    }

    void Elevator::SpacedUpdate(Command& command) {
        Log(lib::LogLevel::k_Info, m_Robot->GetLogger()->Format(
                "Output: %f, Current: %f, Encoder Position: %d, Encoder Velocity: %d",
                m_ElevatorMaster.GetMotorOutputPercent(), m_ElevatorMaster.GetOutputCurrent(), m_EncoderPosition, m_EncoderVelocity));
    }

    bool Elevator::WithinPosition(int targetPosition) {
        return math::withinRange(m_EncoderPosition, targetPosition, ELEVATOR_WITHIN_SET_POINT_AMOUNT);
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

    void Elevator::SetRawOutput(double output) {
        SetController(m_RawController);
    }

    void RawElevatorController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.elevatorInput, JOYSTICK_THRESHOLD);
        m_Output = m_Input * 0.2;
    }

    void RawElevatorController::Control() {
        Log(lib::LogLevel::k_Info, m_Subsystem->GetLogger()->Format("Input Value: %f", m_Output));
        m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output);
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
                                                  ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, m_FeedForward);
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