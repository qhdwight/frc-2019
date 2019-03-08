#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : lib::ControllableSubsystem<Elevator>(robot, "Elevator") {
        ConfigSpeedControllers();
        SetupNetworkTableEntries();
        // TODO think about more
        auto elevator = std::weak_ptr<Elevator>(shared_from_this());
        AddDefaultController(m_RawController = std::make_shared<RawElevatorController>(elevator));
        AddController(m_SetPointController = std::make_shared<SetPointElevatorController>(elevator));
        AddController(m_VelocityController = std::make_shared<VelocityElevatorController>(elevator));
        AddController(m_SoftLandController = std::make_shared<SoftLandElevatorController>(elevator));
    }

    void Elevator::ConfigSpeedControllers() {
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
                                                        ctre::phoenix::motorcontrol::LimitSwitchNormal_NormallyOpen, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClearPositionOnLimitR(true, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigForwardLimitSwitchSource(ctre::phoenix::motorcontrol::LimitSwitchSource_Deactivated,
                                                        ctre::phoenix::motorcontrol::LimitSwitchNormal_Disabled, CONFIG_TIMEOUT);
        // Brake mode
        m_ElevatorMaster.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveOne.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveTwo.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        m_ElevatorSlaveThree.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
        // Ramping
        m_ElevatorMaster.ConfigOpenloopRamp(ELEVATOR_OPEN_LOOP_RAMP, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigClosedloopRamp(ELEVATOR_CLOSED_LOOP_RAMP, CONFIG_TIMEOUT);
        // Current limiting
        m_ElevatorMaster.ConfigContinuousCurrentLimit(ELEVATOR_CONTINUOUS_CURRENT_LIMIT, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigPeakCurrentLimit(ELEVATOR_PEAK_CURRENT_LIMIT, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigPeakCurrentDuration(ELEVATOR_PEAK_CURRENT_DURATION, CONFIG_TIMEOUT);
        // Voltage compensation
        m_ElevatorMaster.ConfigVoltageCompSaturation(ELEVATOR_VOLTAGE_SATURATION, CONFIG_TIMEOUT);
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
        m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, ELEVATOR_P, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, ELEVATOR_D, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kI(SET_POINT_SLOT_INDEX, ELEVATOR_I, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMaxIntegralAccumulator(SET_POINT_SLOT_INDEX, ELEVATOR_MAX_I, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_IntegralZone(SET_POINT_SLOT_INDEX, ELEVATOR_I_ZONE, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, ELEVATOR_F, CONFIG_TIMEOUT);
        // Safety
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(SET_POINT_SLOT_INDEX, 0.5, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigAllowableClosedloopError(SET_POINT_SLOT_INDEX, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, CONFIG_TIMEOUT);

        /* Final enabling */
        m_ElevatorMaster.EnableVoltageCompensation(true);
        m_ElevatorMaster.EnableCurrentLimit(false);
        m_ElevatorMaster.OverrideSoftLimitsEnable(true);
        m_ElevatorMaster.OverrideLimitSwitchesEnable(false);
    }

    void Elevator::SetupNetworkTableEntries() {
        // Add listeners for each entry when a value is updated on the dashboard
        AddNetworkTableListener("Acceleration", ELEVATOR_ACCELERATION, [this](const int acceleration) {
            auto error = m_ElevatorMaster.ConfigMotionAcceleration(acceleration, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("Velocity", ELEVATOR_VELOCITY, [this](const int velocity) {
            auto error = m_ElevatorMaster.ConfigMotionCruiseVelocity(velocity, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("F", ELEVATOR_F, [this](const double f) {
            auto error = m_ElevatorMaster.Config_kF(SET_POINT_SLOT_INDEX, f, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("FF", ELEVATOR_FF, [this](const double ff) {
            m_FeedForward = ff;
            return true;
        });
        AddNetworkTableListener("P", ELEVATOR_P, [this](const double p) {
            auto error = m_ElevatorMaster.Config_kP(SET_POINT_SLOT_INDEX, p, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("D", ELEVATOR_D, [this](const double d) {
            auto error = m_ElevatorMaster.Config_kD(SET_POINT_SLOT_INDEX, d, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
    }

    void Elevator::UpdateUnlocked(Command& command) {
//        if (command.elevatorSoftLand) {
//            SetController(m_SoftLandController);
//        } else if (math::abs(command.elevatorInput) > DEFAULT_INPUT_THRESHOLD) {
//            SetController(m_RawController);
//        }
        if (m_Controller != m_SoftLandController)
            SetController(m_VelocityController);
        if (m_Controller)
            m_Controller->ProcessCommand(command);
    }

    void Elevator::Update() {
        // TODO add brownout detection and smart current monitoring, check sticky faults
        m_ElevatorMaster.GetStickyFaults(m_StickyFaults);
        if (m_StickyFaults.HasAnyFault()) {
            // Don't log reverse limit switch error
            if ((m_StickyFaults.ToBitfield() & ~(1 << 2)) != 0)
                lib::Logger::Log(lib::Logger::LogLevel::k_Error,
                                 lib::Logger::Format("Sticky Faults: %s", FMT_STR(m_StickyFaults.ToString())));
            m_ElevatorMaster.ClearStickyFaults();
        }
        m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(SET_POINT_SLOT_INDEX);
        m_EncoderVelocity = m_ElevatorMaster.GetSelectedSensorVelocity(SET_POINT_SLOT_INDEX);
        if (m_Controller) {
            m_Controller->Control();
        } else {
            LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
        }
    }

    void Elevator::SetWantedSetPoint(int wantedSetPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetWantedSetPoint(wantedSetPoint);
    }

    void Elevator::SpacedUpdate(Command& command) {
        double current = m_ElevatorMaster.GetOutputCurrent(), output = m_ElevatorMaster.GetMotorOutputPercent();
        m_NetworkTable->PutNumber("Encoder", m_EncoderPosition);
        m_NetworkTable->PutNumber("Current", current);
        m_NetworkTable->PutNumber("Output", output);
        Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format(
                "Output: %f, Current: %f, Encoder Position: %d, Encoder Velocity: %d",
                output, current, m_EncoderPosition, m_EncoderVelocity));
    }

    bool Elevator::WithinPosition(int targetPosition) {
        return math::withinRange(m_EncoderPosition, targetPosition, ELEVATOR_WITHIN_SET_POINT_AMOUNT);
    }

    bool Elevator::ShouldUnlock(Command& command) {
        return math::absolute(command.elevatorInput) > DEFAULT_INPUT_THRESHOLD;
    }

    void Elevator::SetRawOutput(double output) {
        SetController(m_RawController);
        m_RawController->SetRawOutput(output);
    }

    void Elevator::SetManual() {
        SetController(m_VelocityController);
    }

    void Elevator::OnReset() {
        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }

    void RawElevatorController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
        m_Output = m_Input * 0.2;
    }

    void RawElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Input Value: %f, Output Value: %f", m_Input, m_Output));
        elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output);
    }

    void RawElevatorController::Reset() {
        m_Input = 0.0;
        m_Output = 0.0;
    }

    void SetPointElevatorController::ProcessCommand(Command& command) {
        auto elevator = m_Subsystem.lock();
        if (!elevator->m_IsLocked) {
            const double input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
            m_WantedSetPoint += static_cast<int>(input * 5000.0);
        }
    }

    void SetPointElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        m_WantedSetPoint = math::clamp(m_WantedSetPoint, ELEVATOR_MIN_CLOSED_LOOP_HEIGHT, ELEVATOR_MAX);
        Log(lib::Logger::LogLevel::k_Info,
            lib::Logger::Format("Wanted Set Point: %d, Feed Forward: %f", m_WantedSetPoint, elevator->m_FeedForward));
        if (elevator->m_EncoderPosition < ELEVATOR_MAX) {
            elevator->LogSample(lib::Logger::LogLevel::k_Info, "Theoretically Okay and Working");
            elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic,
                                           m_WantedSetPoint,
                                           ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                                           elevator->m_FeedForward);
        } else {
            elevator->Log(lib::Logger::LogLevel::k_Error, "Too High");
            elevator->SetController(elevator->m_SoftLandController);
        }
    }

    void SetPointElevatorController::Reset() {
        m_WantedSetPoint = 0;
    }

    void VelocityElevatorController::ProcessCommand(Command& command) {
        // TODO add too high checking
        m_Input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
        m_WantedVelocity = m_Input * ELEVATOR_VELOCITY;
    }

    void VelocityElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        if (elevator->m_EncoderPosition < ELEVATOR_MAX && elevator->m_EncoderPosition > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT) {
            Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Wanted Velocity: %f", m_WantedVelocity));
            elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                                           m_WantedVelocity,
                                           ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                                           elevator->m_FeedForward);
        } else {
            elevator->Log(lib::Logger::LogLevel::k_Error, "Too High");
            elevator->SetController(elevator->m_SoftLandController);
        }
    }

    void VelocityElevatorController::Reset() {
        m_Input = 0.0;
        m_WantedVelocity = 0.0;
    }

    void SoftLandElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                       elevator->m_EncoderPosition > 1000 ? SAFE_ELEVATOR_DOWN : 0.0);
    }
}