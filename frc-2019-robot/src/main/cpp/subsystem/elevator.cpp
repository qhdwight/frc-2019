#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <frc/DriverStation.h>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : lib::ControllableSubsystem<Elevator>(robot, "Elevator") {
        ConfigSpeedControllers();
    }

    void Elevator::ConfigSpeedControllers() {
        m_ElevatorMaster.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveOne.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveTwo.ConfigFactoryDefault(CONFIG_TIMEOUT);
        m_ElevatorSlaveThree.ConfigFactoryDefault(CONFIG_TIMEOUT);

        /* Sensors and Limits */
        m_ElevatorMaster.ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::QuadEncoder, ELEVATOR_MOTION_MAGIC_PID_SLOT,
                                                      CONFIG_TIMEOUT);
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
        m_ElevatorMaster.ConfigVoltageCompSaturation(DEFAULT_VOLTAGE_COMPENSATION, CONFIG_TIMEOUT);
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
        m_ElevatorMaster.Config_kP(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_P, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kD(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_D, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kI(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_I, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMaxIntegralAccumulator(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_MAX_ACCUM, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_IntegralZone(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_I_ZONE, CONFIG_TIMEOUT);
        m_ElevatorMaster.Config_kF(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_F, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigMotionSCurveStrength(ELEVATOR_S_CURVE_STRENGTH, CONFIG_TIMEOUT);
        m_ElevatorMaster.ConfigAllowableClosedloopError(ELEVATOR_MOTION_MAGIC_PID_SLOT, ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, CONFIG_TIMEOUT);
        // Safety
        m_ElevatorMaster.ConfigClosedLoopPeakOutput(ELEVATOR_MOTION_MAGIC_PID_SLOT, 0.75, CONFIG_TIMEOUT);

        /* Final enabling */
        m_ElevatorMaster.EnableVoltageCompensation(true);
        m_ElevatorMaster.EnableCurrentLimit(false);
        m_ElevatorMaster.OverrideSoftLimitsEnable(false);
        m_ElevatorMaster.OverrideLimitSwitchesEnable(true);

        m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, 0.0);
    }

    void Elevator::OnPostInitialize() {
        auto elevator = std::weak_ptr<Elevator>(shared_from_this());
        AddController(m_RawController = std::make_shared<RawElevatorController>(elevator));
        AddController(m_SetPointController = std::make_shared<SetPointElevatorController>(elevator));
        AddController(m_VelocityController = std::make_shared<VelocityElevatorController>(elevator));
        AddController(m_SoftLandController = std::make_shared<SoftLandElevatorController>(elevator));
        SetUnlockedController(m_RawController);
        SetResetController(m_SoftLandController);
        SetupNetworkTableEntries();
    }

    void Elevator::SetupNetworkTableEntries() {
        // Add listeners for each entry when a value is updated on the dashboard
        AddNetworkTableListener("Acceleration", ELEVATOR_ACCELERATION, [this](const int acceleration) {
            auto error = m_ElevatorMaster.ConfigMotionAcceleration(acceleration, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("Velocity", ELEVATOR_VELOCITY, [this](const int velocity) {
            m_MaxVelocity = velocity;
            auto error = m_ElevatorMaster.ConfigMotionCruiseVelocity(velocity, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("I", ELEVATOR_I, [this](const double i) {
            auto error = m_ElevatorMaster.Config_kI(ELEVATOR_MOTION_MAGIC_PID_SLOT, i, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("I Zone", ELEVATOR_I_ZONE, [this](const int iZone) {
            auto error = m_ElevatorMaster.Config_IntegralZone(ELEVATOR_MOTION_MAGIC_PID_SLOT, iZone, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("Max Accum", ELEVATOR_MAX_ACCUM, [this](const int maxAccum) {
            auto error = m_ElevatorMaster.ConfigMaxIntegralAccumulator(ELEVATOR_MOTION_MAGIC_PID_SLOT, maxAccum, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("F", ELEVATOR_F, [this](const double f) {
            auto error = m_ElevatorMaster.Config_kF(ELEVATOR_MOTION_MAGIC_PID_SLOT, f, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("FF", ELEVATOR_FF, [this](const double ff) {
            m_FeedForward = ff;
            return true;
        });
        AddNetworkTableListener("P", ELEVATOR_P, [this](const double p) {
            auto error = m_ElevatorMaster.Config_kP(ELEVATOR_MOTION_MAGIC_PID_SLOT, p, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
        AddNetworkTableListener("D", ELEVATOR_D, [this](const double d) {
            auto error = m_ElevatorMaster.Config_kD(ELEVATOR_MOTION_MAGIC_PID_SLOT, d, CONFIG_TIMEOUT);
            return error == ctre::phoenix::OK;
        });
    }

    void Elevator::Update() {
        // TODO add brownout detection and smart current monitoring
        m_ElevatorMaster.GetStickyFaults(m_StickyFaults);
        if (m_StickyFaults.HasAnyFault()) {
            // Don't log reverse limit switch error
            if ((m_StickyFaults.ToBitfield() & ~(1 << 2)) != 0) {
                lib::Logger::Log(lib::Logger::LogLevel::k_Error,
                                 lib::Logger::Format("Sticky Faults: %s", FMT_STR(m_StickyFaults.ToString())));
                m_ElevatorMaster.ClearStickyFaults();
            }
        }
        m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(ELEVATOR_MOTION_MAGIC_PID_SLOT);
        m_EncoderVelocity = m_ElevatorMaster.GetSelectedSensorVelocity(ELEVATOR_MOTION_MAGIC_PID_SLOT);
//        auto& driverStation = frc::DriverStation::GetInstance();
//        const double timeRemaining = driverStation.GetMatchTime();
//        const bool isTest = driverStation.IsTest();
//        if (timeRemaining < ELEVATOR_LAND_TIME && !isTest) {
//            SetWantedSetPoint(0);
//        }
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
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Output: %f, Current: %f, Encoder Position: %d, Encoder Velocity: %d",
                output, current, m_EncoderPosition, m_EncoderVelocity));
    }

    bool Elevator::WithinPosition(int targetPosition) {
        return math::withinRange(m_EncoderPosition, targetPosition, ELEVATOR_WITHIN_SET_POINT_AMOUNT);
    }

    void Elevator::UpdateUnlocked(Command& command) {
//        if (command.offTheBooksModeEnabled) {
//            SetUnlockedController(m_RawController);
//        } else {
//            SetUnlockedController(m_RawController);
//        }
        ControllableSubsystem::UpdateUnlocked(command);
    }

    bool Elevator::ShouldUnlock(Command& command) {
//        auto& driverStation = frc::DriverStation::GetInstance();
//        const double timeRemaining = driverStation.GetMatchTime();
//        const bool isTest = driverStation.IsTest();
//        return (timeRemaining > ELEVATOR_LAND_TIME && !isTest) &&
//               (command.elevatorInput > DEFAULT_INPUT_THRESHOLD ||
//                (command.elevatorInput < -DEFAULT_INPUT_THRESHOLD && m_EncoderPosition >= ELEVATOR_MIN_CLOSED_LOOP_HEIGHT));
        return (command.elevatorInput > DEFAULT_INPUT_THRESHOLD ||
                (command.elevatorInput < -DEFAULT_INPUT_THRESHOLD && m_EncoderPosition >= ELEVATOR_MIN_CLOSED_LOOP_HEIGHT));
    }

    void Elevator::SetRawOutput(double output) {
        SetController(m_RawController);
        m_RawController->SetRawOutput(output);
    }

    void Elevator::SoftLand() {
        Log(lib::Logger::LogLevel::k_Info, "Safe Land");
        SetController(m_SoftLandController);
    }

    void RawElevatorController::ProcessCommand(Command& command) {
        m_Input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
        m_Output = math::clamp(m_Input, 0.0, 0.65);
    }

    void RawElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Input Value: %f, Output Value: %f", m_Input, m_Output));
        if (elevator->m_Robot->ShouldOutput()) {
            elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output);
//            if ((elevator->m_EncoderPosition > ELEVATOR_MIN_RAW_HEIGHT || m_Output > 0.01) &&
//                elevator->m_EncoderPosition < ELEVATOR_MAX) {
//                elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output);
//            } else {
//                elevator->Log(lib::Logger::LogLevel::k_Warning, "Not in range for raw control");
//                elevator->SoftLand();
//            }
        }
    }

    void RawElevatorController::Reset() {
        m_Input = 0.0;
        m_Output = 0.0;
    }

    void SetPointElevatorController::ProcessCommand(Command& command) {
        auto elevator = m_Subsystem.lock();
        const double input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
        m_WantedSetPoint += static_cast<int>(input * 5000.0);
    }

    void SetPointElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        m_WantedSetPoint = math::clamp(m_WantedSetPoint, ELEVATOR_MIN, ELEVATOR_MAX_CLOSED_LOOP_HEIGHT);
        Log(lib::Logger::LogLevel::k_Debug,
            lib::Logger::Format("Wanted Set Point: %d, Feed Forward: %f", m_WantedSetPoint, elevator->m_FeedForward));
        if ((elevator->m_EncoderPosition > ELEVATOR_MIN_CLOSED_LOOP_HEIGHT || m_WantedSetPoint > ELEVATOR_MIN) &&
            elevator->m_EncoderPosition < ELEVATOR_MAX) {
            elevator->LogSample(lib::Logger::LogLevel::k_Debug, "Theoretically Okay and Working");
            if (elevator->m_Robot->ShouldOutput()) {
                elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic,
                                               m_WantedSetPoint,
                                               ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                                               elevator->m_FeedForward);
            }
        } else {
            elevator->Log(lib::Logger::LogLevel::k_Warning, "Not in closed loop range for set point");
            elevator->SoftLand();
        }
    }

    void SetPointElevatorController::Reset() {
        m_WantedSetPoint = 0;
    }

    void VelocityElevatorController::ProcessCommand(Command& command) {
        auto elevator = m_Subsystem.lock();
        m_Input = math::threshold(command.elevatorInput, DEFAULT_INPUT_THRESHOLD);
        m_WantedVelocity = m_Input * elevator->m_MaxVelocity;
    }

    void VelocityElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        if ((elevator->m_EncoderPosition > (ELEVATOR_MIN_CLOSED_LOOP_HEIGHT * 2) || m_WantedVelocity > 0.01) &&
            elevator->m_EncoderPosition < ELEVATOR_MAX) {
            if (elevator->m_EncoderPosition < ELEVATOR_MAX_CLOSED_LOOP_HEIGHT || m_WantedVelocity < -0.01) {
                Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format("Wanted Velocity: %f", m_WantedVelocity));
                if (elevator->m_Robot->ShouldOutput()) {
                    elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity,
                                                   m_WantedVelocity,
                                                   ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward,
                                                   elevator->m_FeedForward);
                }
            } else {
                elevator->Log(lib::Logger::LogLevel::k_Warning, "Trying to go too high");
                elevator->SoftLand();
            }
        } else {
            elevator->Log(lib::Logger::LogLevel::k_Warning, "Not in closed loop range for velocity");
            elevator->SoftLand();
        }
    }

    void VelocityElevatorController::Reset() {
        m_Input = 0.0;
        m_WantedVelocity = 0.0;
    }

    void SoftLandElevatorController::Control() {
        auto elevator = m_Subsystem.lock();
        if (elevator->m_Robot->ShouldOutput()) {
            elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput,
                                           elevator->m_EncoderPosition > 500 ? ELEVATOR_SAFE_DOWN : 0.0);
        }
    }
}