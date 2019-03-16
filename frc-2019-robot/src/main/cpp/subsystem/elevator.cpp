#include <subsystem/elevator.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <frc/DriverStation.h>

namespace garage {
    Elevator::Elevator(std::shared_ptr<Robot>& robot) : lib::ControllableSubsystem<Elevator>(robot, "Elevator") {
        ConfigSpeedControllers();
    }

    void Elevator::ConfigSpeedControllers() {
        m_SparkMaster.RestoreFactoryDefaults();
        m_SparkSlave.RestoreFactoryDefaults();
        m_SparkSlave.Follow(m_SparkMaster, false);
        m_SparkMaster.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_SparkSlave.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
        m_SparkMaster.SetClosedLoopRampRate(ELEVATOR_CLOSED_LOOP_RAMP);
        m_SparkMaster.SetOpenLoopRampRate(ELEVATOR_OPEN_LOOP_RAMP);
        m_SparkController.SetP(ELEVATOR_P, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetI(ELEVATOR_I, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetD(ELEVATOR_D, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetIZone(ELEVATOR_I_ZONE, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetIMaxAccum(ELEVATOR_MAX_ACCUM, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetFF(ELEVATOR_F, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetOutputRange(-1.0, 1.0, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetSmartMotionMaxVelocity(ELEVATOR_VELOCITY, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetSmartMotionMinOutputVelocity(0.0, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetSmartMotionMaxAccel(ELEVATOR_ACCELERATION, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetSmartMotionAllowedClosedLoopError(ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkController.SetSmartMotionAccelStrategy(rev::CANPIDController::AccelStrategy::kSCurve, ELEVATOR_CLOSED_LOOP_SLOT);
        m_SparkMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_ReverseLimitSwitch.EnableLimitSwitch(true);
        m_SparkMaster.Set(0.0);
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
        AddNetworkTableListener("Acceleration", ELEVATOR_ACCELERATION, [this](const double acceleration) {
            auto error = m_SparkController.SetSmartMotionMaxAccel(acceleration, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("Velocity", ELEVATOR_VELOCITY, [this](const int velocity) {
            m_MaxVelocity = velocity;
            auto error = m_SparkController.SetSmartMotionMinOutputVelocity(velocity, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("I", ELEVATOR_I, [this](const double i) {
            auto error = m_SparkController.SetI(i, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("I Zone", ELEVATOR_I_ZONE, [this](const int iZone) {
            auto error = m_SparkController.SetIZone(iZone, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("Max Accum", ELEVATOR_MAX_ACCUM, [this](const int maxAccum) {
            auto error = m_SparkController.SetIMaxAccum(maxAccum, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("F", ELEVATOR_F, [this](const double f) {
            auto error = m_SparkController.SetFF(f, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("FF", ELEVATOR_FF, [this](const double ff) {
            m_FeedForward = ff;
            return true;
        });
        AddNetworkTableListener("P", ELEVATOR_P, [this](const double p) {
            auto error = m_SparkController.SetP(p, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
        AddNetworkTableListener("D", ELEVATOR_D, [this](const double d) {
            auto error = m_SparkController.SetD(d, ELEVATOR_CLOSED_LOOP_SLOT);
            return error == rev::CANError::kOK;
        });
    }

    void Elevator::Update() {
        // TODO add brownout detection and smart current monitoring
        if (m_ReverseLimitSwitch.Get()) {
            if (m_IsFirstLimitSwitchHit) {
                m_IsFirstLimitSwitchHit = false;
                m_Encoder.SetPosition(0.0);
            }
        } else {
            m_IsFirstLimitSwitchHit = true;
        }
        m_EncoderPosition = m_Encoder.GetPosition();
        m_EncoderVelocity = m_Encoder.GetVelocity();
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
        uint16_t faults = m_SparkMaster.GetStickyFaults();
        // Ignore reverse and forward limit switch faults
        faults &= ~(1 << 14);
        faults &= ~(1 << 15);
        if (faults) {
            lib::Logger::Log(lib::Logger::LogLevel::k_Error,
                             lib::Logger::Format("Sticky Faults: %d", faults));
            m_SparkMaster.ClearFaults();
        }
    }

    void Elevator::SetWantedSetPoint(double wantedSetPoint) {
        SetController(m_SetPointController);
        m_SetPointController->SetWantedSetPoint(wantedSetPoint);
    }

    void Elevator::SpacedUpdate(Command& command) {
        double current = m_SparkMaster.GetOutputCurrent(), output = m_SparkMaster.GetAppliedOutput();
        m_NetworkTable->PutNumber("Encoder", m_EncoderPosition);
        m_NetworkTable->PutNumber("Current", current);
        m_NetworkTable->PutNumber("Output", output);
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Output: %f, Current: %f, Encoder Position: %d, Encoder Velocity: %d",
                output, current, m_EncoderPosition, m_EncoderVelocity));
    }

    bool Elevator::WithinPosition(double targetPosition) {
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
            elevator->m_SparkMaster.Set(m_Output);
//            elevator->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, m_Output);
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
                elevator->m_SparkController.SetReference(m_WantedSetPoint, rev::ControlType::kSmartMotion,
                                                         ELEVATOR_CLOSED_LOOP_SLOT, elevator->m_FeedForward);
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
                    elevator->m_SparkController.SetReference(m_WantedVelocity, rev::ControlType::kSmartVelocity,
                                                             ELEVATOR_CLOSED_LOOP_SLOT, elevator->m_FeedForward);
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
            elevator->m_SparkMaster.Set(elevator->m_EncoderPosition > 500 ? ELEVATOR_SAFE_DOWN : 0.0);
        }
    }
}