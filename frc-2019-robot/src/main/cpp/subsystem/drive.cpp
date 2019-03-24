#include <subsystem/drive.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

#include <cmath>

namespace garage {
    Drive::Drive(std::shared_ptr<Robot>& robot) : ControllableSubsystem(robot, "Drive") {
        m_LeftSlave.RestoreFactoryDefaults();
        m_RightSlave.RestoreFactoryDefaults();
        m_LeftMaster.RestoreFactoryDefaults();
        m_RightMaster.RestoreFactoryDefaults();
        m_RightMaster.SetInverted(true);
        // Following
        m_LeftSlave.Follow(m_LeftMaster);
        m_RightSlave.Follow(m_RightMaster);
        // Open loop ramping
        m_LeftMaster.SetOpenLoopRampRate(DRIVE_RAMPING);
        m_RightMaster.SetOpenLoopRampRate(DRIVE_RAMPING);
        // Input dead band
        m_LeftMaster.SetParameter(rev::CANSparkMax::ConfigParameter::kInputDeadband, DRIVE_INPUT_DEAD_BAND);
        m_RightMaster.SetParameter(rev::CANSparkMax::ConfigParameter::kInputDeadband, DRIVE_INPUT_DEAD_BAND);
        // Voltage compensation
        m_LeftMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        m_RightMaster.EnableVoltageCompensation(DEFAULT_VOLTAGE_COMPENSATION);
        StopMotors();
    }

    void Drive::OnPostInitialize() {
        auto drive = std::weak_ptr<Drive>(shared_from_this());
        AddController(m_ManualController = std::make_shared<ManualDriveController>(drive));
        AddController(m_AutoAlignController = std::make_shared<AutoAlignDriveController>(drive));
        SetUnlockedController(m_ManualController);
    }

    void Drive::Reset() {
        ControllableSubsystem::Reset();
        StopMotors();
    }

    void Drive::StopMotors() {
        m_LeftOutput = 0.0;
        m_RightOutput = 0.0;
        m_LeftMaster.Set(0.0);
        m_RightMaster.Set(0.0);
    }

    bool Drive::ShouldUnlock(Command& command) {
        return std::fabs(command.driveForward) > DEFAULT_INPUT_THRESHOLD ||
               std::fabs(command.driveTurn) > DEFAULT_INPUT_THRESHOLD;
    }

    void Drive::SpacedUpdate(Command& command) {
        const double
                leftOutput = m_LeftMaster.GetAppliedOutput(),
                rightOutput = m_RightMaster.GetAppliedOutput(),
                leftCurrent = m_LeftMaster.GetOutputCurrent(),
                rightCurrent = m_RightMaster.GetOutputCurrent();
        const double heading = m_Pigeon.GetFusedHeading(), fixedHeading = math::fixAngle(heading);
        m_NetworkTable->PutNumber("Gyro", fixedHeading);
        m_NetworkTable->PutNumber("Left Output", leftOutput);
        m_NetworkTable->PutNumber("Right Output", rightOutput);
        m_NetworkTable->PutNumber("Left Encoder", m_LeftEncoderPosition);
        m_NetworkTable->PutNumber("Left Current", leftCurrent);
        m_NetworkTable->PutNumber("Right Encoder", m_RightEncoderPosition);
        m_NetworkTable->PutNumber("Right Current", rightCurrent);
        Log(lib::Logger::LogLevel::k_Debug, lib::Logger::Format(
                "Left Output: %f, Right Output: %f, Left Current: %f, Right Current: %f",
                leftOutput, rightOutput, leftCurrent, rightCurrent));
    }

    void Drive::Update() {
        m_RightEncoderPosition = m_RightEncoder.GetPosition();
        m_LeftEncoderPosition = m_LeftEncoder.GetPosition();
        if (m_Controller) {
            m_Controller->Control();
        } else {
            LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
        }
        if (m_Robot->ShouldOutput()) {
            m_LeftMaster.Set(m_LeftOutput);
            m_RightMaster.Set(m_RightOutput);
        }
    }

    double Drive::GetHeading() {
        return m_Pigeon.GetFusedHeading();
    }

    void Drive::ResetGyroAndEncoders() {
        m_Pigeon.SetFusedHeading(0.0);
        m_LeftEncoder.SetPosition(0.0);
        m_RightEncoder.SetPosition(0.0);
    }

    void Drive::SetDriveOutput(double left, double right) {
        if (!m_IsLocked) {
            Lock();
        }
        m_LeftOutput = left;
        m_RightOutput = right;
    }

    double Drive::GetTilt() {
        double angles[3];
        m_Pigeon.GetYawPitchRoll(angles);
        return angles[1];
    }

    int Drive::GetDiscreteRightEncoderTicks() {
        return std::lround(m_RightEncoderPosition * 100.0);
    }

    int Drive::GetDiscreteLeftEncoderTicks() {
        return std::lround(m_LeftEncoderPosition * 100.0);
    }

    void Drive::AutoAlign() {
        SetController(m_AutoAlignController);
    }

    void ManualDriveController::Reset() {
        m_ForwardInput = 0.0;
        m_TurnInput = 0.0;
        m_OldTurnInput = 0.0;
        m_QuickStopAccumulator = 0.0;
        m_NegativeInertiaAccumulator = 0.0;
        m_IsQuickTurn = false;
    }

    void ManualDriveController::ProcessCommand(Command& command) {
        m_ForwardInput = command.driveForward;
        m_TurnInput = command.driveTurn;
        m_IsQuickTurn = command.isQuickTurn;
    }

    void ManualDriveController::Control() {
        double turnInput = m_TurnInput, forwardInput = m_ForwardInput;
        const double negativeInertia = turnInput - m_OldTurnInput;
        m_OldTurnInput = turnInput;
        double negativeInertiaScalar;
        if (turnInput * negativeInertia > 0.0) {
            negativeInertiaScalar = DRIVE_NEGATIVE_INERTIA_TURN_SCALAR;
        } else {
            if (std::fabs(turnInput) > DRIVE_NEGATIVE_INERTIA_THRESHOLD) {
                negativeInertiaScalar = DRIVE_NEGATIVE_INERTIA_FAR_SCALAR;
            } else {
                negativeInertiaScalar = DRIVE_NEGATIVE_INERTIA_CLOSE_SCALAR;
            }
        }
        const double negativeInertiaPower = negativeInertia * negativeInertiaScalar;
        m_NegativeInertiaAccumulator += negativeInertiaPower;
        turnInput += m_NegativeInertiaAccumulator;
        if (m_NegativeInertiaAccumulator > 1.0) {
            m_NegativeInertiaAccumulator -= 1.0;
        } else if (m_NegativeInertiaAccumulator < -1.0) {
            m_NegativeInertiaAccumulator += 1.0;
        } else {
            m_NegativeInertiaAccumulator = 0.0;
        }
        const double linearPower = forwardInput;
        double overPower, angularPower;
        if (m_IsQuickTurn) {
            if (std::fabs(linearPower) < DRIVE_QUICK_STOP_DEAD_BAND) {
                const double alpha = DRIVE_QUICK_STOP_WEIGHT;
                m_QuickStopAccumulator = (1 - alpha) * m_QuickStopAccumulator + alpha * math::clamp(turnInput, -1.0, 1.0) * DRIVE_QUICK_STOP_SCALAR;
            }
            overPower = 1.0;
            angularPower = turnInput;
        } else {
            overPower = 0.0;
            angularPower = std::fabs(forwardInput) * turnInput * DRIVE_SENSITIVITY - m_QuickStopAccumulator;
            if (m_QuickStopAccumulator > 1.0) {
                m_QuickStopAccumulator -= 1.0;
            } else if (m_QuickStopAccumulator < -1.0) {
                m_QuickStopAccumulator += 1.0;
            } else {
                m_QuickStopAccumulator = 0.0;
            }
        }
        double leftOutput = linearPower, rightOutput = linearPower;
        leftOutput += angularPower;
        rightOutput -= angularPower;
        if (leftOutput > 1.0) {
            rightOutput -= overPower * (leftOutput - 1.0);
            leftOutput = 1.0;
        } else if (rightOutput > 1.0) {
            leftOutput -= overPower * (rightOutput - 1.0);
            rightOutput = 1.0;
        } else if (leftOutput < -1.0) {
            rightOutput += overPower * (-1.0 - leftOutput);
            leftOutput = -1.0;
        } else if (rightOutput < -1.0) {
            leftOutput += overPower * (-1.0 - rightOutput);
            rightOutput = -1.0;
        }
        auto drive = m_Subsystem.lock();
        drive->m_LeftOutput = leftOutput;
        drive->m_RightOutput = rightOutput;
    }

    void AutoAlignDriveController::Control() {
        auto drive = m_Subsystem.lock();
        const double
                tx = m_LimelightTable->GetNumber("tx", 0.0),
                ta = m_LimelightTable->GetNumber("ta", 0.0),
                tv = m_LimelightTable->GetNumber("tv", 0.0);
//        drive->Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("%f, %f, %f, %f", tx, ty, ta, tv));
        double forwardOutput, turnOutput;
        if (tv < 1.0) {
            forwardOutput = 0.0;
            turnOutput = 0.0;
        } else {
            turnOutput = math::clamp(tx * VISION_TURN_P, -VISION_MAX_TURN, VISION_MAX_TURN);
            const double delta = VISION_DESIRED_TARGET_AREA - ta;
            const double thresholdDelta = math::absolute(delta) > VISION_AREA_THRESHOLD ? delta : 0.0;
            forwardOutput = math::clamp(thresholdDelta * VISION_FORWARD_P, -VISION_MAX_FORWARD, VISION_MAX_FORWARD);
        }
        drive->m_LeftOutput = forwardOutput + turnOutput;
        drive->m_RightOutput = forwardOutput - turnOutput;
    }
}
