#pragma once

#include <hardware_map.hpp>

#include <lib/controllable_subsystem.hpp>

#include <rev/CANSparkMax.h>

#define OUTRIGGER_LOWER 0.0
#define OUTRIGGER_UPPER 90.0

#define OUTRIGGER_STOW_ANGLE 90.0
#define OUTRIGGER_FULL_EXTENDED_ANGLE 270.0

#define OUTRIGGER_RAMPING 0.25

#define OUTRIGGER_P 0.0
#define OUTRIGGER_I 0.0
#define OUTRIGGER_D 0.0
#define OUTRIGGER_MAX_ACCUM 0.0
#define OUTRIGGER_I_ZONE 0.0
#define OUTRIGGER_FF 0.0
#define OUTRIGGER_VELOCITY 2000.0
#define OUTRIGGER_ACCELERATION 1000.0
#define OUTRIGGER_ALLOWABLE_ERROR 0.0
#define OUTRIGGER_ANGLE_FF 0.0

#define OUTRIGGER_SET_POINT_PID_SLOT 0

namespace garage {
    class Outrigger;

    using OutriggerController = lib::SubsystemController<Outrigger>;

    class SetPointOutriggerController : public OutriggerController {
    protected:
        double m_SetPoint = OUTRIGGER_LOWER;

    public:
        SetPointOutriggerController(std::weak_ptr<Outrigger>& outrigger)
                : OutriggerController(outrigger, "Set Point Outrigger Controller") {}

        void SetSetPoint(double setPoint) {
            m_SetPoint = setPoint;
        }

        void Reset() override {
            m_SetPoint = 0.0;
        }

        void Control() override;
    };

    class RawOutriggerController : public OutriggerController {
    protected:
        double m_Output = 0.0;

    public:
        RawOutriggerController(std::weak_ptr<Outrigger>& outrigger)
                : OutriggerController(outrigger, "Raw Outrigger Controller") {}

        void SetRawOutput(double output) {
            m_Output = output;
        }

        void Reset() override {
            m_Output = 0.0;
        }

        void Control() override;
    };

    class Outrigger : public lib::ControllableSubsystem<Outrigger> {
        friend class RawOutriggerController;

        friend class SetPointOutriggerController;

    protected:
        rev::CANSparkMax
                m_OutriggerMaster{OUTRIGGER_ARM_MASTER, rev::CANSparkMax::MotorType::kBrushless},
                m_OutriggerSlave{OUTRIGGER_ARM_SLAVE, rev::CANSparkMax::MotorType::kBrushless},
                m_OutriggerWheel{OUTRIGGER_WHEEL, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_OutriggerController = m_OutriggerMaster.GetPIDController();
        rev::CANEncoder m_Encoder = m_OutriggerMaster.GetEncoder();
        double m_EncoderPosition = OUTRIGGER_UPPER, m_Angle = OUTRIGGER_STOW_ANGLE;
        std::shared_ptr<SetPointOutriggerController> m_SetPointController;
        std::shared_ptr<RawOutriggerController> m_RawController;

        void StopMotors();

        void Update() override;

        bool ShouldUnlock(Command& command) override;

    public:
        Outrigger(std::shared_ptr<Robot>& robot);

        void Reset() override;

        void OnPostInitialize() override;
    };
}