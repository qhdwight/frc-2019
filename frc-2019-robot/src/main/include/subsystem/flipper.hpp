#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>
#include <lib/subsystem_controller.hpp>

#include <rev/CANSparkMax.h>

#define FLIPPER_SET_POINT_LOWER 3.0
#define FLIPPER_SET_POINT_UPPER 38.0

#define FLIPPER_LOWER 0.0
#define FLIPPER_UPPER 40.0

#define FLIPPER_P 4e-5
#define FLIPPER_I 1e-7
#define FLIPPER_D 7e-4
#define FLIPPER_I_ZONE 3.0
#define FLIPPER_FF 0.000156
#define FLIPPER_VELOCITY 4400.0
#define FLIPPER_ACCELERATION 2300.0
#define FLIPPER_ALLOWABLE_ERROR 0.05

namespace garage {
    class Flipper;

    using FlipperController = lib::SubsystemController<Flipper>;

    class RawFlipperController : public FlipperController {
    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;
    };

    class SetPointFlipperController : public FlipperController {
    private:
        double m_SetPoint = 0.0;

    public:
        using SubsystemController::SubsystemController;

        void Control(Command& command) override;
    };

    class Flipper : public lib::Subsystem {
    private:
        rev::CANSparkMax m_FlipperMaster{FLIPPER, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_FlipperController = m_FlipperMaster.GetPIDController();
        rev::CANEncoder m_Encoder = m_FlipperMaster.GetEncoder();
        rev::CANDigitalInput m_LimitSwitch = m_FlipperMaster.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        bool m_IsLimitSwitchDown = true;
        double m_EncoderPosition = 0.0, m_EncoderVelocity = 0.0;
        std::shared_ptr<FlipperController> m_Controller;
        std::shared_ptr<RawFlipperController> m_RawController;
        std::shared_ptr<SetPointFlipperController> m_SetPointController;

        friend class SetPointFlipperController;

    protected:
        void UpdateUnlocked(Command& command) override;

        void Update() override;

        void SpacedUpdate(Command& command) override;

        bool SetController(std::shared_ptr<FlipperController> controller);

    public:
        Flipper(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;

        void SetRawOutput(double output, bool forceSet = false);

        void SetSetPoint(double setPoint, bool forceSet = false);
    };
}
