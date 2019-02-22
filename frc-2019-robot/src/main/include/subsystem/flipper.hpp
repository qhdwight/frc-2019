#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

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
    enum class FlipperControlMode {
        k_Manual, k_SetPoint
    };

    class Flipper : public lib::Subsystem {
    private:
        FlipperControlMode m_ControlMode = FlipperControlMode::k_SetPoint;
        rev::CANSparkMax m_Flipper{FLIPPER, rev::CANSparkMax::MotorType::kBrushless};
        rev::CANPIDController m_FlipperController = m_Flipper.GetPIDController();
        rev::CANEncoder m_Encoder = m_Flipper.GetEncoder();
        rev::CANDigitalInput m_LimitSwitch = m_Flipper.GetForwardLimitSwitch(rev::CANDigitalInput::LimitSwitchPolarity::kNormallyClosed);
        bool m_IsLimitSwitchDown = true, m_FirstLimitSwitchHit = true;
        double m_EncoderPosition = 0.0, m_SetPoint = 0.0, m_LastSetPoint = 0.0;

    protected:
        void ProcessCommand(Command& command) override;

        void Update() override;

        void SpacedUpdate(Command& command) override;

    public:
        Flipper(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;
    };
}
