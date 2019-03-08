#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem_controller.hpp>
#include <lib/controllable_subsystem.hpp>

#include <frc/Servo.h>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class HatchIntake : public lib::ControllableSubsystem<HatchIntake> {
    private:
        bool m_IntakeOpen = false;
        uint16_t m_ServoOutput = 0;
        frc::Servo m_Servo{HATCH_SERVO};

    protected:
        void UpdateUnlocked(Command& command) override;

        void Update() override;

    public:
        HatchIntake(std::shared_ptr<Robot>& robot);

        void TeleopInit() override;
    };
}
