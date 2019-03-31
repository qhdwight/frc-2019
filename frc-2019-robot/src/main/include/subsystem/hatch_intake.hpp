#pragma once

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <frc/Servo.h>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

#define HATCH_SERVO_LOWER 700
#define HATCH_SERVO_UPPER 1700

namespace garage {
    class HatchIntake : public lib::Subsystem {
    protected:
        bool m_IntakeOpen = false;
        uint16_t m_ServoOutput = HATCH_SERVO_LOWER;
        frc::Servo m_Servo{HATCH_SERVO};

        void UpdateUnlocked(Command& command) override;

        void Update() override;

        bool ShouldUnlock(Command& command) override;

    public:
        HatchIntake(std::shared_ptr<Robot>& robot);

        void Reset() override;

        void SetIntakeOpen(bool isOpen);
    };
}
