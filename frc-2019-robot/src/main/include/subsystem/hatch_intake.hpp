#pragma once

#define SERVO_LOW 1000
#define SERVO_HIGH 2000

#include <hardware_map.hpp>

#include <lib/subsystem.hpp>

#include <frc/Servo.h>

#include <ctre/phoenix/motorcontrol/can/VictorSPX.h>

namespace garage {
    class HatchIntake : public lib::Subsystem {
    private:
        bool m_IntakeOpen;
        uint16_t m_ServoOutput;
        frc::Servo m_Servo {HATCH_SERVO};
    public:
        HatchIntake(std::shared_ptr<Robot>& robot);

        void ExecuteCommand(Command& command) override;
    };
}
