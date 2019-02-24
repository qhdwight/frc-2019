#pragma once

#include <subsystem/flipper.hpp>
#include <subsystem/ball_intake.hpp>

#include <lib/routine.hpp>
#include <lib/sequential_routine.hpp>

#include <memory>

namespace garage {
    class IntakeBallThenStowRoutine : public lib::Routine {
    private:
        std::shared_ptr<BallIntake> m_BallIntake;

        void Begin() override;

    public:
        IntakeBallThenStowRoutine(std::shared_ptr<Robot>& robot);
    };

    class StowRoutine : public lib::Routine {
    private:
        std::shared_ptr<Flipper> m_Flipper;

    public:
        StowRoutine(std::shared_ptr<Robot>& robot);
    };

    class BallIntakeRoutine : lib::SequentialRoutine {
    public:
        BallIntakeRoutine(std::shared_ptr<Robot>& robot);
    };
}