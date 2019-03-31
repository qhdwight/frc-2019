#include <routine/post_hatch_place_routine.hpp>

#include <routine/timed_drive_routine.hpp>

#include <robot.hpp>

namespace garage {
    PostHatchPlaceRoutine::PostHatchPlaceRoutine(std::shared_ptr<Robot> robot)
            : SequentialRoutine(robot, "Post Hatch Placement Routine", {
            std::make_shared<OpenHatchIntakeRoutine>(robot),
            std::make_shared<lib::WaitRoutine>(robot, 600l, "Wait for Hatch Intake Routine"),
            std::make_shared<TimedDriveRoutine>(robot, 800l, -0.08, "Back Out Routine")
    }) {

    }

    OpenHatchIntakeRoutine::OpenHatchIntakeRoutine(std::shared_ptr<Robot> robot)
            : Routine(robot, "Open Hatch Intake Routine") {

    }

    void OpenHatchIntakeRoutine::Start() {
        Routine::Start();
        auto hatchIntake = m_Robot->GetSubsystem<HatchIntake>();
        hatchIntake->SetIntakeOpen(true);
    }
}