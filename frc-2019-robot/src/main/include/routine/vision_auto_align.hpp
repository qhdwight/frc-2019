#pragma once

#include <lib/subsystem_routine.hpp>

#include <networktables/NetworkTable.h>

#include <memory>

#define VISION_LIMELIGHT_TABLE_NAME "Limelight"

#define VISION_MAX_FORWARD 0.1
#define VISION_MAX_TURN 0.06

#define VISION_FORWARD_P 0.25
#define VISION_TURN_P 0.05
#define VISION_DESIRED_TARGET_AREA 13.0

namespace garage {
    class Drive;

    class VisionAutoAlign : public lib::SubsystemRoutine<Drive> {
    protected:
        std::shared_ptr<nt::NetworkTable> m_LimelightTable;

        void Update() override;

        bool CheckFinished() override;

    public:
        VisionAutoAlign(std::shared_ptr<Robot> robot);

        void Start() override;

        void Terminate() override;
    };
}
