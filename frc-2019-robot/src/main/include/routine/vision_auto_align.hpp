#pragma once

#include <lib/subsystem_routine.hpp>

#include <networktables/NetworkTable.h>

#include <memory>

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
