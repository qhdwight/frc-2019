#pragma once

#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <memory>

namespace garage {
    namespace lib {
        /**
         * Wrapper around the Limelight network table to provide a clean interface for grabbing values
         */
        class Limelight {
        public:
            enum class LedMode {
                k_Default = 0, k_Off = 1, k_Blink = 2, k_On = 3
            };

        protected:
            std::shared_ptr<nt::NetworkTable> m_NetworkTable;

        public:
            Limelight();

            bool HasTarget();

            double GetHorizontalAngleToTarget();

            double GetTargetPercentArea();

            double GetSkew();

            void SetLedMode(LedMode ledMode);

            void SetPipeline(int pipelineIndex);
        };
    }
}