#include <lib/limelight.hpp>

namespace garage {
    namespace lib {
        Limelight::Limelight() {
            m_NetworkTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
        }

        bool Limelight::HasTarget() {
            return m_NetworkTable->GetNumber("tv", 0.0) > 0.5;
        }

        double Limelight::GetTargetPercentArea() {
            return m_NetworkTable->GetNumber("ta", 0.0);
        }

        double Limelight::GetHorizontalAngleToTarget() {
            return m_NetworkTable->GetNumber("tx", 0.0);
        }

        void Limelight::SetLedMode(Limelight::LedMode ledMode) {
            m_NetworkTable->PutNumber("ledMode", static_cast<double>(ledMode));
        }

        void Limelight::SetPipeline(int pipelineIndex) {
            m_NetworkTable->PutNumber("pipeline", pipelineIndex);
        }
    }
}