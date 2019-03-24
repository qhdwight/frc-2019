#include <routine/vision_auto_align.hpp>

#include <robot.hpp>

namespace garage {
    VisionAutoAlign::VisionAutoAlign(std::shared_ptr<Robot> robot)
            : SubsystemRoutine(robot, "Vision Auto Align"), m_LimelightTable(robot->GetNetworkTable()->GetInstance().GetTable(VISION_LIMELIGHT_TABLE_NAME)) {

    }

    void VisionAutoAlign::Start() {
        Routine::Start();
        m_Subsystem->Lock();
    }

    void VisionAutoAlign::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool VisionAutoAlign::CheckFinished() {
        return false;
    }

    void VisionAutoAlign::Update() {
        const double
                tx = m_LimelightTable->GetNumber("tx", 0.0),
                ty = m_LimelightTable->GetNumber("ty", 0.0),
                ta = m_LimelightTable->GetNumber("ta", 0.0),
                tv = m_LimelightTable->GetNumber("tv", 0.0);
        m_Subsystem->Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("%f, %f, %f, %f", tx, ty, ta, tv));
        double forwardOutput, turnOutput;
        if (tv < 1.0) {
            forwardOutput = 0.0;
            turnOutput = 0.0;
        } else {
            turnOutput = math::clamp(tx * VISION_TURN_P, -VISION_MAX_TURN, VISION_MAX_TURN);
            const double delta = VISION_DESIRED_TARGET_AREA - ta;
            const double thresholdDelta = math::absolute(delta) > VISION_AREA_THRESHOLD ? delta : 0.0;
            forwardOutput = math::clamp(thresholdDelta * VISION_FORWARD_P, -VISION_MAX_FORWARD, VISION_MAX_FORWARD);
        }
        m_Subsystem->SetDriveOutput(forwardOutput + turnOutput, forwardOutput - turnOutput);
    }
}
