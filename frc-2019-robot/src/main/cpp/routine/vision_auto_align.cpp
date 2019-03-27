#include <routine/vision_auto_align.hpp>

#include <robot.hpp>

namespace garage {
    void VisionAutoAlign::Start() {
    }

    void VisionAutoAlign::Terminate() {
        Routine::Terminate();
        m_Subsystem->Unlock();
    }

    bool VisionAutoAlign::CheckFinished() {
        return false;
    }

    void VisionAutoAlign::Update() {
        
    }
}
