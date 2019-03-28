#pragma once

#include <memory>
#include <string>

namespace garage {
    class Robot;
    namespace lib {
        class Subsystem;
        class Routine {
        protected:
            std::shared_ptr<Robot> m_Robot;
            std::string m_Name;
            bool m_IsFinished = true;

            virtual void Update() {}

            virtual bool CheckFinished() { return true; }

        public:
            Routine(std::shared_ptr<Robot>& robot, const std::string& name);

            virtual void PostInitialize() {}

            virtual void Start();

            virtual void Terminate();

            virtual bool Periodic();

            virtual bool ShouldTerminateBasedOnUnlock(std::shared_ptr<Subsystem> subsystem) {
                return true;
            }

            virtual bool IsFinished() {
                return m_IsFinished;
            }
        };
    }
}
