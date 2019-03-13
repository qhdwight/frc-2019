#pragma once

#include <memory>
#include <string>

namespace garage {
    class Robot;
    namespace lib {
        class Routine {
        protected:
            std::shared_ptr<Robot> m_Robot;
            std::string m_Name;
            bool m_Running = false;

            virtual void Update() {}

        public:
            Routine(std::shared_ptr<Robot>& robot, const std::string& name);

            virtual void Start();

            virtual void Terminate();

            virtual bool CheckFinished() { return true; }

            virtual void Periodic();
        };
    }
}
