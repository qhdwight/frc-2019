#pragma once

#include <memory>

namespace garage {
    class Robot;
    namespace lib {
        class Routine {
        protected:
            std::shared_ptr<Robot> m_Robot;
        public:
            Routine(std::shared_ptr<Robot>& robot);

            virtual void Begin() {}

            virtual void Terminate() {}

            virtual bool CheckFinished() { return true; }

            virtual void Update() {}
        };
    }
}
