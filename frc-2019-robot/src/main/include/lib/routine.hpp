#pragma once

namespace garage {
    namespace lib {
        class Routine {
        public:
            virtual void Begin() {}

            virtual void Terminate() {}

            virtual bool CheckFinished() { return true; }

            virtual void Update() {}
        };
    }
}
