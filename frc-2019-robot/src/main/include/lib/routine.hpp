#pragma once

namespace garage {
    namespace lib {
        class Routine {
        public:
            virtual bool IsFinished() { return true; };
        };
    }
}
