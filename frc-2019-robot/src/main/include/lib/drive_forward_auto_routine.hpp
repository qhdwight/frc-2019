#pragma once

#include <lib/auto_routine.hpp>

namespace garage {
    namespace lib {
        class DriveForwardAutoRoutine : public AutoRoutine {
        protected:
            void GetWaypoints() override;

        public:
            using AutoRoutine::AutoRoutine;
        };
    }
}