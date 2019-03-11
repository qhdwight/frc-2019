#pragma once

#include <lib/auto_routine.hpp>

namespace garage {
    namespace test {
        class TestDriveAutoRoutine : public lib::AutoRoutine {
        protected:
            void GetWaypoints() override;

        public:
            using AutoRoutine::AutoRoutine;
        };
    }
}