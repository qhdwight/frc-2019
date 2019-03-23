#pragma once

#include <lib/auto_routine.hpp>

#define PATH_LENGTH 256

namespace garage {
    namespace lib {
        class AutoRoutineFromCSV : public lib::AutoRoutine {
        protected:
            const std::string m_Path;

        public:
            AutoRoutineFromCSV(std::shared_ptr<Robot>& robot, const std::string& path, const std::string& name)
                    : lib::AutoRoutine(robot, name), m_Path(path) {}

        protected:
            void PrepareWaypoints() override;
        };
    }
}