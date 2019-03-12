#pragma once

#include <lib/routine.hpp>

#include <vector>
#include <memory>

namespace garage {
    namespace lib {
        using RoutineVector=std::vector<std::shared_ptr<Routine>>;

        class MultiRoutine : public Routine {
        protected:
            RoutineVector m_SubRoutines;
        public:
            MultiRoutine(std::shared_ptr<Robot> robot, const std::string& name, RoutineVector&& routines);

            MultiRoutine(std::shared_ptr<Robot> robot, const std::string& name, RoutineVector& routines);
        };
    }
}