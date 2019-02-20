#include <lib/multi_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        MultiRoutine::MultiRoutine(std::shared_ptr<Robot> robot, std::string name, std::vector<std::shared_ptr<Routine>>&& routines)
                : Routine(robot, std::move(name)) {
            m_SubRoutines = routines;
        }

        MultiRoutine::MultiRoutine(std::shared_ptr<Robot> robot, std::string name, std::vector<std::shared_ptr<Routine>>& routines)
                : Routine(robot, std::move(name)) {
            m_SubRoutines = routines;
        }
    }
}