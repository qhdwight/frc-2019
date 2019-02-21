#include <lib/multi_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        MultiRoutine::MultiRoutine(std::shared_ptr<Robot> robot, const std::string& name, std::vector<std::shared_ptr<Routine>>&& routines)
                : Routine(robot, name) {
            m_SubRoutines = routines;
        }

        MultiRoutine::MultiRoutine(std::shared_ptr<Robot> robot, const std::string& name, std::vector<std::shared_ptr<Routine>>& routines)
                : Routine(robot, name) {
            m_SubRoutines = routines;
        }
    }
}