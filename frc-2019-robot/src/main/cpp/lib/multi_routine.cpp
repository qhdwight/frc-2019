#include <lib/multi_routine.hpp>

#include <algorithm>

namespace garage {
    namespace lib {
        MultiRoutine::MultiRoutine(std::shared_ptr<Robot> robot, std::string name, std::vector<std::shared_ptr<Routine>>&& routines)
            : Routine(robot, std::move(name)) {
            m_SubRoutines = routines;
        }

        void MultiRoutine::Begin() {
            Routine::Begin();
            for (auto& routine : m_SubRoutines)
                routine->Begin();
        }

        void MultiRoutine::Update() {
            for (auto& routine : m_SubRoutines)
                routine->Update();
        }

        bool MultiRoutine::CheckFinished() {
            return std::all_of(m_SubRoutines.begin(), m_SubRoutines.end(),
                               [](auto& routine) {
                                   return routine->CheckFinished();
                               });
        }
    }
}