#include <lib/sequential_routine.hpp>

namespace garage {
    namespace lib {
        bool SequentialRoutine::IsFinished() {
            bool allFinished = true;
            for (auto& routine : m_SubRoutines) {
                if (routine.first && !routine.second.IsFinished()) allFinished = false;
            }
            return allFinished;
        }
    }
}
