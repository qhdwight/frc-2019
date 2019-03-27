#pragma once

#include <command.hpp>

#include <lib/routine.hpp>

#include <deque>
#include <memory>
#include <utility>

namespace garage {
    class Robot;
    namespace lib {
        class RoutineManager {
        private:
            std::shared_ptr<Robot> m_Robot;
            std::deque<std::shared_ptr<Routine>> m_QueuedRoutines;
            std::shared_ptr<Routine> m_ActiveRoutine;
        public:
            RoutineManager(std::shared_ptr<Robot>& robot);

            void AddRoutine(std::shared_ptr<Routine> routine);

            void AddRoutinesFromCommand(Command& command);

            void Reset();

            void Update();

            std::weak_ptr<Routine> GetActiveRoutine();

            void TerminateAllRoutines();

            void TerminateActiveRoutine();
        };
    }
}