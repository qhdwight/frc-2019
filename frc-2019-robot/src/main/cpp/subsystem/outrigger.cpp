#include <subsystem/outrigger.hpp>

namespace garage {

    Outrigger::Outrigger(std::shared_ptr<Robot>& robot) : Subsystem(robot) {

    }

    void Outrigger::ExecuteCommand(Command& command) {
        Subsystem::ExecuteCommand(command);
    }
}