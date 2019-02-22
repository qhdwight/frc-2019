#include <routine/set_elevator_position_routine.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    namespace test {
        SetElevatorPositionRoutine::SetElevatorPositionRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int position)
            : Routine(robot, name), m_Position(position) {

        }

        void SetElevatorPositionRoutine::Begin() {
            m_Robot->GetElevator()->Lock();
            m_Robot->GetElevator()->SetElevatorWantedPosition(m_Position);
        }

        void SetElevatorPositionRoutine::Terminate() {
            m_Robot->GetElevator()->Unlock();
        }

        bool SetElevatorPositionRoutine::CheckFinished() {
            return math::withinRange(m_Robot->GetElevator()->GetElevatorPosition(), m_Position, 1000);
        }
    }
}
