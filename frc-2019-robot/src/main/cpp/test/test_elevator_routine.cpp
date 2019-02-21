#include <test/test_elevator_routine.hpp>

#include <robot.hpp>

#include <garage_math/garage_math.hpp>

namespace garage {
    namespace test {
        TestElevatorRoutine::TestElevatorRoutine(std::shared_ptr<Robot>& robot, const std::string& name, int position)
            : Routine(robot, std::move(name)), m_Position(position) {

        }

        void TestElevatorRoutine::Begin() {
            m_Robot->GetElevator()->Lock();
            m_Robot->GetElevator()->SetElevatorWantedPosition(m_Position);
        }

        void TestElevatorRoutine::Terminate() {
            m_Robot->GetElevator()->Unlock();
        }

        bool TestElevatorRoutine::CheckFinished() {
            return math::withinRange(m_Robot->GetElevator()->GetElevatorPosition(), m_Position, 1000);
        }
    }
}
