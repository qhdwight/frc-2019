#include <test/test_elevator_routine.hpp>

#include <robot.hpp>

namespace garage {
    namespace test {
        void TestElevatorRoutine::Begin() {
            m_Robot->GetElevator()->Lock();
            m_Robot->GetElevator()->SetElevatorWantedPosition(m_Position);
        }

        void TestElevatorRoutine::Terminate() {
            m_Robot->GetElevator()->Unlock();
        }

        TestElevatorRoutine::TestElevatorRoutine(std::shared_ptr<Robot>& robot, int position) : Routine(robot), m_Position(position) {

        }
    }
}
