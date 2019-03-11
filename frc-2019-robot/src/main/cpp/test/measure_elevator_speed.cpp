#include <test/measure_elevator_speed.hpp>

#include <numeric>
#include <algorithm>

namespace garage {
    namespace test {
        void MeasureElevatorSpeed::Begin() {
            Routine::Begin();
            m_Subsystem->SetRawOutput(m_Output);
        }

        void MeasureElevatorSpeed::Update() {
            double velocity = m_Subsystem->GetVelocity();
            m_Velocities.push_back(velocity);
        }

        void MeasureElevatorSpeed::Terminate() {
            Routine::Terminate();
            std::sort(m_Velocities.begin(), m_Velocities.end());
            const double medianVelocity = m_Velocities[m_Velocities.size() / 2];
            lib::Logger::Log(lib::Logger::LogLevel::k_Info, lib::Logger::Format("Median Velocity: %f", medianVelocity));
            m_Subsystem->Unlock();
        }

        bool MeasureElevatorSpeed::CheckFinished() {
            return m_Subsystem->GetPosition() > 120000;
        }
    }
}