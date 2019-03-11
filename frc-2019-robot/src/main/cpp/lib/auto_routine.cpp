#include <lib/auto_routine.hpp>

#include <robot.hpp>

#include <chrono>

namespace garage {
    namespace lib {
        AutoRoutine::AutoRoutine(std::shared_ptr<Robot>& robot, const std::string& name)
                : Routine(robot, name), m_Drive(robot->GetDrive()) {
        }

        void AutoRoutine::CalculatePath() {
            GetWaypoints();
            PrepareWaypoints();
            PrepareEncoder();
        }

        void AutoRoutine::GetWaypoints() {
            m_Waypoints = {};
        }

        void AutoRoutine::PrepareWaypoints() {
            auto start = std::chrono::high_resolution_clock::now();
            TrajectoryCandidate trajectoryCandidate;
            pathfinder_prepare(m_Waypoints.data(), static_cast<int>(m_Waypoints.size()), FIT_HERMITE_QUINTIC, PATHFINDER_SAMPLES_HIGH,
                               TIME_STEP, MAX_SPEED, MAX_ACCELERATION, MAX_JERK, &trajectoryCandidate);
            m_Drive->Log(Logger::LogLevel::k_Info, "Prepared Points");
            const int length = trajectoryCandidate.length;
            const auto reserveLength = static_cast<const unsigned long>(length);
            m_Drive->Log(Logger::LogLevel::k_Info, Logger::Format("Trajectory has %d points", reserveLength));
            std::vector<Segment> trajectory;
            trajectory.reserve(reserveLength);
            m_LeftTrajectory.reserve(reserveLength);
            m_RightTrajectory.reserve(reserveLength);
            pathfinder_generate(&trajectoryCandidate, trajectory.data());
            m_Drive->Log(Logger::LogLevel::k_Info, "Generated Trajectory");
            pathfinder_modify_tank(trajectory.data(), length, m_LeftTrajectory.data(), m_RightTrajectory.data(), WHEELBASE_DISTANCE);
            m_Drive->Log(Logger::LogLevel::k_Info, Logger::Format("%d", trajectory.size()));
            for (auto& meme : trajectory) {
                m_Drive->Log(Logger::LogLevel::k_Info, Logger::Format("%f, %f", meme.x, meme.velocity));
            }
            m_Drive->Log(Logger::LogLevel::k_Info, "Modified Trajectory for Tank Drive");
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            m_Drive->Log(Logger::LogLevel::k_Info, Logger::Format("Path generation took %d milliseconds", duration));
        }

        void AutoRoutine::PrepareEncoder() {
            m_LeftFollower = m_RightFollower = {0.0, 0.0, 0.0, 0, 0};
            m_LeftEncoderConfig = m_RightEncoderConfig = {0, TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE, 1.0, 0.0, 0.0,
                                                          1.0 / MAX_SPEED, 0.0};
        }

        void AutoRoutine::Begin() {
            Routine::Begin();
            m_Robot->GetDrive()->Lock();
        }

        void AutoRoutine::Update() {
            const int
                    leftEncoder = m_Drive->GetDiscreteLeftEncoderTicks(), rightEncoder = m_Drive->GetDiscreteRightEncoderTicks(),
                    size = static_cast<int>(m_LeftTrajectory.size());
            const double
                    leftOutput = pathfinder_follow_encoder(m_LeftEncoderConfig, &m_LeftFollower, m_LeftTrajectory.data(), size, leftEncoder),
                    rightOutput = pathfinder_follow_encoder(m_RightEncoderConfig, &m_RightFollower, m_RightTrajectory.data(), size, rightEncoder),
                    heading = m_Robot->GetDrive()->GetHeading(),
                    desiredHeading = r2d(m_LeftFollower.heading);
            double headingDelta = std::fmod(desiredHeading - heading, 360.0);
            if (std::fabs(headingDelta) > 180.0) {
                headingDelta = (headingDelta > 0) ? (headingDelta - 360.0) : (headingDelta + 360.0);
            }
            const double turn = 0.8 * (-1.0 / 80.0) * headingDelta;
            m_Drive->Log(Logger::LogLevel::k_Info,
                         Logger::Format("Left Output: %f, Right Output: %f, Left Encoder: %d, Right Encoder %d, Heading: %f", leftOutput, rightOutput,
                                        leftEncoder, rightEncoder, heading));
            m_Drive->SetDriveOutput(leftOutput + turn, rightOutput - turn);
        }

        bool AutoRoutine::CheckFinished() {
            return false;
        }

        void AutoRoutine::Terminate() {
            Routine::Terminate();
            m_Robot->GetDrive()->Unlock();
        }
    }
}
