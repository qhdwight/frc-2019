#include <lib/auto_routine.hpp>

#include <robot.hpp>
#include <subsystem/drive.hpp>

namespace garage {
    namespace lib {
        AutoRoutine::AutoRoutine(std::shared_ptr<Robot>& robot, const std::string& name) : Routine(robot, name) {
            GetWaypoints();
            PrepareWaypoints();
            PrepareEncoder();
        }

        void AutoRoutine::GetWaypoints() {
            m_Waypoints = { };
        }

        void AutoRoutine::PrepareWaypoints() {
            TrajectoryCandidate trajectoryCandidate;
            pathfinder_prepare(m_Waypoints.data(), static_cast<int>(m_Waypoints.size()), FIT_HERMITE_QUINTIC, PATHFINDER_SAMPLES_HIGH,
                               TIME_STEP, MAX_SPEED, MAX_ACCELERATION, MAX_JERK, &trajectoryCandidate);
            const int length = trajectoryCandidate.length;
            const auto reserveLength = static_cast<const unsigned long>(length);
            Segment trajectory{};
            m_LeftTrajectory.reserve(reserveLength);
            m_RightTrajectory.reserve(reserveLength);
            pathfinder_generate(&trajectoryCandidate, &trajectory);
            pathfinder_modify_tank(&trajectory, length, m_LeftTrajectory.data(), m_RightTrajectory.data(), WHEELBASE_DISTANCE);
        }

        void AutoRoutine::PrepareEncoder() {
            m_LeftFollower = m_RightFollower = {};
            m_LeftEncoderConfig = m_RightEncoderConfig = {0, TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE, 1.0, 0.0, 0.0,
                                                          1.0 / MAX_SPEED, 0.0};
        }

        void AutoRoutine::Begin() {
            Routine::Begin();
            m_Robot->GetDrive()->Lock();
        }

        void AutoRoutine::Update() {
            const int leftEncoder = 0, rightEncoder = 0, size = static_cast<int>(m_LeftTrajectory.size());
            const double
                    leftOutput = pathfinder_follow_encoder(m_LeftEncoderConfig, &m_LeftFollower, m_LeftTrajectory.data(), size, leftEncoder),
                    rightOutput = pathfinder_follow_encoder(m_RightEncoderConfig, &m_RightFollower, m_RightTrajectory.data(), size, rightEncoder),
                    heading = r2d(m_Robot->GetDrive()->GetHeading()),
                    desiredHeading = m_LeftFollower.heading,
                    headingDelta = desiredHeading - heading,
                    turn = 0.8 * (-1.0/80.0) * headingDelta;
            m_Robot->GetDrive()->SetDriveOutput(leftOutput + turn, rightOutput - turn);
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
