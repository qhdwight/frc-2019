#include <lib/auto_routine.hpp>

#include <robot.hpp>

#include <chrono>

namespace garage {
    namespace lib {
        AutoRoutine::AutoRoutine(std::shared_ptr<Robot>& robot, const std::string& name)
                : SubsystemRoutine(robot, robot->GetDrive(), name) {
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
                               AUTO_TIME_STEP, AUTO_MAX_VELOCITY, AUTO_MAX_ACCELERATION, AUTO_MAX_JERK, &trajectoryCandidate);
            m_Subsystem->Log(Logger::LogLevel::k_Info, "Prepared Points");
            m_TrajectorySize = trajectoryCandidate.length;
            const auto reserveLength = static_cast<const unsigned long>(m_TrajectorySize);
            m_Subsystem->Log(Logger::LogLevel::k_Info, Logger::Format("Trajectory has %d points", reserveLength));
            std::vector<Segment> trajectory;
            trajectory.resize(reserveLength);
            m_LeftTrajectory.resize(reserveLength);
            m_RightTrajectory.resize(reserveLength);
            pathfinder_generate(&trajectoryCandidate, trajectory.data());
            m_Subsystem->Log(Logger::LogLevel::k_Info, "Generated Trajectory");
            pathfinder_modify_tank(trajectory.data(), m_TrajectorySize, m_LeftTrajectory.data(), m_RightTrajectory.data(), AUTO_WHEELBASE_DISTANCE);
            m_Subsystem->Log(Logger::LogLevel::k_Info, Logger::Format("%d", trajectory.size()));
            m_Subsystem->Log(Logger::LogLevel::k_Info, "Modified Trajectory for Tank Drive");
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            m_Subsystem->Log(Logger::LogLevel::k_Info, Logger::Format("Path generation took %d milliseconds", duration));
//            for (auto& segment : trajectory) {
//                m_Drive->Log(Logger::LogLevel::k_Info, Logger::Format("%f, %f, %f, %f, %f, %f, %f, %f, %f",
//                                                                      segment.dt, segment.x, segment.y, segment.position, segment.velocity,
//                                                                      segment.acceleration, segment.jerk, segment.heading));
//            }
        }

        void AutoRoutine::PrepareEncoder() {
            m_LeftEncoderConfig = m_RightEncoderConfig = {0, // Initial encoder position
                                                          AUTO_TICKS_PER_REVOLUTION, // Ticks per revolution
                                                          AUTO_WHEEL_CIRCUMFERENCE, // Wheel circumference in meters
                                                          AUTO_P, AUTO_I, AUTO_D, // P, I, D
                                                          AUTO_V, // Velocity factor
                                                          AUTO_A}; // Acceleration factor
//            m_Subsystem->Log(Logger::LogLevel::k_Info, Logger::Format("Left Encoder Config: %d, %d, %f, %f, %f, %f, %f, %f, %f",
//                                                                  m_LeftEncoderConfig.initial_position, m_LeftEncoderConfig.ticks_per_revolution,
//                                                                  m_LeftEncoderConfig.wheel_circumference, m_LeftEncoderConfig.kp,
//                                                                  m_LeftEncoderConfig.ki, m_LeftEncoderConfig.kd,
//                                                                  m_LeftEncoderConfig.kv, m_LeftEncoderConfig.ka));
//            m_Subsystem->Log(Logger::LogLevel::k_Info, Logger::Format("Right Encoder Config: %d, %d, %f, %f, %f, %f, %f, %f, %f",
//                                                                  m_RightEncoderConfig.initial_position, m_RightEncoderConfig.ticks_per_revolution,
//                                                                  m_RightEncoderConfig.wheel_circumference, m_RightEncoderConfig.kp,
//                                                                  m_RightEncoderConfig.ki, m_RightEncoderConfig.kd,
//                                                                  m_RightEncoderConfig.kv, m_RightEncoderConfig.ka));
        }

        void AutoRoutine::Begin() {
            Routine::Begin();
            m_Subsystem->Lock();
            m_LeftFollower = m_RightFollower = {0.0, 0.0, 0.0, 0, 0};
        }

        void AutoRoutine::Update() {
            const int
                    leftEncoder = m_Subsystem->GetDiscreteLeftEncoderTicks(), rightEncoder = m_Subsystem->GetDiscreteRightEncoderTicks();
            const double
                    leftOutput = pathfinder_follow_encoder(m_LeftEncoderConfig, &m_LeftFollower, m_LeftTrajectory.data(), m_TrajectorySize,
                                                           leftEncoder),
                    rightOutput = pathfinder_follow_encoder(m_RightEncoderConfig, &m_RightFollower, m_RightTrajectory.data(), m_TrajectorySize,
                                                            rightEncoder),
                    heading = m_Robot->GetDrive()->GetHeading(),
                    desiredHeading = r2d(m_LeftFollower.heading);
            double headingDelta = std::fmod(desiredHeading - heading, 360.0);
            if (std::fabs(headingDelta) > 180.0) {
                headingDelta = (headingDelta > 0) ? (headingDelta - 360.0) : (headingDelta + 360.0);
            }
            const double turn = 0.8 * (-1.0 / 80.0) * headingDelta;
            m_Subsystem->Log(Logger::LogLevel::k_Info,
                         Logger::Format("Left Output: %f, Right Output: %f, Left Encoder: %d, Right Encoder %d, Heading: %f, Heading Delta: %f",
                                        leftOutput, rightOutput, leftEncoder, rightEncoder, heading, headingDelta));
            m_Subsystem->SetDriveOutput(leftOutput, rightOutput);
        }

        bool AutoRoutine::CheckFinished() {
            return false;
        }

        void AutoRoutine::Terminate() {
            Routine::Terminate();
            m_Subsystem->Unlock();
        }
    }
}
