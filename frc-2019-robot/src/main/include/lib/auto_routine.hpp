#pragma once

#include <lib/routine.hpp>

#include <pathfinder.h>

#include <vector>

#define MAX_SPEED 15.0
#define MAX_ACCELERATION 10.0
#define MAX_JERK 60.0
#define TIME_STEP 1.0 / 50.0
#define WHEELBASE_DISTANCE 0.6731
#define WHEEL_CIRCUMFERENCE 0.4787787204060999
#define TICKS_PER_REVOLUTION 4096

namespace garage {
    namespace lib {
        class AutoRoutine : public Routine {
        protected:
            std::vector<Waypoint> m_Waypoints;
            std::vector<Segment> m_LeftTrajectory, m_RightTrajectory;
            EncoderConfig m_LeftEncoderConfig, m_RightEncoderConfig;
            EncoderFollower m_LeftFollower, m_RightFollower;

            void GetWaypoints() {
                m_Waypoints = {
                        {0.0, 0.0, d2r(0.0)},
                        {5.0, 0.0, d2r(0.0)},
                        {5.0, 5.0, d2r(45.0)}
                };
            }

            void PrepareWaypoints() {
                TrajectoryCandidate trajectoryCandidate;
                pathfinder_prepare(m_Waypoints.data(), m_Waypoints.size(), FIT_HERMITE_QUINTIC, PATHFINDER_SAMPLES_HIGH,
                                   TIME_STEP, MAX_SPEED, MAX_ACCELERATION, MAX_JERK, &trajectoryCandidate);
                const int length = trajectoryCandidate.length;
                Segment trajectory{};
                m_LeftTrajectory.reserve(length);
                m_RightTrajectory.reserve(length);
                pathfinder_generate(&trajectoryCandidate, &trajectory);
                pathfinder_modify_tank(&trajectory, length, m_LeftTrajectory.data(), m_RightTrajectory.data(), WHEELBASE_DISTANCE);
            }

            void PrepareEncoder() {
                m_LeftFollower = m_RightFollower = {};
                m_LeftEncoderConfig = m_RightEncoderConfig = {0, TICKS_PER_REVOLUTION, WHEEL_CIRCUMFERENCE, 1.0, 0.0, 0.0,
                                                              1.0 / MAX_SPEED, 0.0};
            }

        public:
            AutoRoutine(std::shared_ptr<Robot>& robot) : Routine(robot) {
                GetWaypoints();
            }

            void Update() override {
                const int leftEncoder = 0, rightEncoder = 0;
                const double
                        leftOutput = pathfinder_follow_encoder(m_LeftEncoderConfig, &m_LeftFollower,
                                                               m_LeftTrajectory.data(), m_LeftTrajectory.size(),
                                                               leftEncoder),
                        rightOutput = pathfinder_follow_encoder(m_RightEncoderConfig, &m_RightFollower,
                                                                m_RightTrajectory.data(), m_RightTrajectory.size(),
                                                                rightEncoder);
            }
        };
    }
}