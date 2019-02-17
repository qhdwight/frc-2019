#pragma once

#include <lib/routine.hpp>

#include <pathfinder.h>

#include <vector>

#define MAX_SPEED 15.0

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
                                   0.001, MAX_SPEED, 10.0, 60.0, &trajectoryCandidate);
                const int length = trajectoryCandidate.length;
                Segment trajectory{};
                m_LeftTrajectory.reserve(length);
                m_RightTrajectory.reserve(length);
                pathfinder_generate(&trajectoryCandidate, &trajectory);
                pathfinder_modify_tank(&trajectory, length, m_LeftTrajectory.data(), m_RightTrajectory.data(), 0.6731);
            }

            void PrepareEncoder() {
                m_LeftFollower = m_RightFollower = {};
                m_LeftEncoderConfig = m_RightEncoderConfig = {0, 4096, 0.4787787204060999, 1.0, 0.0, 0.0,
                                                              1.0 / MAX_SPEED, 0.0};
            }

        public:
            AutoRoutine() {
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