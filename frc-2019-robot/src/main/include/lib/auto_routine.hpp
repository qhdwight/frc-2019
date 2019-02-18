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

            virtual void GetWaypoints();

            void PrepareWaypoints();

            void PrepareEncoder();

        public:
            AutoRoutine(std::shared_ptr<Robot>& robot);

            void Begin() override;

            bool CheckFinished() override;

            void Terminate() override;

            void Update() override;
        };
    }
}