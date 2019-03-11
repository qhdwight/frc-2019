#pragma once

#include <lib/routine.hpp>

#include <pathfinder.h>

#include <vector>

#define AUTO_P 1.0
#define AUTO_I 0.0
#define AUTO_D 0.0
#define AUTO_V 0.0
#define AUTO_A 0.0
#define AUTO_MAX_VELOCITY 15.0
#define AUTO_MAX_ACCELERATION 10.0
#define AUTO_MAX_JERK 60.0

#define AUTO_TIME_STEP (1.0 / 50.0)
#define AUTO_WHEELBASE_DISTANCE 0.6731
#define AUTO_WHEEL_CIRCUMFERENCE 0.4787787204060999
#define AUTO_TICKS_PER_REVOLUTION 600

namespace garage {
    class Drive;
    namespace lib {
        class AutoRoutine : public Routine {
        protected:
            std::shared_ptr<Drive> m_Drive;
            std::vector<Waypoint> m_Waypoints;
            int m_TrajectorySize;
            std::vector<Segment> m_LeftTrajectory, m_RightTrajectory;
            EncoderConfig m_LeftEncoderConfig, m_RightEncoderConfig;
            EncoderFollower m_LeftFollower, m_RightFollower;

            virtual void GetWaypoints();

            void PrepareWaypoints();

            void PrepareEncoder();

        public:
            AutoRoutine(std::shared_ptr<Robot>& robot, const std::string& name);

            void CalculatePath();

            void Begin() override;

            bool CheckFinished() override;

            void Terminate() override;

            void Update() override;
        };
    }
}