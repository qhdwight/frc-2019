#pragma once

#include <lib/robot_pose.hpp>

#include <garage_math/garage_math.hpp>

#include <rev/CANSparkMax.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>

namespace garage {
    namespace lib {
        struct SensorValues {
        public:
            double leftTicks, rightTicks, heading;
        };

        class PoseEstimator {
        private:
            RobotPose m_RobotPose;
            SensorValues m_LastSensorValues;
        public:
            PoseEstimator(rev::CANEncoder& left, rev::CANEncoder& right,
                          ctre::phoenix::sensors::PigeonIMU& gyro);

            void Reset();

            RobotPose Update();

            rev::CANEncoder& m_Left, & m_Right;
            ctre::phoenix::sensors::PigeonIMU& m_Gyro;
        };
    }
}