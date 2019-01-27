#pragma once

#include <lib/robot_pose.hpp>

#include <garage_math/garage_math.hpp>

#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

namespace garage {
    namespace lib {
        struct SensorValues {
        public:
            int leftTicks, rightTicks;
            double heading;
        };
        class PoseEstimator {
        private:
            RobotPose m_RobotPose;
            SensorValues m_LastSensorValues;
        public:
            PoseEstimator(ctre::phoenix::motorcontrol::can::TalonSRX& left, ctre::phoenix::motorcontrol::can::TalonSRX& right,
                          ctre::phoenix::sensors::PigeonIMU& gyro);

            void Reset();

            RobotPose Update();

            ctre::phoenix::motorcontrol::can::TalonSRX &m_Left, &m_Right;
            ctre::phoenix::sensors::PigeonIMU& m_Gyro;
        };
    }
}