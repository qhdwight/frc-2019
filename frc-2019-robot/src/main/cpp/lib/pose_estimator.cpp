#include <lib/pose_estimator.hpp>

namespace garage {
    namespace lib {
        PoseEstimator::PoseEstimator(ctre::phoenix::motorcontrol::can::TalonSRX& left, ctre::phoenix::motorcontrol::can::TalonSRX& right,
                                     ctre::phoenix::sensors::PigeonIMU& gyro) : m_Left(left), m_Right(right), m_Gyro(gyro) {

        }

        lib::RobotPose PoseEstimator::Update() {
            const int leftTicks = m_Left.GetSelectedSensorPosition(), rightTicks = m_Right.GetSelectedSensorPosition(),
                    leftTicksDelta = leftTicks - m_LastSensorValues.leftTicks, rightTicksDelta = rightTicks - m_LastSensorValues.rightTicks;
            const double heading = m_Gyro.GetFusedHeading(), headingDelta = heading - m_LastSensorValues.heading;
            const double T = 69.0;
            const double theta = (rightTicksDelta - leftTicksDelta) / T;
            const double r = (T / 2) * (static_cast<double>(rightTicksDelta + leftTicksDelta)/(rightTicksDelta - leftTicksDelta));
            
        }
    }
}
