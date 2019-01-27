#include <lib/pose_estimator.hpp>

#include <cmath>

namespace garage {
    namespace lib {
        PoseEstimator::PoseEstimator(ctre::phoenix::motorcontrol::can::TalonSRX& left, ctre::phoenix::motorcontrol::can::TalonSRX& right,
                                     ctre::phoenix::sensors::PigeonIMU& gyro) : m_Left(left), m_Right(right), m_Gyro(gyro) {

        }

        void PoseEstimator::Reset() {
            m_Gyro.SetFusedHeading(0);
            m_RobotPose.position.data().fill(0.0);
            m_LastSensorValues = {};
        }

        lib::RobotPose PoseEstimator::Update() {
            const int leftTicks = m_Left.GetSelectedSensorPosition(), rightTicks = m_Right.GetSelectedSensorPosition(),
                    leftTicksDelta = leftTicks - m_LastSensorValues.leftTicks, rightTicksDelta = rightTicks - m_LastSensorValues.rightTicks;
            const double heading = m_Gyro.GetFusedHeading() * (M_PI / 180.0);
            if (std::abs(heading) > 0.01) {
                const double
                        theta = heading, halfDistanceBetweenWheels = 25.5 * (4096.0 / 18.8495559215),
                        sinTheta = std::sin(theta),
                        distanceOne = (2.0 * leftTicksDelta) / theta + halfDistanceBetweenWheels,
                        distanceTwo = (2.0 * rightTicksDelta) / theta - halfDistanceBetweenWheels,
                        distanceAverage = (distanceOne + distanceTwo) / 2.0,
                        alpha = M_PI - theta / 2.0,
                        xDelta = distanceAverage * sinTheta * std::cos(alpha),
                        yDelta = distanceAverage * sinTheta * std::sin(alpha);
                m_LastSensorValues = { leftTicks, rightTicks, heading };
                m_RobotPose.position.x() += xDelta;
                m_RobotPose.position.y() += yDelta;
            }
            return m_RobotPose;
        }
    }
}
