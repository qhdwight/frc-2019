#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/IterativeRobot.h>

class Robot : public frc::IterativeRobot {
public:
  void TeleopInit() override {
    m_LeftSPX.Follow(m_LeftSRX);
    m_RightSPX.Follow(m_RightSRX);
  }

  void TeleopPeriodic() override {
    double
      forward = -m_Stick.GetY(),
      turn = m_Stick.GetX();
    m_LeftSRX.Set(ControlMode::PercentOutput, forward + turn*2);
    m_RightSRX.Set(ControlMode::PercentOutput, forward + turn*2);
  }

private:
  frc::Joystick m_Stick { 0 };
  TalonSRX m_RightSRX { 0 }, m_LeftSRX { 1 };
  VictorSPX m_RightSPX { 2 }, m_LeftSPX { 3 };
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
