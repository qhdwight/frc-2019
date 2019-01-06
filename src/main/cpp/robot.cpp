#include <frc/Joystick.h>
#include <ctre/Phoenix.h>
#include <frc/IterativeRobot.h>

class Robot : public frc::IterativeRobot {
public:
  void TeleopInit() override {
    
  }

  void TeleopPeriodic() override {
  }

private:
  frc::Joystick m_Stick { 0 };
  TalonSRX m_LeftSRX { 0 }, m_RightSRX { 1 };
  VictorSPX m_LeftSPX { 2 }, m_RightSPX { 3 };
};

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
