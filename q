[1mdiff --git a/frc-2019-robot/src/main/cpp/subsystem/elevator.cpp b/frc-2019-robot/src/main/cpp/subsystem/elevator.cpp[m
[1mindex f092f58..34b75e5 100644[m
[1m--- a/frc-2019-robot/src/main/cpp/subsystem/elevator.cpp[m
[1m+++ b/frc-2019-robot/src/main/cpp/subsystem/elevator.cpp[m
[36m@@ -22,6 +22,7 @@[m [mnamespace garage {[m
 //        m_ElevatorMaster.EnableCurrentLimit(true);[m
 //        m_ElevatorMaster.ConfigPeakCurrentLimit(300, CONFIG_TIMEOUT);[m
 //        m_ElevatorMaster.ConfigContinuousCurrentLimit(100, CONFIG_TIMEOUT);[m
[32m+[m[32m        m_ElevatorMaster.EnableVoltageCompensation(true);[m
         // Configure following and inversion[m
         m_ElevatorSlaveOne.Follow(m_ElevatorMaster);[m
         m_ElevatorSlaveTwo.Follow(m_ElevatorMaster);[m
[36m@@ -75,11 +76,12 @@[m [mnamespace garage {[m
         m_SetPointController = std::make_shared<SetPointElevatorController>(elevator);[m
         m_HybridController = std::make_shared<HybridElevatorController>(elevator);[m
         m_SoftLandController = std::make_shared<SoftLandElevatorController>(elevator);[m
[31m-        SetController(m_SetPointController);[m
[32m+[m[32m        SetElevatorWantedSetPoint(0);[m
     }[m
 [m
     void Elevator::TeleopInit() {[m
[31m-[m
[32m+[m[32m        if (m_Controller)[m
[32m+[m[32m            m_Controller->Reset();[m
     }[m
 [m
     void Elevator::UpdateUnlocked(Command& command) {[m
[36m@@ -97,22 +99,21 @@[m [mnamespace garage {[m
 //        m_ElevatorMaster.GetStickyFaults(m_StickyFaults);[m
 //        m_ElevatorMaster.ClearStickyFaults();[m
         // Reset encoder with limit switch[m
[31m-        static bool s_FirstLimitSwitchDown = true;[m
         m_IsLimitSwitchDown = static_cast<bool>(m_ElevatorMaster.GetSensorCollection().IsRevLimitSwitchClosed());[m
[31m-        if (m_IsLimitSwitchDown && s_FirstLimitSwitchDown) {[m
[32m+[m[32m        if (m_IsLimitSwitchDown && m_FirstLimitSwitchDown) {[m
             /* This is the first update frame that our limit switch has been set to the down position */[m
             // We want to reset to encoder back to zero because we know we are at[m
             // the lowest position possible and the encoder has probably drifted over time[m
[31m-            auto error = m_ElevatorMaster.SetSelectedSensorPosition(SET_POINT_SLOT_INDEX);[m
[32m+[m[32m            auto error = m_ElevatorMaster.SetSelectedSensorPosition(0, SET_POINT_SLOT_INDEX);[m
             if (error == ctre::phoenix::ErrorCode::OKAY) {[m
                 Log(lib::LogLevel::k_Info, "Limit switch hit and encoder reset");[m
[31m-                s_FirstLimitSwitchDown = false;[m
[32m+[m[32m                m_FirstLimitSwitchDown = false;[m
             } else {[m
                 Log(lib::LogLevel::k_Error, m_Robot->GetLogger()->Format("CTRE Error: %d", error));[m
             }[m
         }[m
         if (!m_IsLimitSwitchDown)[m
[31m-            s_FirstLimitSwitchDown = true;[m
[32m+[m[32m            m_FirstLimitSwitchDown = true;[m
         m_EncoderPosition = m_ElevatorMaster.GetSelectedSensorPosition(SET_POINT_SLOT_INDEX);[m
         m_EncoderVelocity = m_ElevatorMaster.GetSelectedSensorVelocity(SET_POINT_SLOT_INDEX);[m
         if (m_Controller) {[m
[36m@@ -185,7 +186,8 @@[m [mnamespace garage {[m
                 // In middle zone[m
                 m_Subsystem->LogSample(lib::LogLevel::k_Info, "Theoretically Okay and Working");[m
                 if (m_WantedSetPoint != m_LastSetPointSet) {[m
[31m-                    m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint);[m
[32m+[m[32m                    m_Subsystem->m_ElevatorMaster.Set(ctre::phoenix::motorcontrol::ControlMode::MotionMagic, m_WantedSetPoint,[m
[32m+[m[32m                                                      ctre::phoenix::motorcontrol::DemandType::DemandType_ArbitraryFeedForward, ELEVATOR_FF);[m
                     m_LastSetPointSet = m_WantedSetPoint;[m
                 }[m
             } else {[m
[36m@@ -202,6 +204,11 @@[m [mnamespace garage {[m
         }[m
     }[m
 [m
[32m+[m[32m    void SetPointElevatorController::Reset() {[m
[32m+[m[32m        m_WantedSetPoint = 0;[m
[32m+[m[32m        m_LastSetPointSet = 0;[m
[32m+[m[32m    }[m
[32m+[m
     void HybridElevatorController::ProcessCommand(Command& command) {[m
         // TODO add too high checking[m
         m_Input = math::threshold(command.elevatorInput, JOYSTICK_THRESHOLD);[m
[1mdiff --git a/frc-2019-robot/src/main/include/lib/subsystem_controller.hpp b/frc-2019-robot/src/main/include/lib/subsystem_controller.hpp[m
[1mindex 066bdd5..43100be 100644[m
[1m--- a/frc-2019-robot/src/main/include/lib/subsystem_controller.hpp[m
[1m+++ b/frc-2019-robot/src/main/include/lib/subsystem_controller.hpp[m
[36m@@ -24,7 +24,7 @@[m [mnamespace garage {[m
 [m
             void Log(LogLevel logLevel, const std::string& log) {[m
                 auto subsystem = std::dynamic_pointer_cast<Subsystem>(m_Subsystem);[m
[31m-                subsystem->GetLogger()->Log(logLevel, subsystem->GetLogger()->Format(" [%s] %s", m_Name.c_str(), log.c_str()));[m
[32m+[m[32m                subsystem->Log(logLevel, subsystem->GetLogger()->Format(" [%s] %s", m_Name.c_str(), log.c_str()));[m
             }[m
 [m
             virtual void OnEnable() {[m
[36m@@ -32,9 +32,12 @@[m [mnamespace garage {[m
             }[m
 [m
             virtual void OnDisable() {[m
[32m+[m[32m                Reset();[m
                 Log(LogLevel::k_Info, "Disabled");[m
             }[m
 [m
[32m+[m[32m            virtual void Reset() {};[m
[32m+[m
             virtual void ProcessCommand(Command& command) {};[m
 [m
             virtual void Control() {};[m
[1mdiff --git a/frc-2019-robot/src/main/include/subsystem/elevator.hpp b/frc-2019-robot/src/main/include/subsystem/elevator.hpp[m
[1mindex 46c1633..0b35bbd 100644[m
[1m--- a/frc-2019-robot/src/main/include/subsystem/elevator.hpp[m
[1m+++ b/frc-2019-robot/src/main/include/subsystem/elevator.hpp[m
[36m@@ -21,6 +21,7 @@[m
 #define ELEVATOR_D 0.0[m
 //#define ELEVATOR_D ELEVATOR_P * 6.0[m
 #define ELEVATOR_F 1023.0 / ELEVATOR_VELOCITY[m
[32m+[m[32m#define ELEVATOR_FF 0.7[m
 [m
 #define ELEVATOR_ALLOWABLE_CLOSED_LOOP_ERROR 0[m
 [m
[36m@@ -67,6 +68,8 @@[m [mnamespace garage {[m
 [m
         void Control() override;[m
 [m
[32m+[m[32m        void Reset() override;[m
[32m+[m
         void SetWantedSetPoint(int wantedSetPoint) {[m
             m_WantedSetPoint = wantedSetPoint;[m
         }[m
[36m@@ -91,8 +94,16 @@[m [mnamespace garage {[m
     };[m
 [m
     class Elevator : public lib::Subsystem {[m
[31m-    private:[m
[31m-        bool m_IsLimitSwitchDown = true;[m
[32m+[m[32m        friend class RawElevatorController;[m
[32m+[m
[32m+[m[32m        friend class SetPointElevatorController;[m
[32m+[m
[32m+[m[32m        friend class HybridElevatorController;[m
[32m+[m
[32m+[m[32m        friend class SoftLandElevatorController;[m
[32m+[m
[32m+[m[32m    protected:[m
[32m+[m[32m        bool m_IsLimitSwitchDown = true, m_FirstLimitSwitchDown = true;[m
         int m_EncoderPosition, m_EncoderVelocity;[m
         ctre::phoenix::motorcontrol::StickyFaults m_StickyFaults;[m
         ctre::phoenix::motorcontrol::can::TalonSRX m_ElevatorMaster{ELEVATOR_MASTER};[m
[36m@@ -104,15 +115,6 @@[m [mnamespace garage {[m
         std::shared_ptr<HybridElevatorController> m_HybridController;[m
         std::shared_ptr<SoftLandElevatorController> m_SoftLandController;[m
 [m
[31m-        friend class RawElevatorController;[m
[31m-[m
[31m-        friend class SetPointElevatorController;[m
[31m-[m
[31m-        friend class HybridElevatorController;[m
[31m-[m
[31m-        friend class SoftLandElevatorController;[m
[31m-[m
[31m-    protected:[m
         bool ShouldUnlock(Command& command) override;[m
 [m
         void UpdateUnlocked(Command& command) override;[m
