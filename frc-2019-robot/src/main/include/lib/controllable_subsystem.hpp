#pragma once

#include <lib/subsystem.hpp>
#include <lib/subsystem_controller.hpp>

#include <memory>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class ControllableSubsystem : public std::enable_shared_from_this<TSubsystem>, public Subsystem {
            using Controller=SubsystemController<TSubsystem>;

        protected:
            std::vector<std::shared_ptr<Controller>> m_Controllers;
            std::shared_ptr<Controller> m_Controller, m_UnlockedController, m_ResetController;

            virtual bool SetController(std::shared_ptr<Controller> controller) {
                const bool different = controller != m_Controller;
                if (different) {
                    if (controller != m_UnlockedController) {
                        Lock();
                    }
                    if (m_Controller) m_Controller->OnDisable();
                    m_Controller = controller;
                    Log(Logger::LogLevel::k_Info, Logger::Format("Setting controller to: %s", FMT_STR(m_Controller->GetName())));
                    m_NetworkTable->PutString("Controller", m_Controller ? m_Controller->GetName() : "None");
                    if (controller) controller->OnEnable();
                }
                return different;
            }

            virtual void AddController(std::shared_ptr<Controller> controller) {
                if (controller) {
                    m_Controllers.push_back(controller);
                } else {
                    Log(Logger::LogLevel::k_Error, "Trying to add a null controller");
                }
            }

            virtual void SetUnlockedController(std::shared_ptr<Controller> controller) {
                if (controller != m_UnlockedController) {
                    m_UnlockedController = controller;
                    // If we are unlocked and received this update, set to the new mode
                    if (!m_IsLocked && controller != m_Controller) {
                        SetController(controller);
                    }
                }
            }

            virtual void SetResetController(std::shared_ptr<Controller> controller) {
                if (controller != m_ResetController) {
                    m_ResetController = controller;
                }
            }

            void UpdateUnlocked(Command& command) override {
                if (m_Controller) {
                    m_Controller->ProcessCommand(command);
                } else {
                    LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
                }
            }

            void ResetUnlock() override {
                if (m_ResetController) {
                    SetController(m_ResetController);
                } else {
                    Subsystem::ResetUnlock();
                }
            }

        public:
            using Subsystem::Subsystem;

            void Reset() override {
                Subsystem::Reset();
                for (auto& controller : m_Controllers) {
                    controller->Reset();
                }
            }

            void Unlock() override {
                Subsystem::Unlock();
                SetController(m_UnlockedController);
            }
        };
    }
}