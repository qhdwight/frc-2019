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
            std::shared_ptr<Controller> m_Controller, m_DefaultController;

            virtual bool SetController(std::shared_ptr<Controller> controller) {
                const bool different = controller != m_Controller;
                if (different) {
                    if (m_Controller) m_Controller->OnDisable();
                    m_Controller = controller;
                    Log(Logger::LogLevel::k_Info, Logger::Format("Setting controller to: %s", FMT_STR(m_Controller->GetName())));
                    m_NetworkTable->PutString("Controller", m_Controller ? m_Controller->GetName() : "None");
                    if (controller) controller->OnEnable();
                }
                return different;
            }

            virtual void AddController(std::shared_ptr<Controller> controller) {
                m_Controllers.push_back(controller);
            }

            virtual void AddDefaultController(std::shared_ptr<Controller> controller) {
                AddController(controller);
                m_DefaultController = controller;
            }

            void UpdateUnlocked(Command& command) override {
                if (m_Controller) {
                    m_Controller->ProcessCommand(command);
                } else {
                    LogSample(lib::Logger::LogLevel::k_Warning, "No controller detected");
                }
            }

            void OnUnlock() override {
                SetController(m_DefaultController);
            }

            void Reset() override {
                Subsystem::Reset();
                for (auto& controller : m_Controllers) {
                    controller->Reset();
                }
            }

            virtual void OnReset() {}

        public:
            using Subsystem::Subsystem;
        };
    }
}