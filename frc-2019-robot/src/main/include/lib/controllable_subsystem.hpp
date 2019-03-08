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
            std::shared_ptr<Controller> m_Controller;

            virtual bool SetController(std::shared_ptr<Controller> controller) {
                const bool different = controller != m_Controller;
                if (different) {
                    if (m_Controller) m_Controller->OnDisable();
                    m_Controller = controller;
                    m_NetworkTable->PutString("Controller", m_Controller ? m_Controller->GetName() : "None");
                    if (controller) controller->OnEnable();
                }
                return different;
            }

            virtual void AddController(std::shared_ptr<Controller> controller) {
                m_Controllers.push_back(controller);
            }

            void Reset() override {
                for (auto& controller : m_Controllers) {
                    controller->Reset();
                }
            }

        public:
            using Subsystem::Subsystem;
        };
    }
}