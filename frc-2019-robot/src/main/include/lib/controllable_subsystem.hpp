#pragma once

#include <robot.hpp>

#include <lib/subsystem.hpp>
#include <lib/subsystem_controller.hpp>

#include <memory>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class ControllableSubsystem : public std::enable_shared_from_this<TSubsystem>, public Subsystem {
        protected:
            std::vector<std::shared_ptr<SubsystemController<TSubsystem>>> m_Controllers;
            std::shared_ptr<SubsystemController<TSubsystem>> m_Controller;

            virtual bool SetController(std::shared_ptr<SubsystemController<TSubsystem>> controller) {
                const bool different = controller != m_Controller;
                if (different) {
                    if (m_Controller) m_Controller->OnDisable();
                    m_Controller = controller;
                    m_Robot->GetNetworkTable()->PutString("Controller", m_Controller ? m_Controller->GetName() : "None");
                    if (controller) controller->OnEnable();
                }
                return different;
            }

        public:
            using Subsystem::Subsystem;
        };
    }
}