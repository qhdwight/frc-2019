#pragma once

#include <command.hpp>

#include <lib/logger.hpp>
#include <lib/subsystem.hpp>

#include <string>
#include <memory>
//#include <type_traits>

namespace garage {
    namespace lib {
        class Subsystem;

        template<typename TSubsystem>
        class SubsystemController {
        protected:
            std::shared_ptr<Subsystem> m_Subsystem;
            std::string m_Name;

        public:
            SubsystemController(std::shared_ptr<TSubsystem>& subsystem, const std::string& name)
                    : m_Subsystem(std::dynamic_pointer_cast<Subsystem>(subsystem)), m_Name(name) {
//                static_assert(std::is_base_of<Subsystem, TSubsystem>::value, "Must be a subsystem");
            };

            void Log(LogLevel logLevel, const std::string& log) {
                m_Subsystem->m_Robot->GetLogger()->Log(logLevel, m_Subsystem->m_Robot->GetLogger()->Format(" [%s] %s", m_Name.c_str(), log.c_str()));
            }

            virtual void OnEnable() {
                Log(LogLevel::k_Info, "Enabled");
            }

            virtual void OnDisable() {
                Log(LogLevel::k_Info, "Disabled");
            }

            virtual void Control(Command& command) {};
        };
    }
}