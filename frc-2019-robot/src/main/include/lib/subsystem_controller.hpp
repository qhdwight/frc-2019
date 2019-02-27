#pragma once

#include <command.hpp>

#include <lib/logger.hpp>
#include <lib/subsystem.hpp>

#include <string>
#include <memory>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class SubsystemController {
        protected:
            std::shared_ptr<TSubsystem> m_Subsystem;
            std::string m_Name;

        public:
            SubsystemController(std::shared_ptr<TSubsystem>& subsystem, const std::string& name)
                    : m_Subsystem(subsystem), m_Name(name) {
//                static_assert(std::is_base_of<Subsystem, TSubsystem>::value, "Must be a subsystem");
            };

            void Log(LogLevel logLevel, const std::string& log) {
                auto subsystem = std::dynamic_pointer_cast<Subsystem>(m_Subsystem);
                subsystem->Log(logLevel, subsystem->GetLogger()->Format(" [%s] %s", m_Name.c_str(), log.c_str()));
            }

            virtual void OnEnable() {
                Log(LogLevel::k_Info, "Enabled");
            }

            virtual void OnDisable() {
                Reset();
                Log(LogLevel::k_Info, "Disabled");
            }

            virtual void Reset() {};

            virtual void ProcessCommand(Command& command) {};

            virtual void Control() {};

            const std::string& GetName() const {
                return m_Name;
            }
        };
    }
}