#pragma once

#include <command.hpp>

#include <lib/subsystem.hpp>

#include <string>
#include <memory>

namespace garage {
    namespace lib {
        template<typename TSubsystem>
        class SubsystemController {
        protected:
            std::weak_ptr<TSubsystem> m_Subsystem;
            std::string m_Name;

        public:
            SubsystemController(std::shared_ptr<TSubsystem>& subsystem, const std::string& name)
                    : m_Subsystem(subsystem), m_Name(name) {
//                static_assert(std::is_base_of<Subsystem, TSubsystem>::value, "Must be a subsystem");
            };

            void Log(Logger::LogLevel logLevel, const std::string& log) {
                auto subsystem = std::dynamic_pointer_cast<Subsystem>(m_Subsystem.lock());
                subsystem->Log(logLevel, Logger::Format(" [%s] %s", FMT_STR(m_Name), FMT_STR(log)));
            }

            virtual void OnEnable() {
                Log(Logger::LogLevel::k_Info, "Enabled");
            }

            virtual void OnDisable() {
                Reset();
                Log(Logger::LogLevel::k_Info, "Disabled");
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