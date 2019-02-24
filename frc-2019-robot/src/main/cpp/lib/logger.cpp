#include <lib/logger.hpp>

#include <wpi/raw_ostream.h>

#include <cstdarg>

namespace garage {
    namespace lib {
        void Logger::SetLogLevel(LogLevel logLevel) {
            m_LogLevel = logLevel;
        }

        void Logger::Log(LogLevel logLevel, const std::string& log) {
            if (logLevel <= m_LogLevel) {
                wpi::outs() << log << '\n';
            }
        }
    }
}