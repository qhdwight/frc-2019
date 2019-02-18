#include <lib/logger.hpp>

#include <wpi/raw_ostream.h>

namespace garage {
    namespace lib {
        void setLogLevel(LogLevel logLevel) {
            m_LogLevel = logLevel;
        }

        void log(LogLevel logLevel, std::string log) {
            if (logLevel <= m_LogLevel) {
                wpi::outs() << log << '\n';
            }
        }
    }
}