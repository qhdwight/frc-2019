#include <lib/logger.hpp>

#include <wpi/raw_ostream.h>

#include <cstdarg>

namespace garage {
    namespace lib {
        std::string Logger::Format(std::string format, ...) {
            va_list arguments;
            va_start(arguments, format);
            auto length = static_cast<size_t>(std::vsnprintf(nullptr, 0, format.c_str(), arguments));
            va_end(arguments);
            std::vector<char> output(length + 1);
            va_start(arguments, format);
            auto* front = output.data();
            std::vsnprintf(front, length + 1, format.c_str(), arguments);
            va_end(arguments);
            return front;
        }

        void Logger::SetLogLevel(LogLevel logLevel) {
            m_LogLevel = logLevel;
        }

        void Logger::Log(LogLevel logLevel, std::string log) {
            if (logLevel <= m_LogLevel) {
                wpi::outs() << log << '\n';
            }
        }
    }
}