#include <lib/logger.hpp>

#include <wpi/raw_ostream.h>

#include <cstdarg>

namespace garage {
    namespace lib {
        Logger::LogLevel Logger::s_LogLevel = LogLevel::k_Info;

        std::string Logger::Format(const std::string& format, ...) {
            // TODO fixed size buffer for better performance?
            va_list arguments;
                    va_start(arguments, format);
            auto length = static_cast<size_t>(std::vsnprintf(nullptr, 0, FMT_STR(format), arguments));
                    va_end(arguments);
            std::vector<char> output(length + 1);
                    va_start(arguments, format);
            auto* front = output.data();
            std::vsnprintf(front, length + 1, FMT_STR(format), arguments);
                    va_end(arguments);
            return front;
        }

        void Logger::SetLogLevel(LogLevel logLevel) {
            s_LogLevel = logLevel;
        }

        void Logger::Log(LogLevel logLevel, std::string log) {
            if (logLevel <= s_LogLevel) {
                wpi::outs() << log << '\n';
            }
        }
    }
}