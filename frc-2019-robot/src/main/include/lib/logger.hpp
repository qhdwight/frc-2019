#pragma once

#include <string>
#include <memory>

namespace garage {
    namespace lib {
        enum class LogLevel {
            k_Silent = 0, k_Fatal = 1, k_Error = 2, k_Warning = 3, k_Info = 4, k_Debug = 5, k_Verbose = 6
        };

        class Logger {
        private:
            LogLevel m_LogLevel;
        public:
            template<typename ... Args>
            std::string Format(const std::string& format, Args ... args) {
                auto size = static_cast<size_t>(std::snprintf(nullptr, 0, format.c_str(), args ...) + 1);
                std::unique_ptr<char[]> buffer(new char[size]);
                std::snprintf(buffer.get(), size, format.c_str(), args ...);
                return std::string(buffer.get(), buffer.get() + size - 1);
            }

            void SetLogLevel(LogLevel logLevel);

            void Log(LogLevel logLevel, const std::string& log);
        };
    }
}