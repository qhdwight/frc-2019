#pragma once

#include <string>

namespace garage {
    namespace lib {
        enum class LogLevel {
            k_Silent = 0, k_Fatal = 1, k_Error = 2, k_Warning = 3, k_Info = 4, k_Debug = 5, k_Verbose = 6
        };

        class Logger {
        private:
            LogLevel m_LogLevel;
        public:
            std::string Format(std::string format, ...);

            void SetLogLevel(LogLevel logLevel);

            void Log(LogLevel logLevel, std::string log);
        };
    }
}