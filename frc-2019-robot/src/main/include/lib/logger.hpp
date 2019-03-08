#pragma once

#include <string>

#define FMT_STR(STRING) STRING.c_str()

namespace garage {
    namespace lib {
        class Logger {
        public:
            enum class LogLevel {
                k_Silent = 0, k_Fatal = 1, k_Error = 2, k_Warning = 3, k_Info = 4, k_Debug = 5, k_Verbose = 6
            };

        private:
            static LogLevel s_LogLevel;

        public:
            static std::string Format(const std::string& format, ...);

            static void SetLogLevel(LogLevel logLevel);

            static void Log(LogLevel logLevel, std::string log);
        };
    }
}