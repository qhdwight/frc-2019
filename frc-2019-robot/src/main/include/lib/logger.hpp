#pragma once

#include <string>

namespace garage {
    namespace lib {
        enum LogLevel {
            kSilent = 0, kFatal = 1, kError = 2, kWarning = 3, kInfo = 4, kDebug = 5, kVerbose = 6
        };
        class Logger {
        private:
            LogLevel m_LogLevel;
        public:
            void SetLogLevel(LogLevel logLevel);

            void Log(LogLevel logLevel, std::string log);
        };
    }
}