#include <lib/auto_routine_from_csv.hpp>

#include <wpi/json.h>
#include <wpi/Path.h>
#include <wpi/FileSystem.h>
#include <wpi/raw_istream.h>

#include <frc/Filesystem.h>

namespace garage {
    namespace lib {
        void AutoRoutineFromCSV::PrepareWaypoints() {
            wpi::SmallString<PATH_LENGTH> fullPath;
            frc::filesystem::GetDeployDirectory(fullPath);
            wpi::sys::path::append(fullPath, m_Path);
            wpi::SmallString<PATH_LENGTH> leftPath(fullPath), rightPath(fullPath);
            leftPath.append(".left.pf1.csv");
            rightPath.append(".right.pf1.csv");
            auto* pathFile = std::fopen(fullPath.c_str(), "r");
            int count = 0;
            for (char c = static_cast<char>(std::getc(pathFile)); c != EOF; c = static_cast<char>(std::getc(pathFile))) {
                if (c == '\n') {
                    count++;
                }
            }
//            std::error_code errorCode;
//            wpi::raw_fd_istream pathFile(fullPath, errorCode);
//            if (errorCode) {
//                lib::Logger::Log(lib::Logger::LogLevel::k_Fatal,
//                                 lib::Logger::Format("Error reading path file: %s", FMT_STR(errorCode.message())));
//            } else {
//                try {
//
//                } catch (wpi::detail::parse_error& error) {
//                    lib::Logger::Log(lib::Logger::LogLevel::k_Fatal, lib::Logger::Format("Error parsing robot path: %s", error.what()));
//                }
//            }
            m_LeftTrajectory.reserve(count);
            m_RightTrajectory.reserve(count);
            pathfinder_deserialize_csv(pathFile, m_LeftTrajectory.data());
            pathfinder_deserialize_csv(pathFile, m_RightTrajectory.data());
            std::fclose(pathFile);
        }
    }
}