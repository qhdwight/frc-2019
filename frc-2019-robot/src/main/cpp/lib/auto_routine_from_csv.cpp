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
            wpi::sys::path::append(fullPath, "output", m_Path);
            wpi::SmallString<PATH_LENGTH> leftPath(fullPath), rightPath(fullPath);
            leftPath.append(".left.pf1.csv");
            rightPath.append(".right.pf1.csv");
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
            ReadTrajectoryFromFile(leftPath.c_str(), m_LeftTrajectory);
            ReadTrajectoryFromFile(rightPath.c_str(), m_RightTrajectory);
        }

        void AutoRoutineFromCSV::ReadTrajectoryFromFile(const std::string& path, std::vector<Segment>& trajectory) {
            auto* pathFile = std::fopen(path.c_str(), "r");
            if (pathFile) {
                int count = 0, character = 0;
                while ((character = std::getc(pathFile)) != EOF) {
                    if (character == '\n') {
                        count++;
                    }
                }
                rewind(pathFile);
                // CSV has a one line header
                m_TrajectorySize = count - 1;
                Logger::Log(Logger::LogLevel::k_Info,
                            Logger::Format("Loaded path named: %s and path: %s with %d total segments",
                                           FMT_STR(m_Name), FMT_STR(path), m_TrajectorySize));
                trajectory.resize(m_TrajectorySize+1);
                pathfinder_deserialize_csv(pathFile, trajectory.data());
                std::fclose(pathFile);
            } else {
                Logger::Log(Logger::LogLevel::k_Error,
                            Logger::Format("Could not load path with name: %s and path: %s", FMT_STR(m_Name), FMT_STR(path)));
            }
//            for (auto& segment : trajectory) {
//                m_Subsystem->Log(Logger::LogLevel::k_Verbose, Logger::Format("%f, %f, %f, %f, %f, %f, %f, %f, %f",
//                                                                          segment.dt, segment.x, segment.y, segment.position, segment.velocity,
//                                                                          segment.acceleration, segment.jerk, segment.heading));
//            }
        }
    }
}