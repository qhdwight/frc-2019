#include <cstdio>
#include <string>
#include <thread>
#include <mutex>
#include <vector>

#include <networktables/NetworkTableInstance.h>
#include <vision/VisionPipeline.h>
#include <vision/VisionRunner.h>
#include <wpi/StringRef.h>
#include <wpi/json.h>
#include <wpi/raw_istream.h>
#include <wpi/raw_ostream.h>

#include <cameraserver/CameraServer.h>

/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ]
           }
       ]
   }
 */

static const char* kConfigFile = "/boot/frc.json";

namespace {

    struct CameraConfig {
        std::string name;
        std::string path;
        wpi::json config;
    };

    unsigned int team;
    bool server = false;
    std::vector<CameraConfig> cameraConfigs;

    wpi::raw_ostream& ParseError() {
        return wpi::errs() << "config error in '" << kConfigFile << "': ";
    }

    bool ReadCameraConfig(const wpi::json& config) {
        CameraConfig cameraConfig;
        try {
            cameraConfig.name = config.at("name").get<std::string>();
        } catch (const wpi::json::exception& error) {
            ParseError() << "could not read camera name: " << error.what() << '\n';
            return false;
        }
        try {
            cameraConfig.path = config.at("path").get<std::string>();
        } catch (const wpi::json::exception& error) {
            ParseError() << "camera '" << cameraConfig.name << "': could not read path: " << error.what() << '\n';
            return false;
        }
        cameraConfig.config = config;
        cameraConfigs.emplace_back(std::move(cameraConfig));
        return true;
    }

    bool ReadConfig() {
        std::error_code errorCode;
        wpi::raw_fd_istream configStream(kConfigFile, errorCode);
        if (errorCode) {
            wpi::errs() << "could not open '" << kConfigFile << "': " << errorCode.message() << '\n';
            return false;
        }
        wpi::json config;
        try {
            config = wpi::json::parse(configStream);
        } catch (const wpi::json::parse_error& error) {
            ParseError() << "byte " << error.byte << ": " << error.what() << '\n';
            return false;
        }
        if (!config.is_object()) {
            ParseError() << "must be JSON object\n";
            return false;
        }
        try {
            team = config.at("team").get<unsigned int>();
        } catch (const wpi::json::exception& e) {
            ParseError() << "could not read team number: " << e.what() << '\n';
            return false;
        }
        if (config.count("ntmode") != 0) {
            try {
                auto str = config.at("ntmode").get<std::string>();
                wpi::StringRef s(str);
                if (s.equals_lower("client")) {
                    server = false;
                } else if (s.equals_lower("server")) {
                    server = true;
                } else {
                    ParseError() << "could not understand network mode value '" << str << "'\n";
                }
            } catch (const wpi::json::exception& e) {
                ParseError() << "could not read network mode: " << e.what() << '\n';
            }
        }
        try {
            for (auto&& camera : config.at("cameras")) {
                if (!ReadCameraConfig(camera)) return false;
            }
        } catch (const wpi::json::exception& e) {
            ParseError() << "could not read cameras: " << e.what() << '\n';
            return false;
        }
        return true;
    }

    cs::UsbCamera StartCamera(const CameraConfig& config) {
        wpi::outs() << "Starting camera '" << config.name << "' on " << config.path << '\n';
        auto camera = frc::CameraServer::GetInstance()->StartAutomaticCapture(config.name, config.path);
        camera.SetConfigJson(config.config);
        camera.SetConnectionStrategy(cs::VideoSource::kConnectionKeepOpen);
        return camera;
    }

    class HatchVisionPipeline : public frc::VisionPipeline {
    public:
        double count = 0.0;
        void Process(cv::Mat& matrix) override {
            count += 1.0;
        }
    };
}

int main(int argc, char* argv[]) {
    if (argc >= 2) kConfigFile = argv[1];
    if (!ReadConfig()) return EXIT_FAILURE;
    auto networkTable = nt::NetworkTableInstance::GetDefault();
    if (server) {
        wpi::outs() << "Setting up network tables server\n";
        networkTable.StartServer();
    } else {
        wpi::outs() << "Setting up network tables client for team " << team << '\n';
        networkTable.StartClientTeam(team);
    }
    std::vector<cs::VideoSource> cameras;
    for (auto&& cameraConfig : cameraConfigs)
        cameras.emplace_back(StartCamera(cameraConfig));
    std::thread visionThread;
    if (!cameras.empty()) {
        visionThread = std::thread([&] {
            auto entry = networkTable.GetEntry("vision_count");
            entry.SetDefaultDouble(0.0);
            frc::VisionRunner<HatchVisionPipeline> runner(cameras[0], new HatchVisionPipeline(),
                                                          [&](HatchVisionPipeline& pipeline) {
                                                                entry.SetDouble(pipeline.count);
                                                          });
            runner.RunForever();
        });
    }
    visionThread.join();
    return EXIT_SUCCESS;
}
